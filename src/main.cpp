#include <Arduino.h>
#include <ComManager.h>
#include <Constant.hpp>
#include <ESPmDNS.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <driver/adc.h>
#include <driver/dac.h>
#include <driver/i2s.h>
#include <soc/i2s_reg.h>
#include <soc/i2s_struct.h>

// Thông tin kết nối WiFi
const char *ssid = "Noel";
const char *password = "hongthanh2110";
const char *hostName = "esp32";

ComManager com(ssid, password, hostName);

// Inline assembly để đọc bộ đếm chu kỳ CPU (ccount)
static inline uint32_t get_ccount() {
  uint32_t ccount;
  asm volatile("rsr %0, ccount" : "=r"(ccount));
  return ccount;
}

// Xung sin phát DAC (Tổng cộng 12 mẫu)
// Chèn 4 mẫu bias 127 vào đầu để hấp thụ trễ khởi động của ADC RX (khoảng 25
// us) Sau đó là 2 chu kỳ sin hoàn chỉnh (8 mẫu) có biên độ giảm để tránh bão
// hòa ADC
const uint8_t sine_pulse[12] = {127, 127, 127, 127, 127, 190,
                                127, 64,  127, 190, 127, 64};

// Buffer chứa dữ liệu đọc từ DMA (I2S0)
uint16_t raw_adc_buffer[Constant::ADC_SAMPLES];
int16_t send_adc_buffer[Constant::ADC_SAMPLES];

void init_adc_i2s() {
  // Cấu hình độ phân giải và độ suy hao cho ADC1 Channel 4 (GPIO 32)
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_12);

  // Cấu hình Driver I2S0 cho chế độ ADC DMA với kích thước buffer nhỏ (64 mẫu)
  // để triệt tiêu độ trễ hàng đợi
  i2s_config_t i2s_config = {
      .mode =
          (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
      .sample_rate = 160000, // fs = 160 kHz
      .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
      .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 2, 0)
      .communication_format = I2S_COMM_FORMAT_STAND_I2S,
#else
      .communication_format = I2S_COMM_FORMAT_I2S_MSB,
#endif
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
      .dma_buf_count = 8, // Tăng số lượng buffer
      .dma_buf_len = 64,  // Giảm kích thước mỗi buffer để giảm trễ
      .use_apll = false,
      .tx_desc_auto_clear = false,
      .fixed_mclk = 0};

  // Khởi tạo Driver I2S
  esp_err_t err = i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  if (err != ESP_OK) {
    Serial.printf("Lỗi cài đặt driver I2S: %d\n", err);
  }

  // Gán kênh ADC1 vào I2S
  err = i2s_set_adc_mode(ADC_UNIT_1, ADC1_CHANNEL_4);
  if (err != ESP_OK) {
    Serial.printf("Lỗi cấu hình kênh ADC cho I2S: %d\n", err);
  }

  // Bật ADC trên I2S
  err = i2s_adc_enable(I2S_NUM_0);
  if (err != ESP_OK) {
    Serial.printf("Lỗi kích hoạt I2S ADC: %d\n", err);
  }

  Serial.println("Cấu hình ADC I2S DMA hoàn tất.");
}

void fire_dac_pulse() {
  // Phát xung sin qua DAC1 (GPIO 25)
  uint32_t start_cycles = get_ccount();
  uint32_t cpu_freq_mhz = ESP.getCpuFreqMHz();
  // Tần số fs = 160 kHz tương ứng 6.25 us hoặc (cpu_freq_mhz * 6.25) chu kỳ CPU
  uint32_t cycles_per_sample = (uint32_t)(cpu_freq_mhz * 6.25f);

  for (int i = 0; i < 12; ++i) {
    dac_output_voltage(DAC_CHANNEL_1, sine_pulse[i]);
    // Chờ chính xác thời gian mẫu tiếp theo bằng ccount
    while ((int32_t)(get_ccount() -
                     (start_cycles + (i + 1) * cycles_per_sample)) < 0) {
      // Chờ spin-wait
    }
  }
  // Trả về mức bias sau khi phát xung xong
  dac_output_voltage(DAC_CHANNEL_1, 127);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Khởi động hệ thống test_DMA...");

  // Cấu hình chân DAC
  dac_output_enable(DAC_CHANNEL_1);
  dac_output_voltage(DAC_CHANNEL_1, 127); // Đặt bias ban đầu

  // 1. Khởi tạo ComManager để kết nối WiFi trước khi cấu hình ADC
  com.begin();

  // 2. Khởi tạo ADC DMA qua I2S0 sau khi WiFi đã kết nối
  // Sau khi cài đặt, driver I2S sẽ ở trạng thái đang chạy (Running) mặc định
  init_adc_i2s();

  // Nâng ưu tiên của loopTask lên 20 (cao hơn WiFi/mạng) để triệt tiêu Jitter
  // khi lập lịch
  vTaskPrioritySet(NULL, 20);
}

uint16_t frameId = 0;
uint32_t loopCount = 0;

void loop() {
  // Xử lý các gói tin UDP và cập nhật trạng thái kết nối
  com.update();

  if (com.isStreaming()) {
    static uint64_t last_time = 0;
    uint64_t current_time = esp_timer_get_time();
    double pri_ms = 0.0;
    if (last_time != 0) {
      pri_ms = (double)(current_time - last_time) / 1000.0;
    }
    last_time = current_time;

    // 1. Luôn dừng I2S trước để đưa về trạng thái tĩnh hoàn toàn
    // (Do ở cuối loop và ở setup ta không tắt I2S nên chắc chắn Driver đang ở
    // trạng thái Running, đảm bảo lệnh stop này luôn hợp lệ và không lỗi trạng
    // thái)
    i2s_adc_disable(I2S_NUM_0);
    i2s_stop(I2S_NUM_0);

    // Thêm trễ ngắn 2ms để phần cứng I2S và bộ điều khiển DMA hoàn tất việc
    // reset và giải phóng bộ nhớ

    // 2. Khởi động lại I2S và bật ADC (chu kỳ mới)
    i2s_start(I2S_NUM_0);
    i2s_adc_enable(I2S_NUM_0);

    // 3. Vào vùng chặn ngắt để phát DAC đồng bộ ngay lập tức (không delay,
    // không drain) Điều này đảm bảo không có mẫu rác nào kịp ghi vào DMA, triệt
    // tiêu trễ ngẫu nhiên
    portMUX_TYPE myMutex = SPINLOCK_INITIALIZER;
    portENTER_CRITICAL(&myMutex);
    fire_dac_pulse();
    portEXIT_CRITICAL(&myMutex);

    // Đo thời gian bắt đầu đọc dữ liệu DMA
    uint64_t start_time = esp_timer_get_time();
    size_t bytes_read = 0;

    // Đọc 2048 mẫu (4096 bytes) từ I2S DMA
    esp_err_t res = i2s_read(I2S_NUM_0, raw_adc_buffer, sizeof(raw_adc_buffer),
                             &bytes_read, portMAX_DELAY);
    uint64_t elapsed_time = esp_timer_get_time() - start_time;

    if (res == ESP_OK && bytes_read == sizeof(raw_adc_buffer)) {
      // Tính toán tần số lấy mẫu thực tế dựa trên thời gian thực tế thu nhận
      double fs_actual =
          (double)(Constant::ADC_SAMPLES) * 1000000.0 / (double)elapsed_time;

      // Tính giá trị trung bình (DC bias) của buffer thô
      int32_t sum = 0;
      for (size_t i = 0; i < Constant::ADC_SAMPLES; ++i) {
        // Mask lấy 12-bit từ dữ liệu đọc được từ I2S
        raw_adc_buffer[i] &= 0x0FFF;
        sum += raw_adc_buffer[i];
      }
      int16_t mean = sum / Constant::ADC_SAMPLES;

      // Loại bỏ DC offset và chuẩn hóa biên độ về dải Q15 (signed 16-bit)
      for (size_t i = 0; i < Constant::ADC_SAMPLES; ++i) {
        int32_t centered = ((int32_t)raw_adc_buffer[i] - mean) << 4;
        send_adc_buffer[i] =
            (int16_t)constrain(centered, Constant::Q15_MIN, Constant::Q15_MAX);
      }

      // Đồng bộ hóa phần mềm để loại bỏ jitter:
      // 1. Tìm giá trị dương lớn nhất trong cửa sổ rộng (bỏ qua mẫu 0, 1) để
      // làm mốc biên độ
      volatile int16_t max_val = 0;
      for (int i = 2; i < 120; ++i) {
        int16_t val = send_adc_buffer[i];
        if (val > max_val) {
          max_val = val;
        }
      }

      // 2. Tìm đỉnh cục bộ dương ĐẦU TIÊN vượt quá 45% của max_val để tránh
      // hiện tượng nhảy chu kỳ (cycle jumping)
      volatile int peak_idx = 12; // Mặc định nếu không tìm thấy
      volatile int16_t threshold = (max_val * 45) / 100;
      for (int i = 2; i < 118; ++i) {
        int16_t val = send_adc_buffer[i];
        if (val >= threshold && val >= send_adc_buffer[i - 1] &&
            val >= send_adc_buffer[i + 1]) {
          peak_idx = i;
          break; // Lấy đỉnh cục bộ đầu tiên thỏa mãn
        }
      }

      // 3. Điểm căn chỉnh tham chiếu và tính toán độ dịch (shift)
      // Đặt REF_PEAK_IDX = 1 để dịch đỉnh về mẫu 1 (loại bỏ trễ và peak giả ở
      // mẫu 0)
      const int REF_PEAK_IDX = 1;
      volatile int shift = peak_idx - REF_PEAK_IDX;

      // Giới hạn dịch chuyển để tránh lỗi mảng
      if (shift > 50)
        shift = 50;
      if (shift < -50)
        shift = -50;

      // 4. Căn chỉnh lại mảng tín hiệu và điền 0 vào phần trống để đảm bảo luôn
      // đủ 2048 mẫu
      if (shift > 0) {
        // Dịch trái
        for (size_t i = 0; i < Constant::ADC_SAMPLES - shift; ++i) {
          send_adc_buffer[i] = send_adc_buffer[i + shift];
        }
        for (size_t i = Constant::ADC_SAMPLES - shift;
             i < Constant::ADC_SAMPLES; ++i) {
          send_adc_buffer[i] = 0;
        }
      } else if (shift < 0) {
        // Dịch phải
        int rshift = -shift;
        for (int i = (int)Constant::ADC_SAMPLES - 1; i >= rshift; --i) {
          send_adc_buffer[i] = send_adc_buffer[i - rshift];
        }
        for (int i = 0; i < rshift; ++i) {
          send_adc_buffer[i] = 0;
        }
      }

      // In log mỗi chu kỳ để theo dõi trực quan và kiểm tra căn chỉnh liên tục
      loopCount++;
      if (loopCount % 50 == 0) {
        Serial.printf("[LOG] PRI: %.2f ms | Số mẫu: %u | Tần số lấy mẫu thực "
                      "tế: %.2f kHz (Đọc trong %llu us)\n",
                      pri_ms, Constant::ADC_SAMPLES, fs_actual / 1000.0,
                      elapsed_time);
      }
      // Gửi dữ liệu qua giao thức UDP của ComManager đến SonarViewer dưới định
      // dạng kênh Rx1 (receiverId = 1)
      com.sendFrame(frameId++, send_adc_buffer, Constant::ADC_SAMPLES, 1);
    } else {
      Serial.printf("Lỗi đọc I2S: %d, số bytes đọc được: %u\n", res,
                    bytes_read);
    }

    // Tốc độ lặp (PRI) khoảng 30 ms
    delay(30);
  } else {
    // Chờ kết nối
    delay(100);
  }
}
