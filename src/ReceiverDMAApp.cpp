#include "ReceiverDMAApp.hpp"
#include <Arduino.h>

#ifdef SHOW_SAMPLING_LOG
ReceiverDMAApp::ReceiverDMAApp(AdcDMAService& adcService, uint8_t receiverId)
    : _adcService(adcService), _receiverId(receiverId), _loopCount(0) {}
#else
ReceiverDMAApp::ReceiverDMAApp(AdcDMAService& adcService, uint8_t receiverId)
    : _adcService(adcService), _receiverId(receiverId) {}
#endif

void ReceiverDMAApp::init() {
    // Buffers or additional setup if needed
}

void ReceiverDMAApp::receiveAndProcess(ComManager& com, uint16_t frameId, double priMs, uint64_t adcStartTime) {
    uint64_t start_time = (adcStartTime != 0) ? adcStartTime : esp_timer_get_time();
    size_t bytes_read = 0;

    esp_err_t res = _adcService.readSamples(_raw_adc_buffer, sizeof(_raw_adc_buffer), bytes_read);
    uint64_t elapsed_time = esp_timer_get_time() - start_time;

    if (res == ESP_OK && bytes_read == sizeof(_raw_adc_buffer)) {
        process(_raw_adc_buffer, com, frameId, priMs, elapsed_time);
    } else {
        Serial.printf("Lỗi đọc I2S: %d, số bytes đọc được: %u\n", res, bytes_read);
    }
}

void ReceiverDMAApp::process(const uint16_t* rawSamples, ComManager& com, uint16_t frameId, double priMs, uint64_t elapsed_time, SimulatorDMAApp* simulatorApp) {
    // Sao chép buffer thô vào bộ nhớ cục bộ
    memcpy(_raw_adc_buffer, rawSamples, Constant::ADC_SAMPLES * sizeof(uint16_t));

#ifdef SHOW_SAMPLING_LOG
    // Tính toán tần số lấy mẫu thực tế dựa trên thời gian thực tế thu nhận
    double fs_actual = (double)(Constant::ADC_SAMPLES) * 1000000.0 / (double)elapsed_time;
#endif

    // Tính giá trị trung bình (DC bias) của buffer thô
    int32_t sum = 0;
    for (size_t i = 0; i < Constant::ADC_SAMPLES; ++i) {
        // Mask lấy 12-bit từ dữ liệu đọc được từ I2S
        _raw_adc_buffer[i] &= Constant::ADC_RESOLUTION_MAX;
        sum += _raw_adc_buffer[i];
    }
    // Tối ưu hóa phép chia cho 2048 bằng phép dịch bit phải 11
    int16_t mean = sum >> Constant::ADC_SAMPLES_SHIFT;

    // Chỉ cần chuẩn hóa cửa sổ đầu tiên để tìm đỉnh cục bộ phục vụ căn chỉnh
    for (int i = 0; i < Constant::JITTER_WINDOW_LEN; ++i) {
        int32_t centered = ((int32_t)_raw_adc_buffer[i] - mean) << Constant::Q15_SCALE_SHIFT;
        _send_adc_buffer[i] = (int16_t)constrain(centered, Constant::Q15_MIN, Constant::Q15_MAX);
    }

    // Đồng bộ hóa phần mềm để loại bỏ jitter:
    // 1. Tìm giá trị dương lớn nhất trong cửa sổ rộng (bỏ qua mẫu 0, 1) để làm mốc biên độ
    volatile int16_t max_val = 0;
    for (int i = Constant::JITTER_SEARCH_START_IDX; i < Constant::JITTER_WINDOW_LEN; ++i) {
        int16_t val = _send_adc_buffer[i];
        if (val > max_val) {
            max_val = val;
        }
    }

    // 2. Tìm đỉnh cục bộ dương ĐẦU TIÊN vượt quá 45% của max_val để tránh hiện tượng nhảy chu kỳ (cycle jumping)
    volatile int peak_idx = Constant::DEFAULT_PEAK_IDX; // Mặc định nếu không tìm thấy
    volatile int16_t threshold = (max_val * Constant::PEAK_THRESHOLD_PERCENT) / 100;
    for (int i = Constant::JITTER_SEARCH_START_IDX; i < Constant::JITTER_WINDOW_LEN - Constant::JITTER_SEARCH_START_IDX; ++i) {
        int16_t val = _send_adc_buffer[i];
        if (val >= threshold && val >= _send_adc_buffer[i - 1] && val >= _send_adc_buffer[i + 1]) {
            peak_idx = i;
            break; // Lấy đỉnh cục bộ đầu tiên thỏa mãn
        }
    }

    // 3. Điểm căn chỉnh tham chiếu và tính toán độ dịch (shift)
    const int REF_PEAK_IDX = Constant::REF_PEAK_IDX;
    volatile int shift = peak_idx - REF_PEAK_IDX;

    // Giới hạn dịch chuyển để tránh lỗi mảng
    if (shift > Constant::MAX_ALLOWED_SHIFT)
        shift = Constant::MAX_ALLOWED_SHIFT;
    if (shift < -Constant::MAX_ALLOWED_SHIFT)
        shift = -Constant::MAX_ALLOWED_SHIFT;

    // 4. Căn chỉnh lại mảng tín hiệu và điền 0 vào phần trống để đảm bảo luôn đủ 2048 mẫu.
    if (shift > 0) {
        // Dịch trái
        for (size_t i = 0; i < Constant::ADC_SAMPLES - shift; ++i) {
            int32_t centered = ((int32_t)_raw_adc_buffer[i + shift] - mean) << Constant::Q15_SCALE_SHIFT;
            _send_adc_buffer[i] = (int16_t)constrain(centered, Constant::Q15_MIN, Constant::Q15_MAX);
        }
        // Xóa nhanh vùng cuối mảng bằng memset
        memset(&_send_adc_buffer[Constant::ADC_SAMPLES - shift], 0, shift * sizeof(int16_t));
    } else {
        // Dịch phải (shift <= 0)
        int rshift = -shift;
        for (int i = (int)Constant::ADC_SAMPLES - 1; i >= rshift; --i) {
            int32_t centered = ((int32_t)_raw_adc_buffer[i - rshift] - mean) << Constant::Q15_SCALE_SHIFT;
            _send_adc_buffer[i] = (int16_t)constrain(centered, Constant::Q15_MIN, Constant::Q15_MAX);
        }
        // Xóa nhanh vùng đầu mảng bằng memset
        memset(_send_adc_buffer, 0, rshift * sizeof(int16_t));
    }

#ifdef SIMULATION_MODE
    // Tiêm xung giả lập vào mảng dữ liệu đã đồng bộ
    if (simulatorApp != nullptr) {
        simulatorApp->injectSimulationQ15(_send_adc_buffer, Constant::ADC_SAMPLES, com.getPulseType(), frameId, priMs);
    }
#endif

#ifdef SHOW_SAMPLING_LOG
    _loopCount++;
    if (_loopCount % Constant::LOG_INTERVAL_FRAMES == 0) {
        Serial.printf("[LOG RX%d] PRI: %.2f ms | Số mẫu: %u | Tần số lấy mẫu thực tế: %.2f kHz (Đọc trong %llu us)\n",
                      _receiverId, priMs, Constant::ADC_SAMPLES, fs_actual / 1000.0, elapsed_time);
        Serial.printf("[DEBUG RX%d] peak_idx: %d, shift: %d, max_val: %d, mean: %d\n", 
                      _receiverId, peak_idx, shift, max_val, mean);
    }
#endif

    // Gửi dữ liệu qua giao thức UDP của ComManager đến SonarViewer dưới định dạng kênh tương ứng
    com.sendFrame(frameId, _send_adc_buffer, Constant::ADC_SAMPLES, _receiverId);
}
