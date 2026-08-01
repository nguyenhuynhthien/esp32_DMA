#include "SyncSignalDMAApp.hpp"
#include <Arduino.h>

SyncSignalDMAApp::SyncSignalDMAApp(AdcDMAService& adcService, TransmitterDMAApp& transmitterApp, ReceiverDMAApp& receiverApp1, ReceiverDMAApp& receiverApp2)
    : _adcService(adcService), _transmitterApp(transmitterApp), _receiverApp1(receiverApp1), _receiverApp2(receiverApp2) {}

void SyncSignalDMAApp::init() {
    // Services and Apps are initialized externally or coordinated
}

void IRAM_ATTR SyncSignalDMAApp::runIteration(ComManager& com, uint16_t& frameId, double priMs) {
    // 1. Dừng ADC DMA trước để đưa về trạng thái tĩnh hoàn toàn (bỏ qua chu kỳ đầu tiên)
#ifdef SHOW_TIMING_LOG
    uint64_t t_start_stop = esp_timer_get_time();
#endif
    static bool first_call = true;
    if (!first_call) {
        _adcService.stop();
        // Trễ để phần cứng I2S ổn định hoàn toàn trạng thái tắt, tránh byte-swap (chớp dọc)
        delayMicroseconds(Constant::ADC_STOP_STABILIZATION_US);
    }
    first_call = false;

    // 2. Khỏng động lại ADC DMA (chu kỳ mới, SyncTask sở hữu Mutex từ đây)
    _adcService.start();
#ifdef SHOW_TIMING_LOG
    uint64_t stop_start_time = esp_timer_get_time() - t_start_stop;
#endif

    bool txEnabled = com.isTxEnabled();

    // Đồng bộ: Gửi Task Notification báo hiệu cho TxTask trên Core 0 phát xung ngay lập tức
    extern TaskHandle_t txTaskHandle;
    if (txTaskHandle != nullptr && txEnabled) {
        xTaskNotifyGive(txTaskHandle);
    }

#ifdef SHOW_SAMPLING_LOG
    uint64_t adc_start_time = esp_timer_get_time();
#else
    uint64_t adc_start_time = 0;
#endif

    // 3. Đọc loại xung
    ComManager::PulseType pulseType = com.getPulseType();

    // 4. Phát DAC đã được chuyển sang Core 0. Lấy thông số đo đạc từ các biến toàn cục.
    extern volatile double global_tx_pri_ms;
    extern volatile double global_tx_fs_khz;
    double tx_pri_ms = global_tx_pri_ms;
    double tx_fs_khz = global_tx_fs_khz;

    // 5. Nhận và xử lý dữ liệu từ ADC DMA cho cả 2 kênh
    static uint16_t raw_interleaved_buffer[Constant::ADC_SAMPLES * 4];
    static uint16_t rx1_buffer[Constant::ADC_SAMPLES];
    static uint16_t rx2_buffer[Constant::ADC_SAMPLES];
    
    size_t bytes_read = 0;
    esp_err_t res = _adcService.readSamples(raw_interleaved_buffer, sizeof(raw_interleaved_buffer), bytes_read);
    uint64_t elapsed_time = esp_timer_get_time() - adc_start_time;

    if (res == ESP_OK && bytes_read == sizeof(raw_interleaved_buffer)) {
        #ifdef SHOW_TIMING_LOG
        uint64_t t_start_demux = esp_timer_get_time();
        #endif
        size_t rx1_count = 0;
        size_t rx2_count = 0;
        uint8_t last_chan = 0xFF; // Để theo dõi trạng thái chuyển kênh không phụ thuộc phase

        uint16_t* p_rx1 = rx1_buffer;
        uint16_t* p_rx2 = rx2_buffer;
        const uint16_t* p_raw = raw_interleaved_buffer;
        const uint16_t* p_raw_end = raw_interleaved_buffer + (Constant::ADC_SAMPLES * 4);

        while (p_raw < p_raw_end) {
            uint16_t raw_val = *p_raw++;
            uint8_t chan = raw_val >> 12;
            if (chan != last_chan) {
                if (chan == 4) {
                    *p_rx1++ = raw_val;
                } else {
                    *p_rx2++ = raw_val;
                }
                last_chan = chan;
            }
        }
        rx1_count = p_rx1 - rx1_buffer;
        rx2_count = p_rx2 - rx2_buffer;

        // Điền mẫu cuối cùng nếu không nhận đủ số lượng mẫu yêu cầu
        uint16_t pad_val1 = (rx1_count > 0) ? (rx1_buffer[rx1_count - 1] & Constant::ADC_RESOLUTION_MAX) : 2048;
        while (rx1_count < Constant::ADC_SAMPLES) {
            rx1_buffer[rx1_count++] = pad_val1;
        }
        uint16_t pad_val2 = (rx2_count > 0) ? (rx2_buffer[rx2_count - 1] & Constant::ADC_RESOLUTION_MAX) : 2048;
        while (rx2_count < Constant::ADC_SAMPLES) {
            rx2_buffer[rx2_count++] = pad_val2;
        }
#ifdef SHOW_TIMING_LOG
        uint64_t demux_time = esp_timer_get_time() - t_start_demux;
#endif

        // Debug first 20 samples channel IDs
        static int debug_cnt = 0;
        if (debug_cnt++ % 50 == 0) {
            Serial.printf("[DEBUG] Channel IDs: ");
            for (int i = 0; i < 20; ++i) {
                Serial.printf("%d ", (raw_interleaved_buffer[i] >> 12) & 0xF);
            }
            Serial.printf("\n[DEBUG] RX1 count (unique): %u, RX2 count (unique): %u\n", rx1_count, rx2_count);
        }

        // Xử lý dữ liệu độc lập cho từng Receiver với cùng một frameId
#ifdef SHOW_TIMING_LOG
        uint64_t t_start_proc1 = esp_timer_get_time();
#endif
        _receiverApp1.process(rx1_buffer, com, frameId, priMs, tx_pri_ms, tx_fs_khz, elapsed_time, txEnabled);
#ifdef SHOW_TIMING_LOG
        uint64_t proc1_time = esp_timer_get_time() - t_start_proc1;
#endif

#ifdef SHOW_TIMING_LOG
        uint64_t t_start_proc2 = esp_timer_get_time();
#endif
        _receiverApp2.process(rx2_buffer, com, frameId, priMs, tx_pri_ms, tx_fs_khz, elapsed_time, txEnabled);
#ifdef SHOW_TIMING_LOG
        uint64_t proc2_time = esp_timer_get_time() - t_start_proc2;
#endif

#ifdef SHOW_TIMING_LOG
        if ((debug_cnt - 1) % 50 == 0) {
            Serial.printf("[TIMING] Tre_khoi_dong_ADC: %llu us | Tach_kenh: %llu us | Loc_Dongbo_Gui_Kenh1: %llu us | Loc_Dongbo_Gui_Kenh2: %llu us\n",
                          stop_start_time, demux_time, proc1_time, proc2_time);
        }
#endif

        // Tăng frameId cho chu kỳ phát xung tiếp theo
        frameId++;
    } else {
        Serial.printf("Lỗi đọc I2S: %d, số bytes đọc được: %u\n", res, bytes_read);
    }
}
