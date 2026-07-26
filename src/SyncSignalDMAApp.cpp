#include "SyncSignalDMAApp.hpp"
#include <Arduino.h>

SyncSignalDMAApp::SyncSignalDMAApp(AdcDMAService& adcService, TransmitterDMAApp& transmitterApp, ReceiverDMAApp& receiverApp1, ReceiverDMAApp& receiverApp2)
    : _adcService(adcService), _transmitterApp(transmitterApp), _receiverApp1(receiverApp1), _receiverApp2(receiverApp2) {}

void SyncSignalDMAApp::init() {
    // Services and Apps are initialized externally or coordinated
}

void IRAM_ATTR SyncSignalDMAApp::runIteration(ComManager& com, uint16_t& frameId, double priMs) {
    // 1. Dừng ADC DMA trước để đưa về trạng thái tĩnh hoàn toàn (bỏ qua chu kỳ đầu tiên)
    static bool first_call = true;
    if (!first_call) {
        _adcService.stop();
        // Trễ để phần cứng I2S ổn định hoàn toàn trạng thái tắt, tránh byte-swap (chớp dọc)
        delayMicroseconds(Constant::ADC_STOP_STABILIZATION_US);
    }
    first_call = false;

    // 2. Khởi động lại ADC DMA (chu kỳ mới, SyncTask sở hữu Mutex từ đây)
    _adcService.start();
#ifdef SHOW_SAMPLING_LOG
    uint64_t adc_start_time = esp_timer_get_time();
#else
    uint64_t adc_start_time = 0;
#endif

    // 3. Đọc loại xung
    ComManager::PulseType pulseType = com.getPulseType();

    // 4. Phát DAC đồng bộ ngay lập tức và đo thời gian phát (us)
#ifdef SHOW_SAMPLING_LOG
    // Đo Tx PRI
    static uint64_t last_tx_time = 0;
    uint64_t current_tx_time = esp_timer_get_time();
    double tx_pri_ms = 0.0;
    if (last_tx_time != 0) {
        tx_pri_ms = (double)(current_tx_time - last_tx_time) / 1000.0;
    }
    last_tx_time = current_tx_time;

    uint32_t tx_cycles = _transmitterApp.transmit(pulseType);
    
    // Tính số lượng mẫu phát
    size_t tx_len = (pulseType == ComManager::PULSE_SINGLE) 
                    ? (Constant::FILTER_COEFFS_LEN + Constant::DAC_PULSE_TOTAL_PADDING)
                    : (Constant::BARKER13_PULSE_LEN + Constant::DAC_PULSE_TOTAL_PADDING);
    
    // Đổi chu kỳ CPU sang us
    double tx_elapsed_us = (double)tx_cycles / (double)ESP.getCpuFreqMHz();

    // Tx Fs = Số mẫu * 1000.0 / Thời gian phát (kHz)
    double tx_fs_khz = 0.0;
    if (tx_elapsed_us > 0) {
        tx_fs_khz = (double)tx_len * 1000.0 / tx_elapsed_us;
    }
#else
    _transmitterApp.transmit(pulseType);
    double tx_pri_ms = 0.0;
    double tx_fs_khz = 0.0;
#endif

    // 5. Nhận và xử lý dữ liệu từ ADC DMA cho cả 2 kênh
    static uint16_t raw_interleaved_buffer[Constant::ADC_SAMPLES * 4];
    static uint16_t rx1_buffer[Constant::ADC_SAMPLES];
    static uint16_t rx2_buffer[Constant::ADC_SAMPLES];
    
    size_t bytes_read = 0;
    esp_err_t res = _adcService.readSamples(raw_interleaved_buffer, sizeof(raw_interleaved_buffer), bytes_read);
    uint64_t elapsed_time = esp_timer_get_time() - adc_start_time;

    if (res == ESP_OK && bytes_read == sizeof(raw_interleaved_buffer)) {
        size_t rx1_count = 0;
        size_t rx2_count = 0;
        uint8_t last_chan = 0xFF; // Để theo dõi trạng thái chuyển kênh không phụ thuộc phase

        for (size_t i = 0; i < Constant::ADC_SAMPLES * 4; ++i) {
            uint16_t raw_val = raw_interleaved_buffer[i];
            uint8_t chan = (raw_val >> 12) & 0xF;
            if (chan == Constant::ADC_CHANNEL_RX1 || chan == Constant::ADC_CHANNEL_RX2) {
                if (chan != last_chan) {
                    if (chan == Constant::ADC_CHANNEL_RX1) {
                        if (rx1_count < Constant::ADC_SAMPLES) {
                            rx1_buffer[rx1_count++] = raw_val;
                        }
                    } else {
                        if (rx2_count < Constant::ADC_SAMPLES) {
                            rx2_buffer[rx2_count++] = raw_val;
                        }
                    }
                    last_chan = chan;
                }
            }
        }

        // Điền mẫu cuối cùng nếu không nhận đủ số lượng mẫu yêu cầu
        uint16_t pad_val1 = (rx1_count > 0) ? (rx1_buffer[rx1_count - 1] & Constant::ADC_RESOLUTION_MAX) : 2048;
        while (rx1_count < Constant::ADC_SAMPLES) {
            rx1_buffer[rx1_count++] = pad_val1;
        }
        uint16_t pad_val2 = (rx2_count > 0) ? (rx2_buffer[rx2_count - 1] & Constant::ADC_RESOLUTION_MAX) : 2048;
        while (rx2_count < Constant::ADC_SAMPLES) {
            rx2_buffer[rx2_count++] = pad_val2;
        }

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
        _receiverApp1.process(rx1_buffer, com, frameId, priMs, tx_pri_ms, tx_fs_khz, elapsed_time);
        _receiverApp2.process(rx2_buffer, com, frameId, priMs, tx_pri_ms, tx_fs_khz, elapsed_time);

        // Tăng frameId cho chu kỳ phát xung tiếp theo
        frameId++;
    } else {
        Serial.printf("Lỗi đọc I2S: %d, số bytes đọc được: %u\n", res, bytes_read);
    }
}
