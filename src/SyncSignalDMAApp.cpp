#include "SyncSignalDMAApp.hpp"
#include <Arduino.h>

SyncSignalDMAApp::SyncSignalDMAApp(AdcDMAService& adcService, TransmitterDMAApp& transmitterApp, ReceiverDMAApp& receiverApp1, ReceiverDMAApp& receiverApp2)
    : _adcService(adcService), _transmitterApp(transmitterApp), _receiverApp1(receiverApp1), _receiverApp2(receiverApp2) {}

void SyncSignalDMAApp::init() {
    // Services and Apps are initialized externally or coordinated
}

void IRAM_ATTR SyncSignalDMAApp::runIteration(ComManager& com, uint16_t& frameId, double priMs) {
    // 1. Dừng ADC DMA trước để đưa về trạng thái tĩnh hoàn toàn
    _adcService.stop();

    // 2. Khởi động lại ADC DMA (chu kỳ mới)
    _adcService.start();
    uint64_t adcStartTime = esp_timer_get_time();

    // 3. Vào vùng chặn ngắt để phát DAC đồng bộ ngay lập tức (không delay)
    portMUX_TYPE myMutex = SPINLOCK_INITIALIZER;
    portENTER_CRITICAL(&myMutex);
    _transmitterApp.transmit(com.getPulseType());
    portEXIT_CRITICAL(&myMutex);

#ifdef SHOW_SAMPLING_LOG
    // In log đo chu kỳ phát DAC ngoài vùng critical section để tránh crash CPU
    _transmitterApp.printDacMetrics();
#endif

    // 4. Nhận và xử lý dữ liệu từ ADC DMA cho cả 2 kênh
    static uint16_t raw_interleaved_buffer[Constant::ADC_SAMPLES * 4];
    static uint16_t rx1_buffer[Constant::ADC_SAMPLES];
    static uint16_t rx2_buffer[Constant::ADC_SAMPLES];
    
    size_t bytes_read = 0;
    esp_err_t res = _adcService.readSamples(raw_interleaved_buffer, sizeof(raw_interleaved_buffer), bytes_read);
    uint64_t elapsed_time = esp_timer_get_time() - adcStartTime;

    if (res == ESP_OK && bytes_read == sizeof(raw_interleaved_buffer)) {
        size_t rx1_count = 0;
        size_t rx2_count = 0;
        uint8_t last_chan = 0xFF; // Để theo dõi trạng thái chuyển kênh không phụ thuộc phase

        for (size_t i = 4; i < Constant::ADC_SAMPLES * 4; ++i) {
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

#ifdef SHOW_SAMPLING_LOG
        // Debug first 20 samples channel IDs
        static int debug_cnt = 0;
        if (debug_cnt++ % 50 == 0) {
            Serial.printf("[DEBUG] Channel IDs: ");
            for (int i = 0; i < 20; ++i) {
                Serial.printf("%d ", (raw_interleaved_buffer[i] >> 12) & 0xF);
            }
            Serial.printf("\n[DEBUG] RX1 count (unique): %u, RX2 count (unique): %u\n", rx1_count, rx2_count);
        }
#endif

        // Xử lý dữ liệu độc lập cho từng Receiver với cùng một frameId
        _receiverApp1.process(rx1_buffer, com, frameId, priMs, elapsed_time);
        _receiverApp2.process(rx2_buffer, com, frameId, priMs, elapsed_time);

        // Tăng frameId cho chu kỳ phát xung tiếp theo
        frameId++;
    } else {
        Serial.printf("Lỗi đọc I2S: %d, số bytes đọc được: %u\n", res, bytes_read);
    }
}
