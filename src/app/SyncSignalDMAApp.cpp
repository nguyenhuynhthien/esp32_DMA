#include "SyncSignalDMAApp.hpp"
#include <Arduino.h>

SyncSignalDMAApp::SyncSignalDMAApp(AdcDMAService& adcService, TransmitterDMAApp& transmitterApp, ReceiverDMAApp& receiverApp1, ReceiverDMAApp& receiverApp2, SimulatorDMAApp& simulatorApp)
    : _adcService(adcService), _transmitterApp(transmitterApp), _receiverApp1(receiverApp1), _receiverApp2(receiverApp2), _simulatorApp(simulatorApp) {}

void SyncSignalDMAApp::init() {
    // Services and Apps are initialized externally or coordinated
}

void IRAM_ATTR SyncSignalDMAApp::runIteration(ComManager& com, uint16_t& frameId, double priMs, QueueHandle_t udpQueue) {
#ifdef SHOW_SAMPLING_LOG
    uint64_t t_start = esp_timer_get_time();
#ifdef TRACE_TASK_TIMING
    Serial.printf("[TRACE][CORE %d][SYNC] frame=%u enter pri=%.2f ms\n",
                  xPortGetCoreID(), frameId, priMs);
#endif
#endif

    // 1. Dừng ADC DMA trước để đưa về trạng thái tĩnh hoàn toàn
    _adcService.stop();

    // 2. Khởi động lại ADC DMA (chu kỳ mới)
    _adcService.start();

#ifdef SHOW_SAMPLING_LOG
    uint64_t t_after_adc_init = esp_timer_get_time();
#endif

    // 3. Chỉ phát DAC khi Tx On; còn lại vẫn giữ chu kỳ thu để viewer/UDP tiếp tục chạy
    bool txEnabled = com.isTxEnabled();
    if (txEnabled) {
        portMUX_TYPE myMutex = SPINLOCK_INITIALIZER;
        portENTER_CRITICAL(&myMutex);
        _transmitterApp.transmit(com.getPulseType());
        portEXIT_CRITICAL(&myMutex);
    }

#ifdef SHOW_SAMPLING_LOG
    uint64_t t_after_dac = esp_timer_get_time();
#endif

    uint64_t adcStartTime = esp_timer_get_time();

#ifdef SHOW_SAMPLING_LOG
    // In log đo chu kỳ phát DAC chỉ khi thật sự có phát xung
    if (txEnabled) {
        _transmitterApp.printDacMetrics(priMs);
    }
#endif

    // 4. Nhận và xử lý dữ liệu từ ADC DMA cho cả 2 kênh
    static uint16_t raw_interleaved_buffer[Constant::ADC_SAMPLES * 4];
    static uint16_t rx1_buffer[Constant::ADC_SAMPLES];
    static uint16_t rx2_buffer[Constant::ADC_SAMPLES];
    
    size_t bytes_read = 0;
    esp_err_t res = _adcService.readSamples(raw_interleaved_buffer, sizeof(raw_interleaved_buffer), bytes_read);
    uint64_t elapsed_time = esp_timer_get_time() - adcStartTime;

#ifdef SHOW_SAMPLING_LOG
    uint64_t t_after_read = esp_timer_get_time();
#endif

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
        if (debug_cnt++ % Constant::LOG_INTERVAL_FRAMES == 0) {
            Serial.printf("[DEBUG] Channel IDs: ");
            for (int i = 0; i < 20; ++i) {
                Serial.printf("%d ", (raw_interleaved_buffer[i] >> 12) & 0xF);
            }
            Serial.printf("\n[DEBUG] RX1 count (unique): %u, RX2 count (unique): %u\n", rx1_count, rx2_count);
        }
        uint64_t t_after_demux = esp_timer_get_time();
#endif

        // Xử lý dữ liệu độc lập cho từng Receiver với cùng một frameId
        _receiverApp1.process(rx1_buffer, com, frameId, priMs, elapsed_time, udpQueue, &_simulatorApp, txEnabled);
        _receiverApp2.process(rx2_buffer, com, frameId, priMs, elapsed_time, udpQueue, &_simulatorApp, txEnabled);

#ifdef SHOW_SAMPLING_LOG
    uint64_t t_end = esp_timer_get_time();
    static int profile_cnt = 0;
    if (profile_cnt++ % Constant::LOG_INTERVAL_FRAMES == 0) {
        Serial.printf("[PROFILE] ADC Restart: %llu us | DAC Transmit: %llu us | ADC Read DMA: %llu us | Demux & Pad: %llu us | RX Apps Process: %llu us | Total Run: %llu us\n",
                      t_after_adc_init - t_start,
                      t_after_dac - t_after_adc_init,
                      t_after_read - t_after_dac,
                      t_after_demux - t_after_read,
                      t_end - t_after_demux,
                      t_end - t_start);
    }
#ifdef TRACE_TASK_TIMING
    Serial.printf("[TRACE][CORE %d][SYNC] frame=%u total=%llu us\n",
                  xPortGetCoreID(), frameId, t_end - t_start);
#endif
#endif

        // Tăng frameId cho chu kỳ phát xung tiếp theo
        frameId++;
    } else {
        Serial.printf("Lỗi đọc I2S: %d, số bytes đọc được: %u\n", res, bytes_read);
    }
}
