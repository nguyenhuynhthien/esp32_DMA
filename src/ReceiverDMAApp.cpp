#include "ReceiverDMAApp.hpp"
#include <Arduino.h>

ReceiverDMAApp::ReceiverDMAApp(AdcDMAService& adcService) : _adcService(adcService) {}

void ReceiverDMAApp::init() {
    // Buffers or additional setup if needed
}

int16_t ReceiverDMAApp::calculateDcBias() {
    int32_t sum = 0;
    for (size_t i = 0; i < Constant::ADC_SAMPLES; ++i) {
        _raw_adc_buffer[i] &= Constant::ADC_RESOLUTION_MAX;
        sum += _raw_adc_buffer[i];
    }
    return sum >> Constant::ADC_SAMPLES_SHIFT;
}

void ReceiverDMAApp::processRawBuffer(int16_t mean) {
    for (int i = 0; i < Constant::JITTER_WINDOW_LEN; ++i) {
        int32_t centered = ((int32_t)_raw_adc_buffer[i] - mean) << Constant::Q15_SCALE_SHIFT;
        _send_adc_buffer[i] = (int16_t)constrain(centered, Constant::Q15_MIN, Constant::Q15_MAX);
    }
}

void ReceiverDMAApp::applyIirFilter() {
    static int16_t temp_smooth[Constant::ADC_SAMPLES];
    temp_smooth[0] = _send_adc_buffer[0];
    for (int i = 1; i < Constant::ADC_SAMPLES; ++i) {
        temp_smooth[i] = (int16_t)(Constant::SIGNAL_SMOOTH_ALPHA * (float)_send_adc_buffer[i] + 
                                     Constant::SIGNAL_SMOOTH_BETA * (float)temp_smooth[i - 1]);
    }
    memcpy(_send_adc_buffer, temp_smooth, Constant::ADC_SAMPLES * sizeof(int16_t));
}

int ReceiverDMAApp::findSyncPeak() {
    int peak_idx = -1;
    // Bắt đầu từ i = 2 để bỏ qua hoàn toàn gai nhiễu chuyển mạch ban đầu (0-1).
    for (int i = 2; i < Constant::SYNC_SEARCH_LEN; ++i) {
        if (_send_adc_buffer[i] > Constant::THRESHOLD_SYNC) {
            peak_idx = i;
            break; // Tìm thấy mẫu đầu tiên thì dừng ngay
        }
    }
    return peak_idx;
}

void ReceiverDMAApp::shiftSignal(int shift) {
    if (shift > 0) {
        // Dịch trái
        for (size_t i = 0; i < Constant::ADC_SAMPLES - shift; ++i) {
            _send_adc_buffer[i] = _send_adc_buffer[i + shift];
        }
        memset(&_send_adc_buffer[Constant::ADC_SAMPLES - shift], 0, shift * sizeof(int16_t));
    } else {
        // Dịch phải (shift <= 0)
        int rshift = -shift;
        for (int i = (int)Constant::ADC_SAMPLES - 1; i >= rshift; --i) {
            _send_adc_buffer[i] = _send_adc_buffer[i - rshift];
        }
        memset(_send_adc_buffer, 0, rshift * sizeof(int16_t));
    }
}

void ReceiverDMAApp::receiveAndProcess(ComManager& com, uint16_t& frameId, double priMs, double txPriMs, double txFsKhz) {
    uint64_t start_time = esp_timer_get_time();
    size_t bytes_read = 0;

    esp_err_t res = _adcService.readSamples(_raw_adc_buffer, sizeof(_raw_adc_buffer), bytes_read);
    uint64_t elapsed_time = esp_timer_get_time() - start_time;

#ifdef SHOW_SAMPLING_LOG
    static uint64_t last_rx_start_time = 0;
    uint64_t current_rx_start_time = esp_timer_get_time();
    double rx_pri_ms = 0.0;
    if (last_rx_start_time != 0) {
        rx_pri_ms = (double)(current_rx_start_time - last_rx_start_time) / 1000.0;
    }
    last_rx_start_time = current_rx_start_time;
#endif

    if (res == ESP_OK && bytes_read == sizeof(_raw_adc_buffer)) {
#ifdef SHOW_SAMPLING_LOG
        double fs_actual = (double)(Constant::ADC_SAMPLES) * 1000000.0 / (double)elapsed_time;
#endif

        // 1. DC bias & normalization...
        int16_t mean = calculateDcBias();
        processRawBuffer(mean);

        // 2. IIR...
        applyIirFilter();

        // 3. Peak...
        int peak_idx = findSyncPeak();

        if (peak_idx == -1) {
            static uint32_t dropCount = 0;
            dropCount++;
            if (dropCount % 50 == 0) {
                Serial.printf("[SYNC] Cảnh báo: Không tìm thấy đỉnh đồng bộ > %.1fV trong %d mẫu đầu. Đã bỏ qua %u xung.\n", 
                              Constant::SYNC_THRESHOLD_VOLTS, Constant::SYNC_SEARCH_LEN, dropCount);
            }
            return;
        }

        volatile int shift = peak_idx - 1;
        shiftSignal(shift);

        static uint32_t loopCount = 0;
        loopCount++;
        if (loopCount % 50 == 0) {
#ifdef SHOW_SAMPLING_LOG
            Serial.printf("[LOG] Tx PRI: %.2f ms | Tx Fs: %.2f kHz | Rx PRI: %.2f ms | Rx Fs: %.2f kHz (Đọc trong %llu us)\n",
                          txPriMs, txFsKhz, rx_pri_ms, fs_actual / 1000.0, elapsed_time);
#endif
        }

        // 6. Gửi dữ liệu qua UDP
        static uint32_t sendCount = 0;
        sendCount++;
        if (sendCount % Constant::UDP_SEND_DIVIDER == 0) {
            com.sendFrameAsync(frameId++, _send_adc_buffer, Constant::ADC_SAMPLES, Constant::RX_CHANNEL_1_ID);
        } else {
            frameId++;
        }
    } else {
        Serial.printf("Lỗi đọc I2S: %d, số bytes đọc được: %u\n", res, bytes_read);
    }
}
