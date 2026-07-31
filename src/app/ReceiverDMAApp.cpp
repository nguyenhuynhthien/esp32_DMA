#include "ReceiverDMAApp.hpp"
#include <Arduino.h>

ReceiverDMAApp::ReceiverDMAApp(AdcDMAService& adcService, uint8_t receiverId)
    : _adcService(adcService), _receiverId(receiverId) {}

void ReceiverDMAApp::init() {
    // Buffers or additional setup if needed
}

int16_t ReceiverDMAApp::calculateDcBias() {
    static int16_t cached_bias = 2048;
    static int frame_count = 0;
    if (frame_count++ % 64 == 0) {
        int32_t sum = 0;
        for (size_t i = 0; i < Constant::ADC_SAMPLES; ++i) {
            _raw_adc_buffer[i] &= Constant::ADC_RESOLUTION_MAX;
            sum += _raw_adc_buffer[i];
        }
        cached_bias = sum >> Constant::ADC_SAMPLES_SHIFT;
    }
    return cached_bias;
}

// Hàm bão hòa số nguyên 16-bit nhanh hơn macro constrain của Arduino
inline int16_t saturate16(int32_t val) {
    if (val > Constant::Q15_MAX) return Constant::Q15_MAX;
    if (val < Constant::Q15_MIN) return Constant::Q15_MIN;
    return (int16_t)val;
}

void ReceiverDMAApp::processRawBuffer(int16_t mean) {
    const uint16_t* p_raw = _raw_adc_buffer;
    int16_t* p_send = _send_adc_buffer;
    
    // Mở rộng vòng lặp (Loop Unrolling) gấp 4 lần để tối ưu hóa pipeline của CPU
    size_t i = 0;
    for (; i < Constant::ADC_SAMPLES - 3; i += 4) {
        int32_t val0 = (p_raw[i] & Constant::ADC_RESOLUTION_MAX) - mean;
        int32_t val1 = (p_raw[i+1] & Constant::ADC_RESOLUTION_MAX) - mean;
        int32_t val2 = (p_raw[i+2] & Constant::ADC_RESOLUTION_MAX) - mean;
        int32_t val3 = (p_raw[i+3] & Constant::ADC_RESOLUTION_MAX) - mean;
        
        p_send[i]   = saturate16(val0 << Constant::Q15_SCALE_SHIFT);
        p_send[i+1] = saturate16(val1 << Constant::Q15_SCALE_SHIFT);
        p_send[i+2] = saturate16(val2 << Constant::Q15_SCALE_SHIFT);
        p_send[i+3] = saturate16(val3 << Constant::Q15_SCALE_SHIFT);
    }
    
    // Xử lý các phần tử dư thừa còn lại (nếu tổng số mẫu không chia hết cho 4)
    for (; i < Constant::ADC_SAMPLES; ++i) {
        int32_t val = (p_raw[i] & Constant::ADC_RESOLUTION_MAX) - mean;
        p_send[i] = saturate16(val << Constant::Q15_SCALE_SHIFT);
    }
}

void ReceiverDMAApp::applyIirFilter() {
    static int16_t temp_smooth[Constant::ADC_SAMPLES];
    const int16_t* p_send = _send_adc_buffer;
    int16_t* p_smooth = temp_smooth;
    
    p_smooth[0] = p_send[0];
    int16_t prev = p_smooth[0];
    
    for (int i = 1; i < Constant::ADC_SAMPLES; ++i) {
        int32_t val = ((int32_t)p_send[i] * Constant::SIGNAL_SMOOTH_ALPHA_Q15) + 
                      ((int32_t)prev * Constant::SIGNAL_SMOOTH_BETA_Q15);
        prev = (int16_t)(val >> 15);
        p_smooth[i] = prev;
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

void ReceiverDMAApp::receiveAndProcess(ComManager& com, uint16_t& frameId, double priMs, double txPriMs, double txFsKhz, uint64_t adcStartTime) {
#ifdef SHOW_SAMPLING_LOG
    uint64_t start_time = (adcStartTime != 0) ? adcStartTime : esp_timer_get_time();
#endif
    size_t bytes_read = 0;

    esp_err_t res = _adcService.readSamples(_raw_adc_buffer, sizeof(_raw_adc_buffer), bytes_read);
#ifdef SHOW_SAMPLING_LOG
    uint64_t elapsed_time = (adcStartTime != 0 || start_time != 0) ? (esp_timer_get_time() - start_time) : 0;
#endif

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
#ifdef SHOW_SAMPLING_LOG
            _dropCount++;
            if (_dropCount % 50 == 0) {
                Serial.printf("[SYNC] Cảnh báo: Không tìm thấy đỉnh đồng bộ > %.1fV trong %d mẫu đầu. Đã bỏ qua %u xung.\n", 
                              Constant::SYNC_THRESHOLD_VOLTS, Constant::SYNC_SEARCH_LEN, _dropCount);
            }
#endif
            return;
        }

        volatile int shift = peak_idx - 1;
        shiftSignal(shift);

#ifdef SHOW_SAMPLING_LOG
        _loopCount++;
        if (_loopCount % 50 == 0) {
            Serial.printf("[LOG RX%d] Tx PRI: %.2f ms | Tx Fs: %.2f kHz | Rx PRI: %.2f ms | Rx Fs: %.2f kHz (Đọc trong %llu us)\n",
                          _receiverId, txPriMs, txFsKhz, rx_pri_ms, fs_actual / 1000.0, elapsed_time);
        }
#endif

        // 6. Gửi dữ liệu qua UDP
        _sendCount++;
        if (_sendCount % Constant::UDP_SEND_DIVIDER == 0) {
            com.sendFrameAsync(frameId++, _send_adc_buffer, Constant::ADC_SAMPLES, _receiverId);
        } else {
            frameId++;
        }
    } else {
        Serial.printf("Lỗi đọc I2S: %d, số bytes đọc được: %u\n", res, bytes_read);
    }
}

void ReceiverDMAApp::process(const uint16_t* rawSamples, ComManager& com, uint16_t frameId, double priMs, double txPriMs, double txFsKhz, uint64_t elapsed_time, SimulatorDMAApp* simulatorApp) {
    // Sao chép buffer thô vào bộ nhớ cục bộ
    memcpy(_raw_adc_buffer, rawSamples, Constant::ADC_SAMPLES * sizeof(uint16_t));

#ifdef SHOW_SAMPLING_LOG
    // Tính toán tần số lấy mẫu thực tế dựa trên thời gian thực tế thu nhận
    double fs_actual = (double)(Constant::ADC_SAMPLES) * 1000000.0 / (double)elapsed_time;
#endif

    // 1. DC bias & normalization
    int16_t mean = calculateDcBias();
    processRawBuffer(mean);

    // 2. IIR filter
    applyIirFilter();

    // 3. Peak detection
    int peak_idx = findSyncPeak();

    if (peak_idx == -1) {
#ifdef SHOW_SAMPLING_LOG
        _dropCount++;
        if (_dropCount % 50 == 0) {
            Serial.printf("[SYNC RX%d] Cảnh báo: Không tìm thấy đỉnh đồng bộ > %.1fV trong %d mẫu đầu. Đã bỏ qua %u xung.\n", 
                          _receiverId, Constant::SYNC_THRESHOLD_VOLTS, Constant::SYNC_SEARCH_LEN, _dropCount);
        }
#endif
        return;
    }

    volatile int shift = peak_idx - 1;
    shiftSignal(shift);

#ifdef SIMULATION_MODE
    // Tiêm xung giả lập vào mảng dữ liệu đã đồng bộ
    if (simulatorApp != nullptr) {
        simulatorApp->injectSimulationQ15(_send_adc_buffer, Constant::ADC_SAMPLES, com.getPulseType(), frameId, priMs);
    }
#endif

#ifdef SHOW_SAMPLING_LOG
    _loopCount++;
    if (_loopCount % 50 == 0) {
        Serial.printf("[LOG RX%d] Tx PRI: %.2f ms | Tx Fs: %.2f kHz | Rx PRI: %.2f ms | Rx Fs: %.2f kHz (Đọc trong %llu us)\n",
                      _receiverId, txPriMs, txFsKhz, priMs, fs_actual / 1000.0, elapsed_time);
    }
#endif

    // 6. Gửi dữ liệu qua UDP
    _sendCount++;
    if (_sendCount % Constant::UDP_SEND_DIVIDER == 0) {
        com.sendFrameAsync(frameId, _send_adc_buffer, Constant::ADC_SAMPLES, _receiverId);
    }
}
