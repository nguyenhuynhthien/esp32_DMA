#include <Constant.h>
#include "ReceiverDMAApp.hpp"
#include <Arduino.h>

// Định nghĩa các con trỏ bộ đệm toàn cục cho 2 máy thu độc lập (Rx1 có _receiverId = 1, Rx2 có _receiverId = 2)
uint16_t* g_raw_adc_buffer[2] = {nullptr, nullptr};
int16_t* g_send_adc_buffer[2] = {nullptr, nullptr};
int16_t* g_demod_I[2] = {nullptr, nullptr};
int16_t* g_demod_Q[2] = {nullptr, nullptr};

ReceiverDMAApp::ReceiverDMAApp(AdcDMAService& adcService, uint8_t receiverId)
    : _adcService(adcService), _receiverId(receiverId) {}

void ReceiverDMAApp::init() {
    int idx = (_receiverId == 2) ? 1 : 0;
    
    // Cấp phát động từ Internal DRAM sử dụng heap_caps_malloc để tránh tràn vùng nhớ tĩnh BSS (.dram0.bss)
    if (g_raw_adc_buffer[idx] == nullptr) {
        g_raw_adc_buffer[idx] = (uint16_t*)heap_caps_malloc(Constant::ADC_SAMPLES * sizeof(uint16_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    }
    if (g_send_adc_buffer[idx] == nullptr) {
        g_send_adc_buffer[idx] = (int16_t*)heap_caps_malloc(Constant::ADC_SAMPLES * sizeof(int16_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    }
    if (g_demod_I[idx] == nullptr) {
        g_demod_I[idx] = (int16_t*)heap_caps_malloc(Constant::ADC_SAMPLES * sizeof(int16_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    }
    if (g_demod_Q[idx] == nullptr) {
        g_demod_Q[idx] = (int16_t*)heap_caps_malloc(Constant::ADC_SAMPLES * sizeof(int16_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    }
}

int16_t ReceiverDMAApp::calculateDcBias() {
    int idx = (_receiverId == 2) ? 1 : 0;
    if (_frame_count++ % 64 == 0) {
        int32_t sum = 0;
        for (size_t i = 0; i < Constant::ADC_SAMPLES; ++i) {
            g_raw_adc_buffer[idx][i] &= Constant::ADC_RESOLUTION_MAX;
            sum += g_raw_adc_buffer[idx][i];
        }
        _cached_bias = sum >> Constant::ADC_SAMPLES_SHIFT;
    }
    return _cached_bias;
}

// Hàm bão hòa số nguyên 16-bit nhanh hơn macro constrain của Arduino
inline int16_t saturate16(int32_t val) {
    if (val > Constant::Q15_MAX) return Constant::Q15_MAX;
    if (val < Constant::Q15_MIN) return Constant::Q15_MIN;
    return (int16_t)val;
}

void ReceiverDMAApp::processRawBuffer(int16_t mean) {
    int idx = (_receiverId == 2) ? 1 : 0;
    const uint16_t* p_raw = g_raw_adc_buffer[idx];
    int16_t* p_send = g_send_adc_buffer[idx];

#ifdef SHOW_SAMPLING_LOG
    // Tìm Min/Max thô để chẩn đoán bão hòa phần cứng
    uint16_t raw_min = 4095;
    uint16_t raw_max = 0;
    for (size_t iVal = 0; iVal < Constant::ADC_SAMPLES; ++iVal) {
        uint16_t val = p_raw[iVal] & Constant::ADC_RESOLUTION_MAX;
        if (val < raw_min) raw_min = val;
        if (val > raw_max) raw_max = val;
    }
    
    if (_log_cnt++ % Constant::DIAG_LOG_DIVIDER == 0) {
        Serial.printf("[DIAG RX%d] Raw ADC Min: %u | Max: %u | Bias: %d\n", _receiverId, raw_min, raw_max, mean);
    }
#endif
    
    // Mở rộng vòng lặp (Loop Unrolling) gấp 4 lần để tối ưu hóa pipeline của CPU
    size_t i = 0;
    for (; i < Constant::ADC_SAMPLES - 3; i += 4) {
        int32_t val0 = (p_raw[i] & Constant::ADC_RESOLUTION_MAX) - mean;
        int32_t val1 = (p_raw[i+1] & Constant::ADC_RESOLUTION_MAX) - mean;
        int32_t val2 = (p_raw[i+2] & Constant::ADC_RESOLUTION_MAX) - mean;
        int32_t val3 = (p_raw[i+3] & Constant::ADC_RESOLUTION_MAX) - mean;
        
        p_send[i]   = saturate16(val0 * Constant::Q15_SCALE_FACTOR);
        p_send[i+1] = saturate16(val1 * Constant::Q15_SCALE_FACTOR);
        p_send[i+2] = saturate16(val2 * Constant::Q15_SCALE_FACTOR);
        p_send[i+3] = saturate16(val3 * Constant::Q15_SCALE_FACTOR);
    }
    
    // Xử lý các phần tử dư thừa còn lại (nếu tổng số mẫu không chia hết cho 4)
    for (; i < Constant::ADC_SAMPLES; ++i) {
        int32_t val = (p_raw[i] & Constant::ADC_RESOLUTION_MAX) - mean;
        p_send[i] = saturate16(val * Constant::Q15_SCALE_FACTOR);
    }
}

void ReceiverDMAApp::applyIirFilter() {
    int idx = (_receiverId == 2) ? 1 : 0;
    static int16_t* temp_smooth[2] = {nullptr, nullptr};
    if (temp_smooth[idx] == nullptr) {
        temp_smooth[idx] = (int16_t*)heap_caps_malloc(Constant::ADC_SAMPLES * sizeof(int16_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    }
    const int16_t* p_send = g_send_adc_buffer[idx];
    int16_t* p_smooth = temp_smooth[idx];
    
    p_smooth[0] = p_send[0];
    int16_t prev = p_smooth[0];
    
    for (int i = 1; i < Constant::ADC_SAMPLES; ++i) {
        int32_t val = ((int32_t)p_send[i] * Constant::SIGNAL_SMOOTH_ALPHA_Q15) + 
                      ((int32_t)prev * Constant::SIGNAL_SMOOTH_BETA_Q15);
        prev = (int16_t)(val >> 15);
        p_smooth[i] = prev;
    }
    memcpy(g_send_adc_buffer[idx], temp_smooth[idx], Constant::ADC_SAMPLES * sizeof(int16_t));
}

int ReceiverDMAApp::findSyncPeak(float txGain) {
    int idx = (_receiverId == 2) ? 1 : 0;
    // 1. Tìm đỉnh lớn nhất trong cửa sổ tìm kiếm (bỏ qua i = 0 và i = 1 để tránh nhiễu chuyển mạch)
    int16_t local_max = 0;
    for (int i = 2; i < Constant::SYNC_SEARCH_LEN; ++i) {
        if (g_send_adc_buffer[idx][i] > local_max) {
            local_max = g_send_adc_buffer[idx][i];
        }
    }
    
    // Nếu đỉnh thu được quá thấp (ví dụ < SYNC_MIN_LOCAL_MAX trong dải Q15), coi như không có xung phát (Tx Off hoặc nhiễu nền)
    if (local_max < Constant::SYNC_MIN_LOCAL_MAX) {
        return -1;
    }
    
    // 2. Tính ngưỡng động bằng SYNC_THRESHOLD_PERCENT% giá trị đỉnh thực tế
    int16_t dynamic_threshold = (int16_t)((int32_t)local_max * Constant::SYNC_THRESHOLD_PERCENT / 100);
    
    // 3. Tìm mẫu đầu tiên vượt ngưỡng động này
    int peak_idx = -1;
    for (int i = 2; i < Constant::SYNC_SEARCH_LEN; ++i) {
        if (g_send_adc_buffer[idx][i] > dynamic_threshold) {
            peak_idx = i;
            break; // Tìm thấy sườn trước của xung thì dừng ngay
        }
    }
    return peak_idx;
}

void ReceiverDMAApp::shiftSignal(int shift) {
    int idx = (_receiverId == 2) ? 1 : 0;
    int16_t* p_send = g_send_adc_buffer[idx];
    if (shift > 0) {
        // Dịch trái
        for (size_t i = 0; i < Constant::ADC_SAMPLES - shift; ++i) {
            p_send[i] = p_send[i + shift];
        }
        memset(&p_send[Constant::ADC_SAMPLES - shift], 0, shift * sizeof(int16_t));
    } else {
        // Dịch phải (shift <= 0)
        int rshift = -shift;
        for (int i = (int)Constant::ADC_SAMPLES - 1; i >= rshift; --i) {
            p_send[i] = p_send[i - rshift];
        }
        memset(p_send, 0, rshift * sizeof(int16_t));
    }
}

void ReceiverDMAApp::receiveAndProcess(ComManager& com, uint16_t frameId, double priMs, double txPriMs, double txFsKhz, uint64_t adcStartTime) {
    int idx = (_receiverId == 2) ? 1 : 0;
    
    // Đảm bảo buffer đã được cấp phát trước khi truy xuất
    if (g_raw_adc_buffer[idx] == nullptr || g_send_adc_buffer[idx] == nullptr) {
        init();
    }
    
#ifdef SHOW_SAMPLING_LOG
    uint64_t start_time = (adcStartTime != 0) ? adcStartTime : esp_timer_get_time();
#endif
    size_t bytes_read = 0;

    esp_err_t res = _adcService.readSamples(g_raw_adc_buffer[idx], Constant::ADC_SAMPLES * sizeof(uint16_t), bytes_read);
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

    if (res == ESP_OK && bytes_read == Constant::ADC_SAMPLES * sizeof(uint16_t)) {
#ifdef SHOW_SAMPLING_LOG
        double fs_actual = (double)(Constant::ADC_SAMPLES) * 1000000.0 / (double)elapsed_time;
#endif

        // 1. DC bias & normalization...
        int16_t mean = calculateDcBias();
        processRawBuffer(mean);

        // 2. IIR...
        applyIirFilter();

        // 3. Peak...
        int peak_idx = findSyncPeak(com.getTxGain());

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

        // 1. Gửi dữ liệu thô (Raw) ngay lập tức nếu đang ở chế độ STREAM_RAW và đúng kênh đang được lựa chọn gửi
        if (com.getStreamMode() == ComManager::STREAM_RAW) {
            if (_receiverId == com.getSelectedRxChannel()) {
                _sendCount++;
                if (_sendCount % Constant::UDP_SEND_DIVIDER == 0) {
                    com.sendFrameAsync(frameId, g_send_adc_buffer[idx], Constant::ADC_SAMPLES, _receiverId);
                }
            }
        }

        // 2. DẬP XUNG (Luôn diễn ra) & GIẢI ĐIỀU CHẾ
        // Dập xung phát rò rỉ Tx về mức bias (0 trong dải Q15) theo chiều dài xung trước khi giải điều chế
        size_t blankSamples = (com.getPulseType() == ComManager::PULSE_SINGLE) 
                            ? (Constant::FILTER_COEFFS_LEN + Constant::DAC_PULSE_TOTAL_PADDING)
                            : (Constant::BARKER13_PULSE_LEN + Constant::DAC_PULSE_TOTAL_PADDING);
        for (size_t n = 0; n < blankSamples; ++n) {
            if (n < Constant::ADC_SAMPLES) {
                g_send_adc_buffer[idx][n] = 0;
            }
        }

        performIQDemodulation(g_send_adc_buffer[idx]);
        for (size_t n = 0; n < Constant::ADC_SAMPLES; ++n) {
            int32_t abs_I = abs((int32_t)g_demod_I[idx][n]);
            int32_t abs_Q = abs((int32_t)g_demod_Q[idx][n]);
            int32_t max_val = (abs_I > abs_Q) ? abs_I : abs_Q;
            int32_t min_val = (abs_I > abs_Q) ? abs_Q : abs_I;
            int32_t amp = max_val + ((3 * min_val) >> 3);
            g_send_adc_buffer[idx][n] = (int16_t)((amp > Constant::Q15_MAX) ? Constant::Q15_MAX : amp);
        }

        // 3. Gửi dữ liệu giải điều chế nếu đang ở chế độ STREAM_DEMOD và đúng kênh đang được lựa chọn gửi
        if (com.getStreamMode() == ComManager::STREAM_DEMOD) {
            if (_receiverId == com.getSelectedRxChannel()) {
                _sendCount++;
                if (_sendCount % Constant::UDP_SEND_DIVIDER == 0) {
                    com.sendFrameAsync(frameId, g_send_adc_buffer[idx], Constant::ADC_SAMPLES, _receiverId);
                }
            }
        }
    } else {
        Serial.printf("Lỗi đọc I2S: %d, số bytes đọc được: %u\n", res, bytes_read);
    }
}

void ReceiverDMAApp::process(const uint16_t* rawSamples, ComManager& com, uint16_t frameId, double priMs, double txPriMs, double txFsKhz, uint64_t elapsed_time, 
                             bool txEnabled) {
    int idx = (_receiverId == 2) ? 1 : 0;
    
    // Đảm bảo buffer đã được cấp phát trước khi truy xuất
    if (g_raw_adc_buffer[idx] == nullptr || g_send_adc_buffer[idx] == nullptr) {
        init();
    }
    
    // Sao chép buffer thô vào bộ nhớ cục bộ
    memcpy(g_raw_adc_buffer[idx], rawSamples, Constant::ADC_SAMPLES * sizeof(uint16_t));

#ifdef SHOW_SAMPLING_LOG
    // Tính toán tần số lấy mẫu thực tế dựa trên thời gian thực tế thu nhận
    double fs_actual = (double)(Constant::ADC_SAMPLES) * 1000000.0 / (double)elapsed_time;
#endif

    // 1. DC bias & normalization
    int16_t mean = calculateDcBias();
    processRawBuffer(mean);

    // 2. IIR filter
    applyIirFilter();

    // 3. Peak detection (Only when Tx is enabled)
    bool has_valid_signal = true;
    if (txEnabled) {
        int peak_idx = findSyncPeak(com.getTxGain());
        if (peak_idx == -1) {
#ifdef SHOW_SAMPLING_LOG
            _dropCount++;
            if (_dropCount % 50 == 0) {
                Serial.printf("[SYNC RX%d] Cảnh báo: Không tìm thấy đỉnh đồng bộ > %.1fV trong %d mẫu đầu. Đã bỏ qua %u xung.\n", 
                              _receiverId, Constant::SYNC_THRESHOLD_VOLTS, Constant::SYNC_SEARCH_LEN, _dropCount);
            }
#endif
            has_valid_signal = false;
        } else {
            volatile int shift = peak_idx - 1;
            shiftSignal(shift);
        }
    }

    if (!txEnabled || has_valid_signal) {
#ifdef SHOW_SAMPLING_LOG
        _loopCount++;
        if (_loopCount % 50 == 0) {
            Serial.printf("[LOG RX%d] Tx PRI: %.2f ms | Tx Fs: %.2f kHz | Rx PRI: %.2f ms | Rx Fs: %.2f kHz (Đọc trong %llu us)\n",
                          _receiverId, txPriMs, txFsKhz, priMs, fs_actual / 1000.0, elapsed_time);
        }
#endif

        // 1. Gửi dữ liệu thô (Raw) ngay lập tức nếu đang ở chế độ STREAM_RAW và đúng kênh đang lựa chọn gửi
        if (com.getStreamMode() == ComManager::STREAM_RAW) {
            if (_receiverId == com.getSelectedRxChannel()) {
                _sendCount++;
                if (_sendCount % Constant::UDP_SEND_DIVIDER == 0) {
                    com.sendFrameAsync(frameId, g_send_adc_buffer[idx], Constant::ADC_SAMPLES, _receiverId);
                }
            }
        }

        // 2. DẬP XUNG (Luôn diễn ra) & GIẢI ĐIỀU CHẾ
        // Dập xung phát rò rỉ Tx về mức bias (0 trong dải Q15) theo chiều dài xung trước khi giải điều chế
        size_t blankSamples = (com.getPulseType() == ComManager::PULSE_SINGLE) 
                            ? (Constant::FILTER_COEFFS_LEN + Constant::DAC_PULSE_TOTAL_PADDING)
                            : (Constant::BARKER13_PULSE_LEN + Constant::DAC_PULSE_TOTAL_PADDING);
        for (size_t n = 0; n < blankSamples; ++n) {
            if (n < Constant::ADC_SAMPLES) {
                g_send_adc_buffer[idx][n] = 0;
            }
        }

        performIQDemodulation(g_send_adc_buffer[idx]);
        for (size_t n = 0; n < Constant::ADC_SAMPLES; ++n) {
            int32_t abs_I = abs((int32_t)g_demod_I[idx][n]);
            int32_t abs_Q = abs((int32_t)g_demod_Q[idx][n]);
            int32_t max_val = (abs_I > abs_Q) ? abs_I : abs_Q;
            int32_t min_val = (abs_I > abs_Q) ? abs_Q : abs_I;
            int32_t amp = max_val + ((3 * min_val) >> 3);
            g_send_adc_buffer[idx][n] = (int16_t)((amp > Constant::Q15_MAX) ? Constant::Q15_MAX : amp);
        }

        // 3. Gửi dữ liệu giải điều chế nếu đang ở chế độ STREAM_DEMOD và đúng kênh đang lựa chọn gửi
        if (com.getStreamMode() == ComManager::STREAM_DEMOD) {
            if (_receiverId == com.getSelectedRxChannel()) {
                _sendCount++;
                if (_sendCount % Constant::UDP_SEND_DIVIDER == 0) {
                    com.sendFrameAsync(frameId, g_send_adc_buffer[idx], Constant::ADC_SAMPLES, _receiverId);
                }
            }
        }
    }
}

void ReceiverDMAApp::performIQDemodulation(const int16_t* rawSamples) {
    int idx = (_receiverId == 2) ? 1 : 0;
    
    // Đảm bảo buffer đã được cấp phát trước khi truy xuất
    if (g_demod_I[idx] == nullptr || g_demod_Q[idx] == nullptr) {
        init();
    }
    
    // Giải điều chế I/Q sóng mang 40kHz với fs = 160kHz (Loop Unrolled 4x để tránh rẽ nhánh CPU)
    for (size_t n = 0; n < Constant::ADC_SAMPLES; n += 4) {
        g_demod_I[idx][n]     = rawSamples[n];
        g_demod_Q[idx][n]     = 0;
        
        g_demod_I[idx][n + 1] = 0;
        g_demod_Q[idx][n + 1] = -rawSamples[n + 1];
        
        g_demod_I[idx][n + 2] = -rawSamples[n + 2];
        g_demod_Q[idx][n + 2] = 0;
        
        g_demod_I[idx][n + 3] = 0;
        g_demod_Q[idx][n + 3] = rawSamples[n + 3];
    }

    // Bộ lọc trung bình trượt 4 mẫu tối ưu hóa In-place (tránh malloc và memcpy bổ sung)
    int32_t sumI = 0, sumQ = 0;
    int16_t* pI = g_demod_I[idx];
    int16_t* pQ = g_demod_Q[idx];
    
    sumI = (int32_t)pI[0] + pI[1] + pI[2];
    sumQ = (int32_t)pQ[0] + pQ[1] + pQ[2];

    for (size_t n = 3; n < Constant::ADC_SAMPLES; ++n) {
        sumI += pI[n];
        sumQ += pQ[n];
        
        pI[n - 3] = (int16_t)(sumI >> 2); // Chia cho DEMOD_AVG_LEN = 4
        pQ[n - 3] = (int16_t)(sumQ >> 2);
        
        sumI -= pI[n - 3];
        sumQ -= pQ[n - 3];
    }
}

// Fast integer square root (Giữ lại để đảm bảo tương thích giao diện lớp nếu có file khác extern gọi, hoặc có thể lược bỏ)
uint32_t ReceiverDMAApp::isqrt32(uint32_t n) {
    uint32_t root = 0;
    uint32_t bit = 1 << 30;
    while (bit > n) {
        bit >>= 2;
    }
    while (bit != 0) {
        if (n >= root + bit) {
            n -= root + bit;
            root = (root >> 1) + bit;
        } else {
            root >>= 1;
        }
        bit >>= 2;
    }
    return root;
}
