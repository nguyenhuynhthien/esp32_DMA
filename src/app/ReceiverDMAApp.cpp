#include "ReceiverDMAApp.hpp"
#include "../service/Q15SimdHelper.h"
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
        process(_raw_adc_buffer, com, frameId, priMs, elapsed_time, nullptr);
    } else {
        Serial.printf("Lỗi đọc I2S: %d, số bytes đọc được: %u\n", res, bytes_read);
    }
}

void ReceiverDMAApp::process(const uint16_t* rawSamples, ComManager& com, uint16_t frameId, double priMs, uint64_t elapsed_time, QueueHandle_t udpQueue, SimulatorDMAApp* simulatorApp, bool txEnabled) {
    // Sao chép buffer thô vào bộ nhớ cục bộ
    memcpy(_raw_adc_buffer, rawSamples, Constant::ADC_SAMPLES * sizeof(uint16_t));

#ifdef SHOW_SAMPLING_LOG
    uint64_t t_process_start = esp_timer_get_time();
    // Tính toán tần số lấy mẫu thực tế dựa trên thời gian thực tế thu nhận
    double fs_actual = (double)(Constant::ADC_SAMPLES) * 1000000.0 / (double)elapsed_time;
    uint64_t t_dsp_start = esp_timer_get_time();
#ifdef TRACE_TASK_TIMING
    Serial.printf("[TRACE][CORE %d][RX%u] frame=%u process begin\n",
                  xPortGetCoreID(), _receiverId, frameId);
#endif
#endif

    // Tính giá trị trung bình (DC bias) của buffer thô
    int32_t sum = 0;
    for (size_t i = 0; i < Constant::ADC_SAMPLES; i += 2) {
        _raw_adc_buffer[i] &= Constant::ADC_RESOLUTION_MAX;
        _raw_adc_buffer[i + 1] &= Constant::ADC_RESOLUTION_MAX;
        sum += _raw_adc_buffer[i] + _raw_adc_buffer[i + 1];
    }
    // Tối ưu hóa phép chia cho 2048 bằng phép dịch bit phải 11
    int16_t mean = sum >> Constant::ADC_SAMPLES_SHIFT;

    // Chỉ cần chuẩn hóa cửa sổ đầu tiên để tìm đỉnh cục bộ phục vụ căn chỉnh
    for (int i = 0; i < Constant::JITTER_WINDOW_LEN; i += 2) {
        Q15x2 q = q15x2_add_sat_adc(&_raw_adc_buffer[i], mean);
        _send_adc_buffer[i] = q.lane[0];
        _send_adc_buffer[i + 1] = q.lane[1];
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
        size_t count = Constant::ADC_SAMPLES - shift;
        size_t i = 0;
        for (; i + 1 < count; i += 2) {
            Q15x2 q = q15x2_add_sat_adc(&_raw_adc_buffer[i + shift], mean);
            _send_adc_buffer[i] = q.lane[0];
            _send_adc_buffer[i + 1] = q.lane[1];
        }
        if (i < count) _send_adc_buffer[i] = q15x2_add_sat_adc(&_raw_adc_buffer[i + shift], mean).lane[0];
        // Xóa nhanh vùng cuối mảng bằng memset
        memset(&_send_adc_buffer[Constant::ADC_SAMPLES - shift], 0, shift * sizeof(int16_t));
    } else {
        // Dịch phải (shift <= 0)
        int rshift = -shift;
        int i = (int)Constant::ADC_SAMPLES - 1;
        for (; i - 1 >= rshift; i -= 2) {
            Q15x2 q = q15x2_add_sat_adc(&_raw_adc_buffer[i - 1 - rshift], mean);
            _send_adc_buffer[i - 1] = q.lane[0];
            _send_adc_buffer[i] = q.lane[1];
        }
        if (i >= rshift) _send_adc_buffer[i] = q15x2_add_sat_adc(&_raw_adc_buffer[i - rshift], mean).lane[0];
        // Xóa nhanh vùng đầu mảng bằng memset
        memset(_send_adc_buffer, 0, rshift * sizeof(int16_t));
    }

#ifdef SIMULATION_MODE
    // Tiêm xung giả lập vào mảng dữ liệu đã đồng bộ
    if (simulatorApp != nullptr) {
        simulatorApp->injectSimulationQ15(_send_adc_buffer, Constant::ADC_SAMPLES, com.getPulseType(), frameId, priMs, txEnabled);
    }
#endif

#ifdef SHOW_SAMPLING_LOG
    uint64_t t_dsp_end = esp_timer_get_time();
    uint64_t t_udp_start = esp_timer_get_time();
#endif

    if (udpQueue != nullptr) {
        UdpFrameMessage msg;
        msg.frameId = frameId;
        msg.receiverId = _receiverId;
        memcpy(msg.samples, _send_adc_buffer, Constant::ADC_SAMPLES * sizeof(int16_t));
        xQueueSend(udpQueue, &msg, 0);
    } else {
        // Gửi dữ liệu qua giao thức UDP của ComManager đến SonarViewer dưới định dạng kênh tương ứng
        com.sendFrame(frameId, _send_adc_buffer, Constant::ADC_SAMPLES, _receiverId);
    }

#ifdef SHOW_SAMPLING_LOG
    uint64_t t_udp_end = esp_timer_get_time();

    _loopCount++;
    if (_loopCount % Constant::LOG_INTERVAL_FRAMES == 0) {
        Serial.printf("[LOG RX%d] PRI: %.2f ms | Số mẫu: %u | Tần số lấy mẫu thực tế: %.2f kHz (Đọc trong %llu us)\n",
                      _receiverId, priMs, Constant::ADC_SAMPLES, fs_actual / 1000.0, elapsed_time);
        Serial.printf("[DEBUG RX%d] peak_idx: %d, shift: %d, max_val: %d, mean: %d\n", 
                      _receiverId, peak_idx, shift, max_val, mean);
        Serial.printf("[PROFILE RX%d] DSP Processing: %llu us | Queue Push/UDP Transmit: %llu us\n",
                      _receiverId, t_dsp_end - t_dsp_start, t_udp_end - t_udp_start);
    }
#ifdef TRACE_TASK_TIMING
    Serial.printf("[TRACE][CORE %d][RX%u] frame=%u total=%llu us (dsp=%llu us, udp=%llu us)\n",
                  xPortGetCoreID(), _receiverId, frameId,
                  t_udp_end - t_process_start,
                  t_dsp_end - t_dsp_start,
                  t_udp_end - t_udp_start);
#endif
#endif
}
