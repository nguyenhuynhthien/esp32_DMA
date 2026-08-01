#include "TransmitterDMAApp.hpp"
#include <Constant.h>

TransmitterDMAApp::TransmitterDMAApp(DacDMAService& dacService) : _dacService(dacService) {}

void TransmitterDMAApp::init() {
    // Tự động thêm điểm bias (127) ở đầu và copy SINGLE_PULSE_WAVE nguyên bản 100% vào buffer
    memset(_single_pulse, Constant::DAC_DC_BIAS, Constant::DAC_PULSE_PADDING);
    memcpy(_single_pulse + Constant::DAC_PULSE_PADDING, Constant::SINGLE_PULSE_WAVE, Constant::FILTER_COEFFS_LEN);
    memset(_single_pulse + Constant::DAC_PULSE_PADDING + Constant::FILTER_COEFFS_LEN, Constant::DAC_DC_BIAS, Constant::DAC_PULSE_PADDING);

    // Barker13 nguyên bản 100%
    memset(_barker13_pulse, Constant::DAC_DC_BIAS, Constant::DAC_PULSE_PADDING);
    memcpy(_barker13_pulse + Constant::DAC_PULSE_PADDING, Constant::BARKER13_PULSE_WAVE, Constant::BARKER13_PULSE_LEN);
    memset(_barker13_pulse + Constant::DAC_PULSE_PADDING + Constant::BARKER13_PULSE_LEN, Constant::DAC_DC_BIAS, Constant::DAC_PULSE_PADDING);
}

uint32_t IRAM_ATTR TransmitterDMAApp::transmit(ComManager::PulseType type, float txGain) {
    size_t len = 0;
    const uint8_t* source_pulse = nullptr;
    
    if (type == ComManager::PULSE_SINGLE) {
        len = Constant::FILTER_COEFFS_LEN + Constant::DAC_PULSE_TOTAL_PADDING;
        source_pulse = _single_pulse;
    } else {
        len = Constant::BARKER13_PULSE_LEN + Constant::DAC_PULSE_TOTAL_PADDING;
        source_pulse = _barker13_pulse;
    }

    // Nếu gain gần bằng 1.0 (không suy hao), phát trực tiếp để tối ưu tốc độ
    if (txGain >= Constant::TX_ATTEN_UNITY_THRESHOLD) {
        return _dacService.transmitPulse(source_pulse, len);
    }

    // Áp dụng suy hao động lên bản copy tạm thời trên Stack
    // Giới hạn len tối đa để buffer stack an toàn (Barker 13 + padding = 104 + 8 = 112)
    uint8_t attenuated_pulse[Constant::ATTENUATED_BUFFER_SIZE];
    for (size_t i = 0; i < len; ++i) {
        int32_t deviation = (int32_t)source_pulse[i] - Constant::DAC_DC_BIAS;
        int32_t attenuated_val = Constant::DAC_DC_BIAS + (int32_t)(deviation * txGain);
        attenuated_pulse[i] = (uint8_t)constrain(attenuated_val, Constant::DAC_MIN_VAL, Constant::DAC_MAX_VAL);
    }

    return _dacService.transmitPulse(attenuated_pulse, len);
}

uint32_t IRAM_ATTR TransmitterDMAApp::transmitSimulationBurst(ComManager::PulseType type, float txGain, size_t delaySamples, float echoGain) {
    size_t pulse_len = 0;
    const uint8_t* source_pulse = nullptr;
    
    if (type == ComManager::PULSE_SINGLE) {
        pulse_len = Constant::FILTER_COEFFS_LEN + Constant::DAC_PULSE_TOTAL_PADDING;
        source_pulse = _single_pulse;
    } else {
        pulse_len = Constant::BARKER13_PULSE_LEN + Constant::DAC_PULSE_TOTAL_PADDING;
        source_pulse = _barker13_pulse;
    }

    // Đảm bảo không tràn buffer _burst_buffer (kích thước tối đa là 800)
    if (pulse_len * 2 + delaySamples > 800) {
        return 0;
    }

    size_t write_idx = 0;

    // 1. Ghi xung đồng bộ chính (áp dụng txGain)
    for (size_t i = 0; i < pulse_len; ++i) {
        int32_t deviation = (int32_t)source_pulse[i] - Constant::DAC_DC_BIAS;
        int32_t val = Constant::DAC_DC_BIAS + (int32_t)(deviation * txGain);
        _burst_buffer[write_idx++] = (uint8_t)constrain(val, Constant::DAC_MIN_VAL, Constant::DAC_MAX_VAL);
    }

    // 2. Điền khoảng lặng (mức bias 127) tương ứng với delaySamples
    for (size_t i = 0; i < delaySamples; ++i) {
        _burst_buffer[write_idx++] = Constant::DAC_DC_BIAS;
    }

    // 3. Ghi xung echo giả lập (áp dụng txGain * echoGain)
    float total_echo_gain = txGain * echoGain;
    for (size_t i = 0; i < pulse_len; ++i) {
        int32_t deviation = (int32_t)source_pulse[i] - Constant::DAC_DC_BIAS;
        int32_t val = Constant::DAC_DC_BIAS + (int32_t)(deviation * total_echo_gain);
        _burst_buffer[write_idx++] = (uint8_t)constrain(val, Constant::DAC_MIN_VAL, Constant::DAC_MAX_VAL);
    }

    // 4. Phát toàn bộ burst ra DAC một lần duy nhất
    return _dacService.transmitPulse(_burst_buffer, write_idx);
}
