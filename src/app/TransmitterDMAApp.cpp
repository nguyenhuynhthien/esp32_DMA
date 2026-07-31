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
