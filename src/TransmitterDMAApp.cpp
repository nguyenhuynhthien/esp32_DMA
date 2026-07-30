#include "TransmitterDMAApp.hpp"
#include "Constant.hpp"

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

uint32_t IRAM_ATTR TransmitterDMAApp::transmit(ComManager::PulseType type) {
    size_t len = 0;
    if (type == ComManager::PULSE_SINGLE) {
        len = Constant::FILTER_COEFFS_LEN + Constant::DAC_PULSE_TOTAL_PADDING;
        return _dacService.transmitPulse(_single_pulse, len);
    } else {
        len = Constant::BARKER13_PULSE_LEN + Constant::DAC_PULSE_TOTAL_PADDING;
        return _dacService.transmitPulse(_barker13_pulse, len);
    }
}
