#include "TransmitterDMAApp.hpp"
#include "Constant.hpp"

TransmitterDMAApp::TransmitterDMAApp(DacDMAService& dacService) : _dacService(dacService) {}

void TransmitterDMAApp::init() {
    // Tự động thêm 4 điểm bias (127) ở đầu và copy SINGLE_PULSE_WAVE vào buffer
    memset(_single_pulse, Constant::DAC_DC_BIAS, 4);
    memcpy(_single_pulse + 4, Constant::SINGLE_PULSE_WAVE, Constant::FILTER_COEFFS_LEN);

    // Tự động thêm 4 điểm bias (127) ở đầu và copy BARKER13_PULSE_WAVE vào buffer
    memset(_barker13_pulse, Constant::DAC_DC_BIAS, 4);
    memcpy(_barker13_pulse + 4, Constant::BARKER13_PULSE_WAVE, Constant::BARKER13_PULSE_LEN);
}

void IRAM_ATTR TransmitterDMAApp::transmit() {
    _dacService.transmitPulse(_barker13_pulse, Constant::BARKER13_PULSE_LEN + 4);
}
