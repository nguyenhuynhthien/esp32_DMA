#include "TransmitterDMAApp.hpp"
#include "Constant.hpp"

// Sine pulse (12 samples)
static const uint8_t sine_pulse[12] = {127, 127, 127, 127, 127, 190,
                                        127, 64,  127, 190, 127, 64};

TransmitterDMAApp::TransmitterDMAApp(DacDMAService& dacService) : _dacService(dacService) {}

void TransmitterDMAApp::init() {
    // DacDMAService is initialized externally or coordinated
}

void IRAM_ATTR TransmitterDMAApp::transmit() {
    _dacService.transmitPulse(sine_pulse, Constant::DAC_PULSE_LEN);
}
