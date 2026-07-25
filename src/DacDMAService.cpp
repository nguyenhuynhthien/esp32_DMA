#include "DacDMAService.hpp"

DacDMAService::DacDMAService(DacDMASignal& dacSignal) : _dacSignal(dacSignal) {}

void DacDMAService::init() {
    _dacSignal.init();
}

void IRAM_ATTR DacDMAService::transmitPulse(const uint8_t* pulse, size_t length) {
    _dacSignal.firePulse(pulse, length);
}
