#include "DacDMAService.hpp"

DacDMAService::DacDMAService(DacDMASignal& dacSignal) : _dacSignal(dacSignal) {}

void DacDMAService::init() {
    _dacSignal.init();
}

uint32_t IRAM_ATTR DacDMAService::transmitPulse(const uint8_t* pulse, size_t length) {
    return _dacSignal.firePulse(pulse, length);
}
