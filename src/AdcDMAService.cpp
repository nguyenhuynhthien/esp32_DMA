#include "AdcDMAService.hpp"

AdcDMAService::AdcDMAService(AdcDMASignal& adcSignal) : _adcSignal(adcSignal) {}

void AdcDMAService::init() {
    _adcSignal.init();
}

void IRAM_ATTR AdcDMAService::start() {
    _adcSignal.start();
}

void AdcDMAService::stop() {
    _adcSignal.stop();
}

esp_err_t AdcDMAService::readSamples(uint16_t* buffer, size_t size, size_t& bytesRead) {
    return _adcSignal.read(buffer, size, &bytesRead, portMAX_DELAY);
}
