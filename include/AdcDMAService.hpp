#ifndef ADC_DMA_SERVICE_HPP
#define ADC_DMA_SERVICE_HPP

#include "AdcDMASignal.hpp"

class AdcDMAService {
public:
    AdcDMAService(AdcDMASignal& adcSignal);
    void init();
    void IRAM_ATTR start();
    void stop();
    esp_err_t readSamples(uint16_t* buffer, size_t size, size_t& bytesRead);

private:
    AdcDMASignal& _adcSignal;
};

#endif // ADC_DMA_SERVICE_HPP
