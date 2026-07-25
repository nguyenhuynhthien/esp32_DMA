#ifndef ADC_DMA_SIGNAL_HPP
#define ADC_DMA_SIGNAL_HPP

#include <Arduino.h>
#include <driver/adc.h>
#include <driver/i2s.h>

class AdcDMASignal {
public:
    AdcDMASignal();
    void init();
    void IRAM_ATTR start();
    void stop();
    esp_err_t read(void* dest, size_t size, size_t* bytes_read, TickType_t ticks_to_wait);
};

#endif // ADC_DMA_SIGNAL_HPP
