#ifndef DAC_DMA_SIGNAL_HPP
#define DAC_DMA_SIGNAL_HPP

#include <Arduino.h>
#include <driver/dac.h>

class DacDMASignal {
public:
    DacDMASignal();
    void init();
    void setVoltage(uint8_t voltage);
    void IRAM_ATTR firePulse(const uint8_t* pulse, size_t length);

private:
    static inline uint32_t get_ccount() {
        uint32_t ccount;
        asm volatile("rsr %0, ccount" : "=r"(ccount));
        return ccount;
    }
};

#endif // DAC_DMA_SIGNAL_HPP
