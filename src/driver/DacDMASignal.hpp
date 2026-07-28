#ifndef DAC_DMA_SIGNAL_HPP
#define DAC_DMA_SIGNAL_HPP

#include <Arduino.h>
#include <driver/dac.h>
#include "../../include/Constants.h"

class DacDMASignal {
public:
    DacDMASignal();
    void init();
    void setVoltage(uint8_t voltage);
    void IRAM_ATTR firePulse(const uint8_t* pulse, size_t length);
    
#ifdef SHOW_SAMPLING_LOG
    uint64_t getLastTransmitCycles() const { return _lastTransmitCycles; }
    size_t getLastTransmitLength() const { return _lastTransmitLength; }
#endif

private:
#ifdef SHOW_SAMPLING_LOG
    uint64_t _lastTransmitCycles;
    size_t _lastTransmitLength;
#endif

    static inline uint32_t get_ccount() {
        uint32_t ccount;
        asm volatile("rsr %0, ccount" : "=r"(ccount));
        return ccount;
    }
};

#endif // DAC_DMA_SIGNAL_HPP
