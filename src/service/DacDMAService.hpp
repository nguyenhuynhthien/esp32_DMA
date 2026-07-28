#ifndef DAC_DMA_SERVICE_HPP
#define DAC_DMA_SERVICE_HPP

#include "../driver/DacDMASignal.hpp"
#include "../../include/Constants.h"

class DacDMAService {
public:
    DacDMAService(DacDMASignal& dacSignal);
    void init();
    void IRAM_ATTR transmitPulse(const uint8_t* pulse, size_t length);
#ifdef SHOW_SAMPLING_LOG
    uint64_t getLastTransmitCycles() const { return _dacSignal.getLastTransmitCycles(); }
    size_t getLastTransmitLength() const { return _dacSignal.getLastTransmitLength(); }
#endif

private:
    DacDMASignal& _dacSignal;
};

#endif // DAC_DMA_SERVICE_HPP
