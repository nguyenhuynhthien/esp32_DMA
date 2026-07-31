#ifndef DAC_DMA_SERVICE_HPP
#define DAC_DMA_SERVICE_HPP
#include "driver/DacDMASignal.hpp"

class DacDMAService {
public:
    DacDMAService(DacDMASignal& dacSignal);
    void init();
    uint32_t IRAM_ATTR transmitPulse(const uint8_t* pulse, size_t length);

private:
    DacDMASignal& _dacSignal;
};

#endif // DAC_DMA_SERVICE_HPP
