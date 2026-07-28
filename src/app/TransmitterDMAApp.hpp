#ifndef TRANSMITTER_DMA_APP_HPP
#define TRANSMITTER_DMA_APP_HPP

#include "../service/DacDMAService.hpp"
#include "../../include/Constants.h"

#include "../service/ComManager.h"

class TransmitterDMAApp {
public:
    TransmitterDMAApp(DacDMAService& dacService);
    void init();
    void IRAM_ATTR transmit(ComManager::PulseType pulseType);
#ifdef SHOW_SAMPLING_LOG
    void printDacMetrics(double priMs);
#endif

private:
    DacDMAService& _dacService;
#ifdef SHOW_SAMPLING_LOG
    uint32_t _loopCount;
#endif
    uint8_t _single_pulse[Constant::FILTER_COEFFS_LEN + Constant::DAC_PRE_BIAS_SAMPLES];
    uint8_t _barker13_pulse[Constant::BARKER13_PULSE_LEN + Constant::DAC_PRE_BIAS_SAMPLES];
};

#endif // TRANSMITTER_DMA_APP_HPP
