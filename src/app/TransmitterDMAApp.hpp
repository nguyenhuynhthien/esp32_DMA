#ifndef TRANSMITTER_DMA_APP_HPP
#define TRANSMITTER_DMA_APP_HPP

#include "service/DacDMAService.hpp"
#include <Constant.h>
#include "service/ComManager.hpp"

class TransmitterDMAApp {
public:
    TransmitterDMAApp(DacDMAService& dacService);
    void init();
    uint32_t IRAM_ATTR transmit(ComManager::PulseType type, float txGain = 1.0f);
#ifdef SIMULATION_MODE
    uint32_t IRAM_ATTR transmitSimulationBurst(ComManager::PulseType type, float txGain, size_t delaySamples, float echoGain = 0.15f);
#endif

private:
    DacDMAService& _dacService;
    uint8_t _single_pulse[Constant::FILTER_COEFFS_LEN + Constant::DAC_PULSE_TOTAL_PADDING];
    uint8_t _barker13_pulse[Constant::BARKER13_PULSE_LEN + Constant::DAC_PULSE_TOTAL_PADDING];
#ifdef SIMULATION_MODE
    uint8_t _burst_buffer[800]; // Buffer ghép xung gốc và echo để tránh jitter
#endif
};

#endif // TRANSMITTER_DMA_APP_HPP
