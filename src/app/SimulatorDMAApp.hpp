#ifndef SIMULATOR_DMA_APP_HPP
#define SIMULATOR_DMA_APP_HPP

#include <Arduino.h>
#include "service/ComManager.hpp"
#include <Constant.h>

class SimulatorDMAApp {
public:
    SimulatorDMAApp();
    void init();
    void injectSimulationQ15(int16_t* sendBuffer, size_t size, ComManager::PulseType pulseType, uint16_t frameId, double priMs);

private:
    static constexpr size_t NOISE_TABLE_SIZE = 512;
    static constexpr size_t SIN_LUT_SIZE = 256;
    int16_t _noise_table[NOISE_TABLE_SIZE];
    static int16_t _sin_lut[SIN_LUT_SIZE];
    static bool _sin_lut_initialized;
    size_t _noise_idx;
};

#endif // SIMULATOR_DMA_APP_HPP
