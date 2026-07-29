#ifndef SIMULATOR_DMA_APP_HPP
#define SIMULATOR_DMA_APP_HPP

#include <Arduino.h>
#include "../service/ComManager.h"
#include "../../include/Constants.h"

class SimulatorDMAApp {
public:
    SimulatorDMAApp();
    void init();
    void injectSimulationQ15(int16_t* sendBuffer, size_t size, ComManager::PulseType pulseType, uint16_t frameId, double priMs, bool txEnabled);

private:
    static constexpr size_t NOISE_TABLE_SIZE = 512;
    int16_t _noise_table[NOISE_TABLE_SIZE];
    size_t _noise_idx;
};

#endif // SIMULATOR_DMA_APP_HPP
