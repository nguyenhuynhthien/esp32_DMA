#ifndef SIMULATOR_DMA_APP_HPP
#define SIMULATOR_DMA_APP_HPP

#include <Arduino.h>
#include "service/ComManager.hpp"
#include "TransmitterDMAApp.hpp"
#include <Constant.h>

#ifdef SIMULATION_MODE

class SimulatorDMAApp {
public:
    SimulatorDMAApp();
    void init();
    uint32_t fireSimulatedTransmission(TransmitterDMAApp& transmitterApp, ComManager::PulseType pulseType, float txGain);
    // Chèn nhiễu sau khi buffer đã nhận dữ liệu từ ADC DMA.
    void injectNoise(uint16_t* buffer, size_t sampleCount);

private:
    static constexpr size_t SIN_LUT_SIZE = 256;
    static constexpr uint32_t FALLBACK_NOISE_SEED = 0x6D2B79F5u;
    static int16_t _sin_lut[SIN_LUT_SIZE];
    static bool _sin_lut_initialized;
    uint32_t _noise_state;

    uint32_t nextNoiseValue();
};

#endif // SIMULATION_MODE

#endif // SIMULATOR_DMA_APP_HPP
