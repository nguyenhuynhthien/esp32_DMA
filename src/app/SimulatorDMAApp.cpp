#include <Constant.h>
#ifdef SIMULATION_MODE

#include "SimulatorDMAApp.hpp"

// Khởi tạo các biến tĩnh
int16_t SimulatorDMAApp::_sin_lut[SimulatorDMAApp::SIN_LUT_SIZE];
bool SimulatorDMAApp::_sin_lut_initialized = false;

SimulatorDMAApp::SimulatorDMAApp() : _noise_idx(0) {}

void SimulatorDMAApp::init() {
    // Sinh sẵn bảng nhiễu Gauss bằng thuật toán Box-Muller
    float sigma = 1000.0f; // Độ lệch chuẩn của nhiễu trong dải Q15 (tăng cường độ nhiễu)
    for (size_t i = 0; i < NOISE_TABLE_SIZE; ++i) {
        float u1 = static_cast<float>(random(1, 10000)) / 10000.0f;
        float u2 = static_cast<float>(random(1, 10000)) / 10000.0f;
        float noise = sqrtf(-2.0f * logf(u1)) * cosf(2.0f * PI * u2) * sigma;
        _noise_table[i] = static_cast<int16_t>(constrain(static_cast<int32_t>(noise), Constant::Q15_MIN, Constant::Q15_MAX));
    }

    // Sinh sẵn bảng Sine Q15 LUT (chỉ thực hiện một lần duy nhất)
    if (!_sin_lut_initialized) {
        for (size_t i = 0; i < SIN_LUT_SIZE; ++i) {
            float angle = (2.0f * PI * i) / SIN_LUT_SIZE;
            _sin_lut[i] = static_cast<int16_t>(sinf(angle) * Constant::Q15_MAX);
        }
        _sin_lut_initialized = true;
    }
}

void SimulatorDMAApp::fireSimulatedTransmission(TransmitterDMAApp& transmitterApp, ComManager::PulseType pulseType, float txGain) {
    // Phát chuỗi xung ghép (xung gốc + khoảng lặng 500 mẫu + xung echo) một lần duy nhất ra DAC
    transmitterApp.transmitSimulationBurst(pulseType, txGain, Constant::SIMULATOR_DELAY_SAMPLES, 0.15f);
}

#endif // SIMULATION_MODE
