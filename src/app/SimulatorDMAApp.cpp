#include <Constant.h>
#ifdef SIMULATION_MODE

#include "SimulatorDMAApp.hpp"

// Khởi tạo các biến tĩnh
int16_t SimulatorDMAApp::_sin_lut[SimulatorDMAApp::SIN_LUT_SIZE];
bool SimulatorDMAApp::_sin_lut_initialized = false;

SimulatorDMAApp::SimulatorDMAApp() : _noise_state(FALLBACK_NOISE_SEED) {}

void SimulatorDMAApp::init() {
    // Khởi tạo seed một lần; PRNG sẽ tạo nhiễu mới mà không cần lưu bảng mẫu.
    uint32_t seed = static_cast<uint32_t>(random(1, 0x7FFFFFFF));
    _noise_state = (seed != 0) ? seed : FALLBACK_NOISE_SEED;

    // Sinh sẵn bảng Sine Q15 LUT (chỉ thực hiện một lần duy nhất)
    if (!_sin_lut_initialized) {
        for (size_t i = 0; i < SIN_LUT_SIZE; ++i) {
            float angle = (2.0f * PI * i) / SIN_LUT_SIZE;
            _sin_lut[i] = static_cast<int16_t>(sinf(angle) * Constant::Q15_MAX);
        }
        _sin_lut_initialized = true;
    }
}

uint32_t SimulatorDMAApp::nextNoiseValue() {
    // Xorshift32 chỉ dùng vài phép toán nguyên nên rất nhẹ trên ESP32.
    uint32_t state = _noise_state;
    state ^= state << 13;
    state ^= state >> 17;
    state ^= state << 5;
    _noise_state = state;
    return state;
}

void SimulatorDMAApp::injectNoise(uint16_t* buffer, size_t sampleCount) {
    if (buffer == nullptr || Constant::ADC_NOISE_STDDEV <= 0) return;

    for (size_t sampleIndex = 0; sampleIndex < sampleCount; ++sampleIndex) {
        uint16_t rawSample = buffer[sampleIndex];
        uint32_t randomValue = nextNoiseValue();
        int32_t centeredNoise = static_cast<int32_t>((randomValue >> 24) & 0xFF) +
                                static_cast<int32_t>((randomValue >> 16) & 0xFF) +
                                static_cast<int32_t>((randomValue >> 8) & 0xFF) +
                                static_cast<int32_t>(randomValue & 0xFF) - 510;
        // Tổng bốn byte tạo phân bố gần Gaussian; hệ số giữ sigma xấp xỉ ADC_NOISE_STDDEV.
        int32_t noise = (centeredNoise * Constant::ADC_NOISE_STDDEV * 7) >> 10;
        int32_t noisyValue = (rawSample & Constant::ADC_RESOLUTION_MAX) + noise;
        if (noisyValue < 0) noisyValue = 0;
        if (noisyValue > Constant::ADC_RESOLUTION_MAX) noisyValue = Constant::ADC_RESOLUTION_MAX;

        // Chỉ thay đổi 12 bit ADC; giữ 4 bit mã kênh để bộ demux vẫn nhận diện đúng.
        uint16_t channelMetadata = rawSample & static_cast<uint16_t>(~Constant::ADC_RESOLUTION_MAX);
        buffer[sampleIndex] = channelMetadata | static_cast<uint16_t>(noisyValue);

    }
}

uint32_t SimulatorDMAApp::fireSimulatedTransmission(TransmitterDMAApp& transmitterApp, ComManager::PulseType pulseType, float txGain) {
    // Phát chuỗi xung ghép (xung gốc + khoảng lặng 500 mẫu + xung echo) một lần duy nhất ra DAC
    return transmitterApp.transmitSimulationBurst(pulseType, txGain, Constant::SIMULATOR_DELAY_SAMPLES, 0.15f);
}

#endif // SIMULATION_MODE
