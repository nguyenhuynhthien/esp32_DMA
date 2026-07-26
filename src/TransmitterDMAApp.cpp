#include "TransmitterDMAApp.hpp"
#include "Constant.hpp"

#ifdef SHOW_SAMPLING_LOG
TransmitterDMAApp::TransmitterDMAApp(DacDMAService& dacService) : _dacService(dacService), _loopCount(0) {}
#else
TransmitterDMAApp::TransmitterDMAApp(DacDMAService& dacService) : _dacService(dacService) {}
#endif

void TransmitterDMAApp::init() {
    // Tự động thêm bias ở đầu và copy SINGLE_PULSE_WAVE vào buffer
    memset(_single_pulse, Constant::DAC_DC_BIAS, Constant::DAC_PRE_BIAS_SAMPLES);
    memcpy(_single_pulse + Constant::DAC_PRE_BIAS_SAMPLES, Constant::SINGLE_PULSE_WAVE, Constant::FILTER_COEFFS_LEN);

    // Tự động thêm bias ở đầu và copy BARKER13_PULSE_WAVE vào buffer
    memset(_barker13_pulse, Constant::DAC_DC_BIAS, Constant::DAC_PRE_BIAS_SAMPLES);
    memcpy(_barker13_pulse + Constant::DAC_PRE_BIAS_SAMPLES, Constant::BARKER13_PULSE_WAVE, Constant::BARKER13_PULSE_LEN);
}

void IRAM_ATTR TransmitterDMAApp::transmit(ComManager::PulseType pulseType) {
    if (pulseType == ComManager::PULSE_BARKER13) {
        _dacService.transmitPulse(_barker13_pulse, Constant::BARKER13_PULSE_LEN + Constant::DAC_PRE_BIAS_SAMPLES);
    } else {
        _dacService.transmitPulse(_single_pulse, Constant::FILTER_COEFFS_LEN + Constant::DAC_PRE_BIAS_SAMPLES);
    }
}

#ifdef SHOW_SAMPLING_LOG
void TransmitterDMAApp::printDacMetrics() {
    _loopCount++;
    if (_loopCount % Constant::LOG_INTERVAL_FRAMES == 0) {
        uint64_t elapsed_cycles = _dacService.getLastTransmitCycles();
        size_t length = _dacService.getLastTransmitLength();
        uint32_t cpu_freq_mhz = ESP.getCpuFreqMHz();
        double elapsed_time = (double)elapsed_cycles / (double)cpu_freq_mhz;
        double fs_dac = (double)length * 1000000.0 / elapsed_time;
        Serial.printf("[LOG DAC] PRI: %.2f ms | Số mẫu: %u | Tần số phát thực tế: %.2f kHz (Phát trong %.2f us)\n",
                      elapsed_time / 1000.0, length, fs_dac / 1000.0, elapsed_time);
    }
}
#endif
