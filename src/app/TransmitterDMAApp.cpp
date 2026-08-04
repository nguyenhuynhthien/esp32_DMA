#include "TransmitterDMAApp.hpp"
#include <Constant.h>

TransmitterDMAApp::TransmitterDMAApp(DacDMAService& dacService) : _dacService(dacService) {}

void TransmitterDMAApp::init() {
    // Tự động thêm điểm bias (127) ở đầu và copy SINGLE_PULSE_WAVE nguyên bản 100% vào buffer
    memset(_single_pulse, Constant::DAC_DC_BIAS, Constant::DAC_PULSE_PADDING);
    memcpy(_single_pulse + Constant::DAC_PULSE_PADDING, Constant::SINGLE_PULSE_WAVE, Constant::FILTER_COEFFS_LEN);
    memset(_single_pulse + Constant::DAC_PULSE_PADDING + Constant::FILTER_COEFFS_LEN, Constant::DAC_DC_BIAS, Constant::DAC_PULSE_PADDING);

    // Barker13 nguyên bản 100%
    memset(_barker13_pulse, Constant::DAC_DC_BIAS, Constant::DAC_PULSE_PADDING);
    memcpy(_barker13_pulse + Constant::DAC_PULSE_PADDING, Constant::BARKER13_PULSE_WAVE, Constant::BARKER13_PULSE_LEN);
    memset(_barker13_pulse + Constant::DAC_PULSE_PADDING + Constant::BARKER13_PULSE_LEN, Constant::DAC_DC_BIAS, Constant::DAC_PULSE_PADDING);

#ifdef SIMULATION_MODE
    buildSimulationEchoPulse(
        _single_pulse, _single_echo_pulse,
        Constant::FILTER_COEFFS_LEN + Constant::DAC_PULSE_TOTAL_PADDING);
    buildSimulationEchoPulse(
        _barker13_pulse, _barker13_echo_pulse,
        Constant::BARKER13_PULSE_LEN + Constant::DAC_PULSE_TOTAL_PADDING);
#endif
}

uint32_t IRAM_ATTR TransmitterDMAApp::transmit(ComManager::PulseType type, float txGain) {
    size_t len = 0;
    const uint8_t* source_pulse = nullptr;
    
    if (type == ComManager::PULSE_SINGLE) {
        len = Constant::FILTER_COEFFS_LEN + Constant::DAC_PULSE_TOTAL_PADDING;
        source_pulse = _single_pulse;
    } else {
        len = Constant::BARKER13_PULSE_LEN + Constant::DAC_PULSE_TOTAL_PADDING;
        source_pulse = _barker13_pulse;
    }

    // Nếu gain gần bằng 1.0 (không suy hao), phát trực tiếp để tối ưu tốc độ
    if (txGain >= Constant::TX_ATTEN_UNITY_THRESHOLD) {
        return _dacService.transmitPulse(source_pulse, len);
    }

    // Áp dụng suy hao động lên bản copy tạm thời trên Stack
    // Giới hạn len tối đa để buffer stack an toàn (Barker 13 + padding = 104 + 8 = 112)
    uint8_t attenuated_pulse[Constant::ATTENUATED_BUFFER_SIZE];
    for (size_t i = 0; i < len; ++i) {
        int32_t deviation = (int32_t)source_pulse[i] - Constant::DAC_DC_BIAS;
        int32_t attenuated_val = Constant::DAC_DC_BIAS + (int32_t)(deviation * txGain);
        attenuated_pulse[i] = (uint8_t)constrain(attenuated_val, Constant::DAC_MIN_VAL, Constant::DAC_MAX_VAL);
    }

    return _dacService.transmitPulse(attenuated_pulse, len);
}

#ifdef SIMULATION_MODE
void TransmitterDMAApp::buildSimulationEchoPulse(const uint8_t* source_pulse,
                                                 uint8_t* echo_pulse,
                                                 size_t pulse_len) {
    // Chỉ tạo envelope trên phần waveform; các mẫu padding ở hai đầu giữ mức bias.
    const size_t waveform_start = Constant::DAC_PULSE_PADDING;
    const size_t waveform_end = pulse_len - Constant::DAC_PULSE_PADDING;
    const size_t waveform_len = waveform_end - waveform_start;

    // Số mẫu dùng cho sườn lên và sườn xuống của echo.
    const size_t edge_samples =
        (Constant::SIMULATOR_EDGE_SAMPLES < waveform_len / 2)
            ? Constant::SIMULATOR_EDGE_SAMPLES
            : waveform_len / 2;

    // Echo có tần số mang = tần số phát + độ dịch Doppler.
    // Tính bước thay đổi pha cho mỗi mẫu DAC.
    const float doppler_phase_increment =
        2.0f * PI * Constant::SIMULATOR_DOPPLER_SHIFT_HZ /
        static_cast<float>(Constant::SAMPLE_RATE);
    const float carrier_phase_increment =
        2.0f * PI / static_cast<float>(Constant::DEMOD_SAMPLE_PERIOD) +
        doppler_phase_increment;

    for (size_t i = 0; i < pulse_len; ++i) {
        // Envelope tuyến tính: padding = 0, đầu xung tăng dần, giữa xung = 1,
        // cuối xung giảm dần về 0.
        float edge_gain = 1.0f;
        if (i < waveform_start || i >= waveform_end) {
            edge_gain = 0.0f;
        } else if (edge_samples == 0) {
            edge_gain = 1.0f;
        } else if (i < waveform_start + edge_samples) {
            edge_gain = static_cast<float>(i - waveform_start) / edge_samples;
        } else if (i >= waveform_end - edge_samples) {
            edge_gain = static_cast<float>(waveform_end - i) / edge_samples;
        }

        int32_t deviation = 0;
        if (i >= waveform_start && i < waveform_end) {
            // Xác định mẫu đang thuộc chip nào của waveform (mỗi chip có 8 mẫu).
            const size_t waveform_index = i - waveform_start;
            const size_t chip_index = waveform_index / Constant::FILTER_COEFFS_LEN;

            // Mẫu thứ 2 trong chip là đỉnh carrier, dùng để xác định chip +1/-1.
            const size_t chip_peak_index =
                waveform_start + chip_index * Constant::FILTER_COEFFS_LEN + 1;
            const int32_t chip_peak_deviation =
                static_cast<int32_t>(source_pulse[chip_peak_index]) -
                Constant::DAC_DC_BIAS;
            const float chip_polarity =
                (chip_peak_deviation < 0) ? -1.0f : 1.0f;
            const float carrier_amplitude = static_cast<float>(
                (chip_peak_deviation < 0) ? -chip_peak_deviation : chip_peak_deviation);

            // Sinh carrier tại vị trí hiện tại. Bước pha đã bao gồm Doppler.
            const float carrier_sample =
                sinf(carrier_phase_increment * static_cast<float>(waveform_index));

            // Kết hợp biên độ carrier, dấu chip Barker và envelope sườn.
            deviation = static_cast<int32_t>(carrier_amplitude * chip_polarity *
                                              carrier_sample * edge_gain);
        }

        // Chuyển tín hiệu xoay quanh 0 về miền mã DAC xoay quanh DAC_DC_BIAS.
        const int32_t value = Constant::DAC_DC_BIAS + deviation;
        echo_pulse[i] = static_cast<uint8_t>(
            constrain(value, Constant::DAC_MIN_VAL, Constant::DAC_MAX_VAL));
    }
}

uint32_t IRAM_ATTR TransmitterDMAApp::transmitSimulationBurst(ComManager::PulseType type, float txGain, size_t delaySamples, float echoGain) {
    size_t pulse_len = 0;
    const uint8_t* source_pulse = nullptr;
    const uint8_t* echo_pulse = nullptr;
    
    if (type == ComManager::PULSE_SINGLE) {
        pulse_len = Constant::FILTER_COEFFS_LEN + Constant::DAC_PULSE_TOTAL_PADDING;
        source_pulse = _single_pulse;
        echo_pulse = _single_echo_pulse;
    } else {
        pulse_len = Constant::BARKER13_PULSE_LEN + Constant::DAC_PULSE_TOTAL_PADDING;
        source_pulse = _barker13_pulse;
        echo_pulse = _barker13_echo_pulse;
    }

    // Đảm bảo delaySamples lớn hơn pulse_len
    if (delaySamples < pulse_len) {
        return 0;
    }

    // Đảm bảo không tràn buffer _burst_buffer (kích thước tối đa là 800)
    if (pulse_len + delaySamples > 800) {
        return 0;
    }

    size_t write_idx = 0;

    // 1. Ghi xung đồng bộ chính (áp dụng txGain)
    for (size_t i = 0; i < pulse_len; ++i) {
        int32_t deviation = (int32_t)source_pulse[i] - Constant::DAC_DC_BIAS;
        int32_t val = Constant::DAC_DC_BIAS + (int32_t)(deviation * txGain);
        _burst_buffer[write_idx++] = (uint8_t)constrain(val, Constant::DAC_MIN_VAL, Constant::DAC_MAX_VAL);
    }

    // 2. Điền khoảng lặng (mức bias 127) tương ứng với delaySamples - pulse_len
    size_t silence_samples = delaySamples - pulse_len;
    for (size_t i = 0; i < silence_samples; ++i) {
        _burst_buffer[write_idx++] = Constant::DAC_DC_BIAS;
    }

    const float total_echo_gain = txGain * echoGain;

    // 3. Áp dụng gain lên echo đã tính sẵn.
    for (size_t i = 0; i < pulse_len; ++i) {
        const int32_t deviation =
            static_cast<int32_t>(echo_pulse[i]) - Constant::DAC_DC_BIAS;
        const int32_t val = Constant::DAC_DC_BIAS +
                            static_cast<int32_t>(deviation * total_echo_gain);
        _burst_buffer[write_idx++] = static_cast<uint8_t>(
            constrain(val, Constant::DAC_MIN_VAL, Constant::DAC_MAX_VAL));
    }

    // 4. Phát toàn bộ burst ra DAC một lần duy nhất
    return _dacService.transmitPulse(_burst_buffer, write_idx);
}
#endif // SIMULATION_MODE
