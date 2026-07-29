#include "SimulatorDMAApp.hpp"
#include "../service/Q15SimdHelper.h"

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
}



void SimulatorDMAApp::injectSimulationQ15(int16_t* sendBuffer, size_t size, ComManager::PulseType pulseType, uint16_t frameId, double priMs, bool txEnabled) {
    if (sendBuffer == nullptr || size <= Constant::SIMULATOR_DELAY_SAMPLES) {
        return;
    }

    // 1. Tiêm nhiễu Gauss bằng cách lấy mẫu tuần hoàn từ bảng nhiễu đã sinh sẵn (tối ưu hóa tốc độ cực đại)
    size_t start_idx = random(0, NOISE_TABLE_SIZE);
    size_t i = 0;
    for (; i + 1 < size; i += 2) {
        Q15x2 signal = q15x2_make(sendBuffer[i + 1], sendBuffer[i]);
        Q15x2 noise = q15x2_make(_noise_table[(start_idx + i + 1) % NOISE_TABLE_SIZE],
                                 _noise_table[(start_idx + i) % NOISE_TABLE_SIZE]);
        Q15x2 mixed = q15x2_add_sat(signal, noise);
        sendBuffer[i] = mixed.lane[0];
        sendBuffer[i + 1] = mixed.lane[1];
    }
    if (i < size) sendBuffer[i] = q15x2_add_sat(q15x2_make(sendBuffer[i], 0),
                                                q15x2_make(_noise_table[(start_idx + i) % NOISE_TABLE_SIZE], 0)).lane[0];

    // 2. Tính toán các thông số Doppler sử dụng float (Hardware FPU của ESP32)
    float fd = 0.0f;
    if (pulseType == ComManager::PULSE_BARKER13) {
        fd = 12.5f; // PRF = 50 Hz, fd = 12.5 Hz (tương đương dịch pha delta_phi = pi/2 rad mỗi frame)
    } else {
        fd = 16.67f; // PRF = 66.67 Hz, fd = 16.67 Hz (tương đương dịch pha delta_phi = pi/2 rad mỗi frame)
    }

    // Tần số lấy mẫu fs = 160000 Hz, góc pha Doppler mỗi mẫu (theta_d)
    float theta_d = 2.0f * PI * fd / 160000.0f;
    // Pha slow-time tích lũy giữa các frame
    float delta_phi = 2.0f * PI * fd * (static_cast<float>(priMs) / 1000.0f);
    float initial_phase = static_cast<float>(frameId) * delta_phi;

    size_t pulse_len = 0;
    if (pulseType == ComManager::PULSE_BARKER13) {
        pulse_len = Constant::BARKER13_PULSE_LEN;
    } else {
        pulse_len = Constant::FILTER_COEFFS_LEN;
    }

    // Không bật Tx thì chỉ tiêm nhiễu nền để mô phỏng môi trường thu tĩnh.
    if (!txEnabled) {
        return;
    }

    // Định nghĩa mã Barker 13 (gồm 13 chip, mỗi chip 8 mẫu)
    const int8_t BARKER_CODE[13] = {1, 1, 1, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1};
    float amplitude = 10000.0f; // Tăng biên độ suy hao của xung echo mô phỏng lên 10000 (~30% dải Q15_MAX)

    for (size_t i = 0; i < pulse_len; ++i) {
        size_t target_idx = Constant::SIMULATOR_DELAY_SAMPLES + i;
        if (target_idx >= size) {
            break;
        }

        // Xác định Code(i)
        float code_val = 1.0f;
        if (pulseType == ComManager::PULSE_BARKER13) {
            code_val = static_cast<float>(BARKER_CODE[i / 8]);
        }

        // Xác định Envelope(i) (méo sườn tăng/giảm ở đầu và cuối xung)
        float env = 1.0f;
        size_t ramp_len = pulse_len / 10;
        if (ramp_len == 0) ramp_len = 1;
        if (i < ramp_len) {
            env = static_cast<float>(i) / static_cast<float>(ramp_len);
        } else if (i >= pulse_len - ramp_len) {
            env = static_cast<float>(pulse_len - 1 - i) / static_cast<float>(ramp_len);
        }

        // Tính pha tức thời: i * pi/2 + i * theta_d + initial_phase
        float phase = static_cast<float>(i) * (PI / 2.0f) + static_cast<float>(i) * theta_d + initial_phase;
        
        // Tạo xung mô phỏng dạng Q15 sử dụng hàm sinf() tối ưu cho hardware FPU của ESP32
        int32_t sim_val_q15 = static_cast<int32_t>(amplitude * env * code_val * sinf(phase));

        // Cộng vào buffer Q15 đã đồng bộ
        int32_t current_val = sendBuffer[target_idx];
        sendBuffer[target_idx] = static_cast<int16_t>(constrain(current_val + sim_val_q15, Constant::Q15_MIN, Constant::Q15_MAX));
    }
}
