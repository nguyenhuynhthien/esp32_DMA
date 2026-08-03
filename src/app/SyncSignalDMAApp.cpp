#include "SyncSignalDMAApp.hpp"
#include <Arduino.h>
#include <soc/i2s_struct.h>
#include <soc/i2s_reg.h>

SyncSignalDMAApp::SyncSignalDMAApp(AdcDMAService& adcService, TransmitterDMAApp& transmitterApp, ReceiverDMAApp& receiverApp1, ReceiverDMAApp& receiverApp2)
    : _adcService(adcService), _transmitterApp(transmitterApp), _receiverApp1(receiverApp1), _receiverApp2(receiverApp2) {}

void SyncSignalDMAApp::init() {
    // Services and Apps are initialized externally or coordinated
}

void IRAM_ATTR SyncSignalDMAApp::runIteration(ComManager& com, uint16_t& frameId, double priMs) {
    // 1. Dừng ADC DMA trước để đưa về trạng thái tĩnh hoàn toàn (bỏ qua chu kỳ đầu tiên)
    static bool first_call = true;
    if (!first_call) {
        _adcService.stop();
        // Trễ để phần cứng I2S ổn định hoàn toàn trạng thái tắt, tránh byte-swap (chớp dọc)
        delayMicroseconds(Constant::ADC_STOP_STABILIZATION_US);
    }
    first_call = false;

    // 2. Khởi động lại ADC DMA (chu kỳ mới)
    _adcService.start();

    bool txEnabled = com.isTxEnabled();

    // Đồng bộ: Gửi Task Notification báo hiệu cho TxTask trên Core 0 phát xung ngay lập tức
    extern TaskHandle_t txTaskHandle;
    if (txTaskHandle != nullptr && txEnabled) {
        xTaskNotifyGive(txTaskHandle);
    }

#ifdef SHOW_SAMPLING_LOG
    uint64_t adc_start_time = esp_timer_get_time();
#else
    uint64_t adc_start_time = 0;
#endif

    // 3. Đọc loại xung
    ComManager::PulseType pulseType = com.getPulseType();

    // 4. Lấy thông số đo đạc từ các biến toàn cục (do TxTask đo ở Core 0)
    extern volatile double global_tx_pri_ms;
    extern volatile double global_tx_fs_khz;
    double tx_pri_ms = global_tx_pri_ms;
    double tx_fs_khz = global_tx_fs_khz;

    // 5. Nhận và xử lý dữ liệu từ ADC DMA cho cả 2 kênh (dư thêm 16 mẫu để đảm bảo vòng lặp trích đủ 2048 mẫu)
    static uint16_t raw_interleaved_buffer[Constant::ADC_SAMPLES * 4 + 16];
    static uint16_t rx1_buffer[Constant::ADC_SAMPLES];
    static uint16_t rx2_buffer[Constant::ADC_SAMPLES];
    
    size_t bytes_read = 0;
    esp_err_t res = _adcService.readSamples(raw_interleaved_buffer, sizeof(raw_interleaved_buffer), bytes_read);
    uint64_t elapsed_time = esp_timer_get_time() - adc_start_time;
    // Khấu trừ khoảng 203 us trễ khởi động mềm của driver I2S để tính tần số lấy mẫu thực tế chính xác
    if (elapsed_time > 203) {
        elapsed_time -= 203;
    }

    if (res == ESP_OK && bytes_read == sizeof(raw_interleaved_buffer)) {
        #ifdef SHOW_TIMING_LOG
        uint64_t t_start_demux = esp_timer_get_time();
        #endif

        // 1. Dùng Voting trên 60 mẫu đầu để xác định pha khởi động chuẩn xác tuyệt đối (chống nhiễu khởi động)
        int phase = -1;
        int max_votes = 0;
        for (int p = 0; p < 4; ++p) {
            int votes = 0;
            for (int i = 0; i < 15; ++i) { // Quét qua 15 chu kỳ (60 mẫu)
                uint8_t c0 = (raw_interleaved_buffer[p + 4*i] >> 12) & 0xF;
                uint8_t c1 = (raw_interleaved_buffer[p + 4*i + 1] >> 12) & 0xF;
                uint8_t c2 = (raw_interleaved_buffer[p + 4*i + 2] >> 12) & 0xF;
                uint8_t c3 = (raw_interleaved_buffer[p + 4*i + 3] >> 12) & 0xF;
                if (c0 == 4 && c1 == 4 && c2 == 5 && c3 == 5) {
                    votes++;
                }
            }
            if (votes > max_votes) {
                max_votes = votes;
                phase = p;
            }
        }

        int last_transition_rx1 = -1;
        int last_transition_rx2 = -1;
        size_t rx1_count = 0;
        size_t rx2_count = 0;

        // Nếu vote thành công (tin cậy cao, có ít nhất 8 chu kỳ khớp hoàn hảo), dùng pha đã vote
        if (phase != -1 && max_votes >= 8) {
            last_transition_rx1 = phase + 1;
            last_transition_rx2 = ((phase + 2) & 3) + 1;
            if (last_transition_rx2 < last_transition_rx1) {
                last_transition_rx2 += 4;
            }
            // Đưa mẫu đầu tiên đã sạc đầy của kênh 1 vào bộ đệm
            rx1_buffer[rx1_count++] = raw_interleaved_buffer[last_transition_rx1] & Constant::ADC_RESOLUTION_MAX;
        } else {
            // Fallback: Tìm điểm chuyển tiếp CH4 -> CH5 đầu tiên trong 128 mẫu đầu
            int start_idx = -1;
            for (size_t i = 0; i < 128; ++i) {
                uint8_t c0 = (raw_interleaved_buffer[i] >> 12) & 0xF;
                uint8_t c1 = (raw_interleaved_buffer[i + 1] >> 12) & 0xF;
                if (c0 == 4 && c1 == 5) {
                    start_idx = i;
                    break;
                }
            }
            if (start_idx != -1) {
                last_transition_rx1 = start_idx;
                last_transition_rx2 = start_idx - 2;
                rx1_buffer[rx1_count++] = raw_interleaved_buffer[start_idx] & Constant::ADC_RESOLUTION_MAX;
            }
        }

        // Vòng lặp bánh đà bắt đầu từ điểm pha đã xác định
        if (last_transition_rx1 != -1) {
            size_t total_samples = Constant::ADC_SAMPLES * 4 + 16;
            for (size_t i = last_transition_rx1 + 1; i < total_samples - 1; ++i) {
                uint16_t val0 = raw_interleaved_buffer[i];
                uint16_t val1 = raw_interleaved_buffer[i + 1];
                uint8_t chan0 = val0 >> 12;
                uint8_t chan1 = val1 >> 12;

                bool is_rx1_transition = (chan0 == 4 && chan1 == 5);
                bool is_rx2_transition = (chan0 == 5 && chan1 == 4);

                // Cơ chế Flywheel (Bánh đà): Nếu sau đúng 4 mẫu vẫn không thấy tín hiệu chuyển tiếp (do dV/dt làm nhiễu ID kênh),
                // chúng ta tự động ép sự kiện chuyển tiếp để không bị trượt mẫu.
                if (!is_rx1_transition && (i - last_transition_rx1 >= 4)) {
                    is_rx1_transition = true;
                }
                if (!is_rx2_transition && (i - last_transition_rx2 >= 4)) {
                    is_rx2_transition = true;
                }

                // Cửa sổ trơ (Refractory Window): Từ chối các chuyển tiếp quá gần nhau (nhiễu nhảy nhót < 3 mẫu)
                // để tránh tình trạng kích hoạt chuyển tiếp giả liên tục làm co ngắn xung phát.
                if (is_rx1_transition && (i - last_transition_rx1 < 3)) {
                    is_rx1_transition = false;
                }
                if (is_rx2_transition && (i - last_transition_rx2 < 3)) {
                    is_rx2_transition = false;
                }

                if (is_rx1_transition) {
                    if (rx1_count < Constant::ADC_SAMPLES) {
                        rx1_buffer[rx1_count++] = val0 & Constant::ADC_RESOLUTION_MAX;
                    }
                    last_transition_rx1 = i;
                    last_transition_rx2 = i - 2;
                } else if (is_rx2_transition) {
                    if (rx2_count < Constant::ADC_SAMPLES) {
                        rx2_buffer[rx2_count++] = val0 & Constant::ADC_RESOLUTION_MAX;
                    }
                    last_transition_rx2 = i;
                    last_transition_rx1 = i - 2;
                }
            }
        }

        // Bù mẫu cuối bằng mức bias nếu có lỗi hệ thống không thu đủ mẫu
        while (rx1_count < Constant::ADC_SAMPLES) {
            rx1_buffer[rx1_count++] = (rx1_count > 0) ? rx1_buffer[rx1_count - 1] : 2048;
        }
        while (rx2_count < Constant::ADC_SAMPLES) {
            rx2_buffer[rx2_count++] = (rx2_count > 0) ? rx2_buffer[rx2_count - 1] : 2048;
        }

#ifdef SHOW_TIMING_LOG
        uint64_t demux_time = esp_timer_get_time() - t_start_demux;
#endif

        // Debug first 20 samples channel IDs
        static int debug_cnt = 0;
        if (debug_cnt++ % 50 == 0) {
            Serial.printf("[DEBUG] Channel IDs: ");
            for (int i = 0; i < 20; ++i) {
                Serial.printf("%d ", (raw_interleaved_buffer[i] >> 12) & 0xF);
            }
            Serial.printf("\n[DEBUG] RX1 count (unique): %u, RX2 count (unique): %u\n", rx1_count, rx2_count);
        }

        // DSP luôn chạy cho cả hai receiver; rx_select chỉ quyết định kênh gửi UDP.
        // SyncTask có priority cao; không suspend scheduler vì DSP có logging
        // và các API Arduino có thể cần queue/semaphore.
#ifdef SHOW_TIMING_LOG
        uint64_t t_start_proc1 = esp_timer_get_time();
#endif
        _receiverApp1.process(rx1_buffer, com, frameId, priMs, tx_pri_ms, tx_fs_khz, elapsed_time, txEnabled);
#ifdef SHOW_TIMING_LOG
        uint64_t proc1_time = esp_timer_get_time() - t_start_proc1;
#endif

#ifdef SHOW_TIMING_LOG
        uint64_t t_start_proc2 = esp_timer_get_time();
#endif
        _receiverApp2.process(rx2_buffer, com, frameId, priMs, tx_pri_ms, tx_fs_khz, elapsed_time, txEnabled);
#ifdef SHOW_TIMING_LOG
        uint64_t proc2_time = esp_timer_get_time() - t_start_proc2;
#endif

#ifdef SHOW_TIMING_LOG
        if ((debug_cnt - 1) % 50 == 0) {
            uint64_t stop_start_time = 0;
            Serial.printf("[TIMING] Tre_khoi_dong_ADC: %llu us | Tach_kenh: %llu us | Loc_Dongbo_Gui_Kenh1: %llu us | Loc_Dongbo_Gui_Kenh2: %llu us\n",
                          stop_start_time, demux_time, proc1_time, proc2_time);
        }
#endif

        // Tăng frameId cho chu kỳ phát xung tiếp theo
        frameId++;
    } else {
        Serial.printf("Lỗi đọc I2S: %d, số bytes đọc được: %u\n", res, bytes_read);
    }
}
