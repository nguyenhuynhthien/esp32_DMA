#include "SyncSignalDMAApp.hpp"
#include <Arduino.h>

SyncSignalDMAApp::SyncSignalDMAApp(AdcDMAService& adcService, TransmitterDMAApp& transmitterApp, ReceiverDMAApp& receiverApp)
    : _adcService(adcService), _transmitterApp(transmitterApp), _receiverApp(receiverApp) {}

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

    // 2. Khởi động lại ADC DMA (chu kỳ mới, SyncTask sở hữu Mutex từ đây)
    _adcService.start();

    // 3. Đọc loại xung
    ComManager::PulseType pulseType = com.getPulseType();

    // 4. Phát DAC đồng bộ ngay lập tức và đo thời gian phát (us)
#ifdef SHOW_SAMPLING_LOG
    // Đo Tx PRI
    static uint64_t last_tx_time = 0;
    uint64_t current_tx_time = esp_timer_get_time();
    double tx_pri_ms = 0.0;
    if (last_tx_time != 0) {
        tx_pri_ms = (double)(current_tx_time - last_tx_time) / 1000.0;
    }
    last_tx_time = current_tx_time;

    uint32_t tx_elapsed_us = _transmitterApp.transmit(pulseType);
    
    // Tính số lượng mẫu phát
    size_t tx_len = (pulseType == ComManager::PULSE_SINGLE) 
                    ? (Constant::FILTER_COEFFS_LEN + Constant::DAC_PULSE_TOTAL_PADDING)
                    : (Constant::BARKER13_PULSE_LEN + Constant::DAC_PULSE_TOTAL_PADDING);
    
    // Tx Fs = Số mẫu * 1000.0 / Thời gian phát (kHz)
    double tx_fs_khz = 0.0;
    if (tx_elapsed_us > 0) {
        tx_fs_khz = (double)tx_len * 1000.0 / (double)tx_elapsed_us;
    }

    // 6. Nhận và xử lý dữ liệu từ ADC DMA
    _receiverApp.receiveAndProcess(com, frameId, priMs, tx_pri_ms, tx_fs_khz);
#else
    _transmitterApp.transmit(pulseType);
    _receiverApp.receiveAndProcess(com, frameId, priMs, 0.0, 0.0);
#endif
}
