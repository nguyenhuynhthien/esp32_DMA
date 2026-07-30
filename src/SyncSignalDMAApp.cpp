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

    // 4. Phát DAC đồng bộ ngay lập tức (không dùng critical section chặn ngắt để tránh làm nghẽn ngắt DMA của ADC)
    _transmitterApp.transmit(pulseType);

    // 4. Nhận và xử lý dữ liệu từ ADC DMA
    _receiverApp.receiveAndProcess(com, frameId, priMs);
}
