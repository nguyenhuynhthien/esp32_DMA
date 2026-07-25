#include "SyncSignalDMAApp.hpp"
#include <Arduino.h>

SyncSignalDMAApp::SyncSignalDMAApp(AdcDMAService& adcService, TransmitterDMAApp& transmitterApp, ReceiverDMAApp& receiverApp)
    : _adcService(adcService), _transmitterApp(transmitterApp), _receiverApp(receiverApp) {}

void SyncSignalDMAApp::init() {
    // Services and Apps are initialized externally or coordinated
}

void IRAM_ATTR SyncSignalDMAApp::runIteration(ComManager& com, uint16_t& frameId, double priMs) {
    // 1. Dừng ADC DMA trước để đưa về trạng thái tĩnh hoàn toàn
    _adcService.stop();

    // 2. Khởi động lại ADC DMA (chu kỳ mới)
    _adcService.start();
    uint64_t adcStartTime = esp_timer_get_time();

    // 3. Vào vùng chặn ngắt để phát DAC đồng bộ ngay lập tức (không delay)
    portMUX_TYPE myMutex = SPINLOCK_INITIALIZER;
    portENTER_CRITICAL(&myMutex);
    _transmitterApp.transmit(com.getPulseType());
    portEXIT_CRITICAL(&myMutex);

    // 4. Nhận và xử lý dữ liệu từ ADC DMA
    _receiverApp.receiveAndProcess(com, frameId, priMs, adcStartTime);
}
