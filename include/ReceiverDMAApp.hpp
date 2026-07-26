#ifndef RECEIVER_DMA_APP_HPP
#define RECEIVER_DMA_APP_HPP

#include "AdcDMAService.hpp"
#include "ComManager.h"
#include "Constant.hpp"

class ReceiverDMAApp {
public:
    ReceiverDMAApp(AdcDMAService& adcService, uint8_t receiverId);
    void init();
    void receiveAndProcess(ComManager& com, uint16_t& frameId, double priMs, double txPriMs = 0.0, double txFsKhz = 0.0, uint64_t adcStartTime = 0);
    void process(const uint16_t* rawSamples, ComManager& com, uint16_t frameId, double priMs, double txPriMs, double txFsKhz, uint64_t elapsed_time);

private:
    AdcDMAService& _adcService;
    uint8_t _receiverId;
    uint16_t _raw_adc_buffer[Constant::ADC_SAMPLES];
    int16_t _send_adc_buffer[Constant::ADC_SAMPLES];

    int16_t calculateDcBias();
    void processRawBuffer(int16_t mean);
    void applyIirFilter();
    int findSyncPeak();
    void shiftSignal(int shift);

    uint64_t _last_rx_start_time = 0;
    uint32_t _sendCount = 0;
};

#endif // RECEIVER_DMA_APP_HPP
