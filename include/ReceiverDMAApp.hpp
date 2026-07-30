#ifndef RECEIVER_DMA_APP_HPP
#define RECEIVER_DMA_APP_HPP

#include "AdcDMAService.hpp"
#include "ComManager.h"
#include "Constant.hpp"

class ReceiverDMAApp {
public:
    ReceiverDMAApp(AdcDMAService& adcService);
    void init();
    void receiveAndProcess(ComManager& com, uint16_t& frameId, double priMs);

private:
    AdcDMAService& _adcService;
    uint16_t _raw_adc_buffer[Constant::ADC_SAMPLES];
    int16_t _send_adc_buffer[Constant::ADC_SAMPLES];

    int16_t calculateDcBias();
    void processRawBuffer(int16_t mean);
    void applyIirFilter();
    int findSyncPeak();
    void shiftSignal(int shift);
};

#endif // RECEIVER_DMA_APP_HPP
