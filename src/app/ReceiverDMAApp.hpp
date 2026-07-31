#ifndef RECEIVER_DMA_APP_HPP
#define RECEIVER_DMA_APP_HPP

#include "service/AdcDMAService.hpp"
#include "service/ComManager.hpp"
#include <Constant.h>
#include "SimulatorDMAApp.hpp"


class ReceiverDMAApp {
public:
    ReceiverDMAApp(AdcDMAService& adcService, uint8_t receiverId);
    void init();
    void receiveAndProcess(ComManager& com, uint16_t& frameId, double priMs, double txPriMs = 0.0, double txFsKhz = 0.0, uint64_t adcStartTime = 0);
    void process(const uint16_t* rawSamples, ComManager& com, uint16_t frameId, double priMs, double txPriMs, double txFsKhz, uint64_t elapsed_time, SimulatorDMAApp* simulatorApp = nullptr);

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

#ifdef SHOW_SAMPLING_LOG
    uint32_t _loopCount = 0;
    uint32_t _dropCount = 0;
#endif
    uint32_t _sendCount = 0;
};

#endif // RECEIVER_DMA_APP_HPP
