#ifndef RECEIVER_DMA_APP_HPP
#define RECEIVER_DMA_APP_HPP

#include "../service/AdcDMAService.hpp"
#include "../service/ComManager.h"
#include "../../include/Constants.h"
#include "SimulatorDMAApp.hpp"

#include "UdpFrameMessage.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

class ReceiverDMAApp {
public:
    ReceiverDMAApp(AdcDMAService& adcService, uint8_t receiverId);
    void init();
    void receiveAndProcess(ComManager& com, uint16_t frameId, double priMs, uint64_t adcStartTime = 0);
    void process(const uint16_t* rawSamples, ComManager& com, uint16_t frameId, double priMs, uint64_t elapsed_time, QueueHandle_t udpQueue = nullptr, SimulatorDMAApp* simulatorApp = nullptr);

private:
    AdcDMAService& _adcService;
    uint8_t _receiverId;
#ifdef SHOW_SAMPLING_LOG
    uint32_t _loopCount;
#endif
    uint16_t _raw_adc_buffer[Constant::ADC_SAMPLES];
    int16_t _send_adc_buffer[Constant::ADC_SAMPLES];
};

#endif // RECEIVER_DMA_APP_HPP
