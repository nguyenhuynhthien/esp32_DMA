#ifndef SYNC_SIGNAL_DMA_APP_HPP
#define SYNC_SIGNAL_DMA_APP_HPP

#include "AdcDMAService.hpp"
#include "TransmitterDMAApp.hpp"
#include "ReceiverDMAApp.hpp"
#include "ComManager.h"

class SyncSignalDMAApp {
public:
    SyncSignalDMAApp(AdcDMAService& adcService, TransmitterDMAApp& transmitterApp, ReceiverDMAApp& receiverApp1, ReceiverDMAApp& receiverApp2);
    void init();
    void IRAM_ATTR runIteration(ComManager& com, uint16_t& frameId, double priMs);

private:
    AdcDMAService& _adcService;
    TransmitterDMAApp& _transmitterApp;
    ReceiverDMAApp& _receiverApp1;
    ReceiverDMAApp& _receiverApp2;
};

#endif // SYNC_SIGNAL_DMA_APP_HPP
