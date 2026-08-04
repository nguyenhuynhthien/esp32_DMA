#ifndef SYNC_SIGNAL_DMA_APP_HPP
#define SYNC_SIGNAL_DMA_APP_HPP

#include "service/AdcDMAService.hpp"
#include "TransmitterDMAApp.hpp"
#include "ReceiverDMAApp.hpp"
#ifdef SIMULATION_MODE
#include "SimulatorDMAApp.hpp"
#endif
#include "service/ComManager.hpp"

void printPendingFrameTimingLog();


class SyncSignalDMAApp {
public:
#ifdef SIMULATION_MODE
    // Nhận simulator để chèn nhiễu sau khi đọc ADC.
    SyncSignalDMAApp(AdcDMAService& adcService, TransmitterDMAApp& transmitterApp, ReceiverDMAApp& receiverApp1, ReceiverDMAApp& receiverApp2, SimulatorDMAApp& simulatorApp);
#else
    SyncSignalDMAApp(AdcDMAService& adcService, TransmitterDMAApp& transmitterApp, ReceiverDMAApp& receiverApp1, ReceiverDMAApp& receiverApp2);
#endif
    void init();
    void IRAM_ATTR runIteration(ComManager& com, uint16_t& frameId, double priMs);
    void startParallelProcessing();

private:
    AdcDMAService& _adcService;
    TransmitterDMAApp& _transmitterApp;
    ReceiverDMAApp& _receiverApp1;
    ReceiverDMAApp& _receiverApp2;
#ifdef SIMULATION_MODE
    SimulatorDMAApp& _simulatorApp;
#endif
};

#endif // SYNC_SIGNAL_DMA_APP_HPP
