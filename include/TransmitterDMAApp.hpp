#ifndef TRANSMITTER_DMA_APP_HPP
#define TRANSMITTER_DMA_APP_HPP

#include "DacDMAService.hpp"

class TransmitterDMAApp {
public:
    TransmitterDMAApp(DacDMAService& dacService);
    void init();
    void IRAM_ATTR transmit();

private:
    DacDMAService& _dacService;
};

#endif // TRANSMITTER_DMA_APP_HPP
