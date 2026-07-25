#ifndef TRANSMITTER_DMA_APP_HPP
#define TRANSMITTER_DMA_APP_HPP

#include "DacDMAService.hpp"
#include "Constant.hpp"

class TransmitterDMAApp {
public:
    TransmitterDMAApp(DacDMAService& dacService);
    void init();
    void IRAM_ATTR transmit();

private:
    DacDMAService& _dacService;
    uint8_t _single_pulse[Constant::FILTER_COEFFS_LEN + 4];
    uint8_t _barker13_pulse[Constant::BARKER13_PULSE_LEN + 4];
};

#endif // TRANSMITTER_DMA_APP_HPP
