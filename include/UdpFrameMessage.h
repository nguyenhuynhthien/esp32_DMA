#ifndef UDP_FRAME_MESSAGE_H
#define UDP_FRAME_MESSAGE_H

#include <Arduino.h>
#include "Constant.hpp"

struct UdpFrameMessage {
    uint16_t frameId;
    uint8_t receiverId;
    int16_t samples[Constant::ADC_SAMPLES];
};

#endif // UDP_FRAME_MESSAGE_H
