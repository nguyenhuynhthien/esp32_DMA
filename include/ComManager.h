#ifndef COM_MANAGER_H
#define COM_MANAGER_H

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPmDNS.h>
#include <Constant.hpp>

class ComManager {
public:
    enum PulseType {
        PULSE_SINGLE,
        PULSE_BARKER13
    };

    enum StreamMode {
        STREAM_RAW = 0,
        STREAM_DEMOD = 1,
        STREAM_COMPRESSED = 2
    };

    ComManager(const char* ssid, const char* password, const char* hostName, uint16_t port = Constant::DEFAULT_PORT);
    
    void begin();
    void update();
    bool isStreaming();
    void sendFrame(uint16_t frameId, const int16_t* samples, size_t size, uint8_t receiverId);
    void sendAngle(uint16_t angle);
    void sendTarget(float range, uint16_t angle, float strength, float velocity, uint8_t receiverId);
    void sendFrameAsync(uint16_t frameId, const int16_t* samples, size_t size, uint8_t receiverId);
    bool processAsyncSends();
    PulseType getPulseType() const { return _pulseType; }
    bool isServoEnabled() const { return _isServoEnabled; }
    StreamMode getStreamMode() const { return _streamMode; }
    float getTxGain() const { return _txGain; }
    bool isTxEnabled() const { return _isTxEnabled; }
    int getTargetServoAngle();

private:
    const char* _ssid;
    const char* _password;
    const char* _hostName;
    uint16_t _port;
    
    WiFiUDP _udp;
    IPAddress _remoteIp;
    uint16_t _remotePort;
    bool _isStreaming;
    PulseType _pulseType;
    bool _isServoEnabled;
    StreamMode _streamMode;
    float _txGain;
    bool _isTxEnabled;
    volatile int _targetServoAngle;

    struct QueuedFrame {
        uint16_t frameId;
        const int16_t* samples;
        uint8_t receiverId;
        volatile bool ready;
    };
    QueuedFrame _queuedFrames[3];
};



#endif // COM_MANAGER_H
