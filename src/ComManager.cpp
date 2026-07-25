#include "ComManager.h"

ComManager::ComManager(const char *ssid, const char *password,
                       const char *hostName, uint16_t port)
    : _ssid(ssid), _password(password), _hostName(hostName), _port(port),
      _remotePort(0), _isStreaming(false), _pulseType(PULSE_SINGLE),
      _isServoEnabled(false), _streamMode(STREAM_RAW), _txGain(1.0f),
      _isTxEnabled(false), _targetServoAngle(-1) {
  for (int i = 0; i < 3; ++i) {
    _queuedFrames[i].ready = false;
  }
}

void ComManager::begin() {
  Serial.println("Starting WiFi STA mode...");
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  // Connect to WiFi router
  WiFi.begin(_ssid, _password);

  Serial.print("Connecting to router ");
  Serial.println(_ssid);

  // Wait up to timeout limit for connection to router
  int timeout = 0;
  while (WiFi.status() != WL_CONNECTED &&
         timeout < (int)Constant::WIFI_CONNECT_TIMEOUT_LIMIT) {
    delay(Constant::WIFI_CONNECT_DELAY_MS);
    Serial.print(".");
    timeout++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi Connected to Router!");
    Serial.print("STA IP Address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nCould not connect to router.");
  }

  // Initialize mDNS
  if (!MDNS.begin(_hostName)) {
    Serial.println("Error setting up MDNS!");
  } else {
    Serial.printf("mDNS started. Connect via: %s.local:%d\n", _hostName, _port);
    MDNS.addService("http", "tcp", _port);
  }

  // Start UDP listening
  _udp.begin(_port);
  Serial.printf("UDP Listening on port %d\n", _port);
}

void ComManager::update() {
  int packetSize = _udp.parsePacket();
  if (packetSize > 0) {
    char command[Constant::UDP_BUFFER_SIZE];
    int len = _udp.read(command, sizeof(command) - 1);
    if (len > 0) {
      _remoteIp = _udp.remoteIP();
      _remotePort = _udp.remotePort();
      command[len] = '\0';

      // Trim trailing whitespace/newlines
      while (len > 0 && (command[len - 1] == '\r' || command[len - 1] == '\n' ||
                         command[len - 1] == ' ')) {
        command[--len] = '\0';
      }

      if (strcmp(command, "start") == 0) {
        _udp.beginPacket(_remoteIp, _remotePort);
        _udp.write((const uint8_t *)"ping", 4);
        _udp.endPacket();

        delay(Constant::WIFI_ARP_DELAY_MS);

        _isStreaming = true;
        Serial.printf("Start streaming registered to %s:%d\n",
                      _remoteIp.toString().c_str(), _remotePort);
      } else if (strcmp(command, "stop") == 0) {
        _isStreaming = false;
        Serial.println("Stop streaming registered.");
      } else if (strcmp(command, "cfg:single") == 0) {
        _pulseType = PULSE_SINGLE;
        Serial.println("Pulse config changed: Single");
      } else if (strcmp(command, "cfg:barker13") == 0) {
        _pulseType = PULSE_BARKER13;
        Serial.println("Pulse config changed: Barker13");
      } else if (strcmp(command, "servo:on") == 0) {
        _isServoEnabled = true;
        Serial.println("Servo control enabled.");
      } else if (strcmp(command, "servo:off") == 0) {
        _isServoEnabled = false;
        Serial.println("Servo control disabled.");
      } else if (strcmp(command, "mode:raw") == 0) {
        _streamMode = STREAM_RAW;
        Serial.println("Streaming mode: Raw Signal");
      } else if (strcmp(command, "mode:demod") == 0) {
        _streamMode = STREAM_DEMOD;
        Serial.println("Streaming mode: Demodulated");
      } else if (strcmp(command, "mode:compressed") == 0) {
        _streamMode = STREAM_COMPRESSED;
        Serial.println("Streaming mode: Pulse Compressed");
      } else if (strncmp(command, "tx_atten:", 9) == 0) {
        const char *valStr = command + 9;
        if (strcmp(valStr, "mute") == 0) {
          _txGain = 0.0f;
          Serial.println("Tx attenuation: Mute");
        } else {
          int attenDb = atoi(valStr);
          _txGain = powf(10.0f, -attenDb / 20.0f);
          Serial.printf("Tx attenuation: -%d dB (gain: %.4f)\n", attenDb,
                        _txGain);
        }
      } else if (strcmp(command, "tx:on") == 0) {
        _isTxEnabled = true;
        Serial.println("Tx Enabled");
      } else if (strcmp(command, "tx:off") == 0) {
        _isTxEnabled = false;
        Serial.println("Tx Disabled");
      } else if (strncmp(command, "servo:", 6) == 0) {
        if (strcmp(command, "servo:on") == 0) {
          _isServoEnabled = true;
          Serial.println("Servo control enabled.");
        } else if (strcmp(command, "servo:off") == 0) {
          _isServoEnabled = false;
          Serial.println("Servo control disabled.");
        } else {
          int angle = atoi(command + 6);
          if (angle >= 0 && angle <= 180) {
            _targetServoAngle = angle;
            Serial.printf("Servo command: Move to %d degrees\n", angle);
          }
        }
      }
    }
  }
}

int ComManager::getTargetServoAngle() {
  int angle = _targetServoAngle;
  _targetServoAngle = -1;
  return angle;
}

bool ComManager::isStreaming() { return _isStreaming; }

void ComManager::sendFrame(uint16_t frameId, const int16_t *samples,
                           size_t size, uint8_t receiverId) {
  if (!_isStreaming || size != Constant::ADC_SAMPLES) {
    return;
  }

  const size_t CHUNK_SAMPLES = Constant::CHUNK_SAMPLES;
  const size_t CHUNKS_PER_FRAME = Constant::CHUNKS_PER_FRAME;

  struct __attribute__((packed)) ChunkHeader {
    uint16_t frameId;
    uint8_t chunkIdx;
    uint8_t receiverId;
  };

  for (size_t i = 0; i < CHUNKS_PER_FRAME; ++i) {
    ChunkHeader header;
    header.frameId = frameId;
    header.chunkIdx = (uint8_t)i;
    header.receiverId = receiverId;

    _udp.beginPacket(_remoteIp, _remotePort);
    _udp.write((const uint8_t *)&header, sizeof(header));
    _udp.write((const uint8_t *)(samples + i * CHUNK_SAMPLES),
               CHUNK_SAMPLES * sizeof(int16_t));
    _udp.endPacket();

    // Pace transmission using configured delay to avoid WiFi buffer overflow
    delayMicroseconds(Constant::UDP_PACE_DELAY_US);
  }
}

void ComManager::sendAngle(uint16_t angle) {
  if (_remotePort == 0) {
    return;
  }
  char buf[16];
  int len = snprintf(buf, sizeof(buf), "ang:%d", angle);

  _udp.beginPacket(_remoteIp, _remotePort);
  _udp.write((const uint8_t *)buf, len);
  _udp.endPacket();
}

void ComManager::sendTarget(float range, uint16_t angle, float strength,
                            float velocity, uint8_t receiverId) {
  if (_remotePort == 0) {
    return;
  }
  char buf[64];
  int len = snprintf(buf, sizeof(buf), "target:%.4f,%u,%.2f,%.4f,%u", range,
                     angle, strength, velocity, receiverId);

  _udp.beginPacket(_remoteIp, _remotePort);
  _udp.write((const uint8_t *)buf, len);
  _udp.endPacket();
}

void ComManager::sendFrameAsync(uint16_t frameId, const int16_t *samples,
                                size_t size, uint8_t receiverId) {
  if (!_isStreaming || size != Constant::ADC_SAMPLES) {
    return;
  }
  // Determine queue slot based on receiverId (Rx0 Sum -> slot 2, Rx1 -> slot 0,
  // Rx2 -> slot 1)
  int slot = (receiverId == 0) ? 2 : (receiverId - 1);
  if (_queuedFrames[slot].ready)
    return; // Drop frame if previous one is still sending to avoid corruption

  _queuedFrames[slot].frameId = frameId;
  _queuedFrames[slot].receiverId = receiverId;
  _queuedFrames[slot].samples = samples;
  _queuedFrames[slot].ready = true;
}

bool ComManager::processAsyncSends() {
  bool sentAny = false;
  for (int i = 0; i < 3; ++i) {
    if (_queuedFrames[i].ready) {
      sendFrame(_queuedFrames[i].frameId, _queuedFrames[i].samples,
                Constant::ADC_SAMPLES, _queuedFrames[i].receiverId);
      _queuedFrames[i].ready = false;
      sentAny = true;
      // Yield to let the WiFi stack process and prevent LwIP lockup
      vTaskDelay(pdMS_TO_TICKS(1));
    }
  }
  return sentAny;
}
