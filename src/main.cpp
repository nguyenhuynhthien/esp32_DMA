#include <Arduino.h>
#include <ComManager.h>
#include <Constant.hpp>
#include <ESPmDNS.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#include "AdcDMASignal.hpp"
#include "DacDMASignal.hpp"
#include "AdcDMAService.hpp"
#include "DacDMAService.hpp"
#include "TransmitterDMAApp.hpp"
#include "ReceiverDMAApp.hpp"
#include "SyncSignalDMAApp.hpp"

// Thông tin kết nối WiFi
const char *ssid = "Noel";
const char *password = "hongthanh2110";
const char *hostName = "esp32";

ComManager com(ssid, password, hostName);

// Khởi tạo các Driver
AdcDMASignal adcSignal;
DacDMASignal dacSignal;

// Khởi tạo các Service
AdcDMAService adcService(adcSignal);
DacDMAService dacService(dacSignal);

// Khởi tạo các App
TransmitterDMAApp transmitterApp(dacService);
ReceiverDMAApp receiverApp1(adcService, Constant::RECEIVER_ID_RX1);
ReceiverDMAApp receiverApp2(adcService, Constant::RECEIVER_ID_RX2);
SyncSignalDMAApp syncApp(adcService, transmitterApp, receiverApp1, receiverApp2);

void setup() {
  Serial.begin(Constant::SERIAL_BAUD_RATE);
  delay(Constant::SETUP_DELAY_MS);
  Serial.println("Khởi động hệ thống test_DMA...");

  // 1. Khởi tạo ComManager để kết nối WiFi trước khi cấu hình ADC
  com.begin();

  // 2. Khởi tạo DAC Service
  dacService.init();

  // 3. Khởi tạo ADC Service
  adcService.init();

  // 4. Khởi tạo các ứng dụng
  transmitterApp.init();
  receiverApp1.init();
  receiverApp2.init();
  syncApp.init();

  // Nâng ưu tiên của loopTask lên MAIN_TASK_PRIORITY (cao hơn WiFi/mạng) để triệt tiêu Jitter khi lập lịch
  vTaskPrioritySet(NULL, Constant::MAIN_TASK_PRIORITY);
}

uint16_t frameId = 0;

void loop() {
  // Xử lý các gói tin UDP và cập nhật trạng thái kết nối
  com.update();

  if (com.isStreaming()) {
    static uint64_t last_time = 0;
    uint64_t current_time = esp_timer_get_time();
    double pri_ms = 0.0;
    if (last_time != 0) {
      pri_ms = (double)(current_time - last_time) / 1000.0;
    }
    last_time = current_time;

    // Chạy chu kỳ phát xung và thu tín hiệu đồng bộ thông qua SyncSignalDMAApp
    syncApp.runIteration(com, frameId, pri_ms);

    // Trễ chủ động
    delay(0);
  } else {
    // Chờ kết nối
    delay(Constant::WAIT_CONNECT_DELAY_MS);
  }
}
