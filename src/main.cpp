#include <Arduino.h>
#include <service/ComManager.hpp>
#include <Constant.h>
#include <ESPmDNS.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#include "driver/AdcDMASignal.hpp"
#include "driver/DacDMASignal.hpp"
#include "service/AdcDMAService.hpp"
#include "service/DacDMAService.hpp"
#include "app/TransmitterDMAApp.hpp"
#include "app/ReceiverDMAApp.hpp"
#include "app/SyncSignalDMAApp.hpp"
#include "app/SimulatorDMAApp.hpp"


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
SimulatorDMAApp simulatorApp;
SyncSignalDMAApp syncApp(adcService, transmitterApp, receiverApp1, receiverApp2, simulatorApp);

uint16_t frameId = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);
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
  simulatorApp.init();
  syncApp.init();

  // Tạo NetworkTask chạy trên Core 0 (ưu tiên 4) để xử lý toàn bộ truyền thông UDP (nhận và gửi)
  xTaskCreatePinnedToCore(
      [](void *param) {
        Serial.println("Network Task (Core 0) started.");
        while (true) {
          // Nhận lệnh UDP đến (cần chạy cùng core với việc gửi để an toàn thread)
          com.update();

          // Gửi dữ liệu UDP bất đồng bộ
          if (!com.processAsyncSends()) {
            vTaskDelay(pdMS_TO_TICKS(5));
          } else {
            vTaskDelay(pdMS_TO_TICKS(1));
          }
        }
      },
      "NetworkTask", 4096, nullptr, 4, nullptr, 0
  );

  // Tạo SyncTask chạy trên Core 1 (ưu tiên 20) để chạy vòng lặp thu phát thời gian thực ADC/DAC
  xTaskCreatePinnedToCore(
      [](void *param) {
        Serial.println("Sync Task (Core 1) started.");
        while (true) {
          if (com.isStreaming()) {
            uint64_t start_time = esp_timer_get_time();

            // Xác định target PRI (15ms cho Single Pulse, 20ms cho Barker 13)
            double target_pri = (com.getPulseType() == ComManager::PULSE_SINGLE) ? 15.0 : 20.0;

            static uint64_t last_time = 0;
            uint64_t current_time = esp_timer_get_time();
            double pri_ms = 0.0;
            if (last_time != 0) {
              pri_ms = (double)(current_time - last_time) / 1000.0;
            }
            last_time = current_time;

            syncApp.runIteration(com, frameId, pri_ms);

            // Bù trễ chính xác cao bằng delayMicroseconds để ổn định chu kỳ PRI cố định
            uint64_t elapsed_us = esp_timer_get_time() - start_time;
            uint64_t target_us = (uint64_t)(target_pri * 1000.0);
            if (elapsed_us < target_us) {
              delayMicroseconds(target_us - elapsed_us);
            } else {
              // Nếu bị lố thời gian (quá tải), nhường CPU một chút để tránh Watchdog Trigger
              vTaskDelay(0);
            }
          } else {
            vTaskDelay(pdMS_TO_TICKS(100));
          }
        }
      },
      "SyncTask", 8192, nullptr, 20, nullptr, 1
  );
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
