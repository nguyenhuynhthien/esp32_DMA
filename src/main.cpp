#include <Arduino.h>
#include "service/ComManager.h"
#include "../include/Constants.h"
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
#include "app/UdpFrameMessage.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

QueueHandle_t udpQueue = nullptr;

void udpSendTask(void* pvParameters) {
    ComManager* pCom = (ComManager*)pvParameters;
    static UdpFrameMessage msg;
    Serial.println("[UDP Task] Bắt đầu chạy trên Core 0...");
    while (true) {
        if (xQueueReceive(udpQueue, &msg, portMAX_DELAY) == pdTRUE) {
#ifdef SHOW_SAMPLING_LOG
            uint64_t t_start = esp_timer_get_time();
#ifdef TRACE_TASK_TIMING
            Serial.printf("[TRACE][CORE %d][UDP] Dequeue frame=%u rx=%u\n",
                          xPortGetCoreID(), msg.frameId, msg.receiverId);
#endif
#endif
            pCom->sendFrame(msg.frameId, msg.samples, Constant::ADC_SAMPLES, msg.receiverId);
#ifdef SHOW_SAMPLING_LOG
            uint64_t t_end = esp_timer_get_time();
            static int udp_profile_cnt = 0;
            if (udp_profile_cnt++ % Constant::LOG_INTERVAL_FRAMES == 0) {
                Serial.printf("[PROFILE CORE 0] UDP Transmit thực tế: %llu us (RX%d, Frame %u)\n", 
                              t_end - t_start, msg.receiverId, msg.frameId);
            }
#ifdef TRACE_TASK_TIMING
            Serial.printf("[TRACE][CORE %d][UDP] frame=%u rx=%u took=%llu us\n",
                          xPortGetCoreID(), msg.frameId, msg.receiverId,
                          t_end - t_start);
#endif
#endif
        }
    }
}

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
  simulatorApp.init();
  syncApp.init();

  // Nâng ưu tiên của loopTask lên MAIN_TASK_PRIORITY (cao hơn WiFi/mạng) để triệt tiêu Jitter khi lập lịch
  vTaskPrioritySet(NULL, Constant::MAIN_TASK_PRIORITY);

  // Khởi tạo hàng đợi UDP
  udpQueue = xQueueCreate(Constant::UDP_QUEUE_LEN, sizeof(UdpFrameMessage));
  if (udpQueue == nullptr) {
      Serial.println("Lỗi tạo UDP Queue!");
  } else {
      // Khởi tạo Task truyền UDP trên Core 0
      xTaskCreatePinnedToCore(
          udpSendTask,
          "UdpSendTask",
          Constant::UDP_TASK_STACK_SIZE,
          &com,
          Constant::UDP_TASK_PRIORITY,
          NULL,
          0 // Ghim task vào Core 0
      );
  }
}

uint16_t frameId = 0;

void loop() {
  // Xử lý các gói tin UDP và cập nhật trạng thái kết nối
#ifdef TRACE_TASK_TIMING
  uint64_t loop_begin = esp_timer_get_time();
  Serial.printf("[TRACE][CORE %d][LOOP] begin frame=%u streaming=%d\n",
                xPortGetCoreID(), frameId, com.isStreaming() ? 1 : 0);
#endif
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
    syncApp.runIteration(com, frameId, pri_ms, udpQueue);

    // Trễ chủ động
    delay(0);
  } else {
    // Chờ kết nối
    delay(Constant::WAIT_CONNECT_DELAY_MS);
  }
#ifdef TRACE_TASK_TIMING
  Serial.printf("[TRACE][CORE %d][LOOP] end frame=%u duration=%llu us\n",
                xPortGetCoreID(), frameId, esp_timer_get_time() - loop_begin);
#endif
}
