#ifndef RECEIVER_DMA_APP_HPP
#define RECEIVER_DMA_APP_HPP

#ifdef SIMULATION_MODE
#include "SimulatorDMAApp.hpp"
#endif
#include "service/AdcDMAService.hpp"
#include "service/ComManager.hpp"
#include <Constant.h>

void printPendingDspTimingLogs();

class ReceiverDMAApp {
public:
  ReceiverDMAApp(AdcDMAService &adcService, uint8_t receiverId);
  void init();
  void receiveAndProcess(ComManager &com, uint16_t frameId, double priMs,
                         double txPriMs = 0.0, double txFsKhz = 0.0,
                         uint64_t adcStartTime = 0);
  void process(const uint16_t *rawSamples, ComManager &com, uint16_t frameId,
               double priMs, double txPriMs, double txFsKhz,
               uint64_t elapsed_time, 
               bool txEnabled = false);

private:
  AdcDMAService &_adcService;
  uint8_t _receiverId;


  int16_t calculateDcBias();
  void processRawBuffer(int16_t mean);
  void applyIirFilter();
  int findSyncPeak(float txGain = 1.0f);
  void shiftSignal(int shift);
  void performIQDemodulation(const int16_t* rawSamples);
  static uint32_t isqrt32(uint32_t n);

  // Matched filter declarations
  struct FilterTap {
      uint8_t k;
      int16_t val;
  };
  FilterTap _tapsI[Constant::BARKER13_PULSE_LEN];
  FilterTap _tapsQ[Constant::BARKER13_PULSE_LEN];
  int _numTapsI = 0;
  int _numTapsQ = 0;
  int _filterLen = 32;
  int _matchedFilterShift = 0;
  ComManager::PulseType _coeffPulseType = ComManager::PULSE_SINGLE;
  bool _coefficientsInitialized = false;

  void initDSPCoefficients(ComManager::PulseType pulseType);
  void performMatchedFiltering();
  void performBarker13MatchedFiltering(int filterEnd, int32_t roundOffset);

  int16_t _cached_bias = 2048;
  int _frame_count = 0;

#ifdef SHOW_SAMPLING_LOG
  uint32_t _loopCount = 0;
  uint32_t _dropCount = 0;
  uint32_t _log_cnt = 0;
#endif
  uint32_t _sendCount = 0;
};

#endif // RECEIVER_DMA_APP_HPP
