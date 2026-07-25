#ifndef CONSTANT_HPP
#define CONSTANT_HPP

#include <Arduino.h>

namespace Constant {
// --- Base Sonar & DSP Parameters ---
// Center frequency of the transducer in Hz (Value: 40000.0)
constexpr double CENTER_FREQ = 40000.0;

// Period of I/Q carrier modulation, defined as samples per carrier cycle (Value: 4)
constexpr int DEMOD_SAMPLE_PERIOD = 4;

// --- Derived Sonar & DSP Parameters ---
// ADC sampling rate in Hz, derived from center frequency and demodulation period (Value: 160000.0)
constexpr double SAMPLE_RATE = CENTER_FREQ * DEMOD_SAMPLE_PERIOD;

// --- ADC & DAC Parameters ---
// Maximum value of the 12-bit ADC resolution (Value: 4095)
constexpr uint16_t ADC_RESOLUTION_MAX = 4095;

// DC offset of the ADC, derived as the midpoint of the ADC range (Value: 2048.0f)
constexpr float ADC_DC_OFFSET = static_cast<float>(ADC_RESOLUTION_MAX + 1) / 2.0f;

// DC bias offset for the 8-bit DAC output (Value: 127)
constexpr uint8_t DAC_DC_BIAS = 127;

// ADC channel corresponding to Receiver 1 (GPIO 32, Value: 4)
constexpr int ADC_CHANNEL_RX1 = 4;

// ADC channel corresponding to Receiver 2 (GPIO 33, Value: 5)
constexpr int ADC_CHANNEL_RX2 = 5;

// --- Buffer & Sampling Configuration ---
// Number of samples in a single UDP packet chunk (Value: 512)
constexpr size_t CHUNK_SAMPLES = 512;

// Number of chunks that make up a complete signal frame (Value: 4)
constexpr size_t CHUNKS_PER_FRAME = 4;

// Total number of ADC samples collected per frame, derived from chunk settings (Value: 2048)
constexpr size_t ADC_SAMPLES = CHUNK_SAMPLES * CHUNKS_PER_FRAME;

// Length of each DMA buffer, matching the size of a single sample chunk (Value: 512)
constexpr size_t DMA_BUF_LEN = CHUNK_SAMPLES;

// Number of DMA buffers allocated for I2S continuous sampling (Value: 2)
constexpr size_t DMA_BUF_COUNT = 2;

// --- DSP Processing & Filter Parameters ---
// Window size for FFT spectral smoothing and processing (Value: 15)
constexpr int FFT_WINDOW_SIZE = 15;

// Length of the matched filter coefficients, matching single pulse length (Value: 8)
constexpr size_t FILTER_COEFFS_LEN = 8;

// Total length in samples for the Barker 13 pulse waveform (Value: 104)
constexpr size_t BARKER13_PULSE_LEN = 13 * FILTER_COEFFS_LEN;

// Number of pulses in the slow-time dimension for Doppler processing (Value: 16)
constexpr size_t SLOW_TIME_LEN = 16;

// Length of the FFT used for Doppler velocity calculations (Value: 8)
constexpr size_t DOPPLER_FFT_LEN = 8;

// Conversion factor relating CPU cycles to ADC samples (Value: 6.25f)
constexpr float CPU_CYCLES_PER_SAMPLE_FACTOR = 6.25f;

// Calibration factor to adjust actual physical sampling rate discrepancy (Value: 1.10f)
constexpr float SAMPLING_CALIBRATION_FACTOR = 1.10f;

// Speed of sound in dry air at room temperature in m/s (Value: 343.0f)
constexpr float SPEED_OF_SOUND = 343.0f;

// --- Q15 Fixed-Point Math Parameters ---
// Number of fraction bits in Q15 fixed-point format (Value: 15)
constexpr int Q15_SHIFT = 15;

// Maximum value of a signed 16-bit Q15 fixed-point number (Value: 32767)
constexpr int16_t Q15_MAX = (1 << Q15_SHIFT) - 1;

// Minimum value of a signed 16-bit Q15 fixed-point number (Value: -32768)
constexpr int16_t Q15_MIN = -(1 << Q15_SHIFT);

// Pre-calculated value of cos(pi/4) scaled in Q15 format, approximately 0.7071 (Value: 23170)
constexpr int32_t COS_PI_4_Q15 = static_cast<int32_t>(0.7071067811865476 * (1 << Q15_SHIFT));

// Moving average filter length used in demodulation (Value: 4)
constexpr int DEMOD_AVG_LEN = 4;

// Scaling factor to map Q15 values to 8-bit DAC output range (Value: 256)
constexpr int16_t Q15_DAC_SCALE = 256;

// Right shift amount applied after matched filter multiplication/accumulation (Value: 13)
constexpr int MATCHED_FILTER_SHIFT = 13;

// Rounding offset added before matched filter scaling shift, equal to 2^(shift-1) (Value: 4096)
constexpr int32_t MATCHED_FILTER_ROUND_OFFSET = 1 << (MATCHED_FILTER_SHIFT - 1);

// Squared scale factor for target threshold, representing 1024 squared (Value: 1048576)
constexpr int64_t TARGET_THRESHOLD_SCALE_SQ = 1024LL * 1024LL;

// --- Detection Thresholds & Blanking ---
// Base target detection threshold for Barker 13 pulse compressed signals (Value: 120)
constexpr int32_t BASE_BARKER13_THRESHOLD = 120;

// Base target detection threshold for Single Pulse signals (Value: 300)
constexpr int32_t BASE_SINGLE_PULSE_THRESHOLD = 300;

// Number of initial samples to blank to filter out transmitter leakage (Value: 220)
constexpr int TX_LEAKAGE_BLANK_SAMPLES = 220;

// Scaling factor applied to signal stream before transmission or display (Value: 3)
constexpr int COMPRESSED_STREAM_SCALE = 3;

// --- Waveform Definitions ---
// Pre-calculated single pulse waveform (1 chip = 2 cycles of 4 samples/cycle = 8 samples)
constexpr uint8_t SINGLE_PULSE_WAVE[FILTER_COEFFS_LEN] = {
    127, 254, 127, 0, 127, 254, 127, 0};

// Pre-calculated Barker 13 waveform (13 chips, 2 cycles of 4 samples/cycle per chip = 8 samples/chip, total 104 samples)
constexpr uint8_t BARKER13_PULSE_WAVE[BARKER13_PULSE_LEN] = {
    127, 254, 127, 0,   127, 254, 127, 0,   127, 254, 127, 0,   127, 254, 127,
    0,   127, 254, 127, 0,   127, 254, 127, 0,   127, 254, 127, 0,   127, 254,
    127, 0,   127, 254, 127, 0,   127, 254, 127, 95,  127, 95,  127, 254, 127,
    0,   127, 254, 127, 0,   127, 254, 127, 0,   127, 159, 127, 159, 127, 0,
    127, 254, 127, 0,   127, 254, 127, 0,   127, 254, 127, 95,  127, 95,  127,
    254, 127, 0,   127, 159, 127, 159, 127, 0,   127, 254, 127, 95,  127, 95,
    127, 254, 127, 0,   127, 159, 127, 159, 127, 0,   127, 254, 127, 0};

// --- Timing & Network Timeouts ---
// Delay in microseconds when polling for RX availability (Value: 50)
constexpr uint32_t RX_POLL_DELAY_US = 50;

// Timeout in milliseconds to wait for transmitter response/ready (Value: 80)
constexpr uint32_t TX_RESPONSE_TIMEOUT_MS = 80;

// Step angle in degrees for servo movement (Value: 10)
constexpr int SERVO_STEP_DEG = 10;

// Delay in milliseconds when the transmitter is in an idle state (Value: 20)
constexpr uint32_t TX_IDLE_DELAY_MS = 20;

// Pulse Repetition Interval (PRI) in milliseconds for single pulse mode (Value: 15)
constexpr uint32_t PRI_SINGLE_MS = 15;

// Pulse Repetition Interval (PRI) in milliseconds for Barker 13 mode (Value: 20)
constexpr uint32_t PRI_BARKER13_MS = 20;

// Busy wait time in microseconds during transmissions (Value: 100)
constexpr uint32_t TX_BUSY_WAIT_US = 100;

// Microsecond threshold above which a task yields execution to other tasks (Value: 2000)
constexpr uint32_t TX_YIELD_THRESHOLD_US = 2000;

// Delay in milliseconds between WiFi connection status polls (Value: 500)
constexpr uint32_t WIFI_CONNECT_DELAY_MS = 500;

// Maximum iteration count for WiFi connection retry, representing 10s timeout (Value: 20)
constexpr uint32_t WIFI_CONNECT_TIMEOUT_LIMIT = 10000 / WIFI_CONNECT_DELAY_MS;

// Delay in milliseconds for ARP resolution before sending UDP packets (Value: 300)
constexpr uint32_t WIFI_ARP_DELAY_MS = 300;

// Delay in microseconds between UDP packets to control transmission pacing (Value: 250)
constexpr uint32_t UDP_PACE_DELAY_US = 250;

// --- Network Communication Settings ---
// Default UDP port used for network communication (Value: 8080)
constexpr uint16_t DEFAULT_PORT = 8080;

// Size of the UDP packet payload buffer in bytes (Value: 32)
constexpr size_t UDP_BUFFER_SIZE = 32;

// --- Tasks & System Configuration ---
// Baud rate configuration for Serial terminal communication (Value: 115200)
constexpr uint32_t SERIAL_BAUD_RATE = 115200;

// Delay in milliseconds during setup to allow hardware stabilization (Value: 1000)
constexpr uint32_t SETUP_DELAY_MS = 1000;

// FreeRTOS task stack size allocated for application tasks in bytes (Value: 8192)
constexpr uint32_t TASK_STACK_SIZE = 8192;

// FreeRTOS task priority level assigned to application tasks (Value: 10)
constexpr uint32_t TASK_PRIORITY = 10;

// Delay in milliseconds at the end of execution loops (Value: 1000)
constexpr uint32_t LOOP_DELAY_MS = 1000;

// --- Target Physical Parameters & Calculations ---
// ADC reference voltage in Volts (Value: 3.3f)
constexpr float ADC_REF_VOLTS = 3.3f;

// Gain restoration factor to scale Q15 amplitude values (Value: 1024.0f)
constexpr float GAIN_RESTORATION_FACTOR = 1024.0f;

// Minimum amplitude limit to prevent log10(0) domain errors (Value: 1.0f)
constexpr float MIN_AMPLITUDE_LIMIT = 1.0f;

// Logarithmic multiplier to scale voltage ratios to decibel levels (Value: 20.0f)
constexpr float DB_SCALE_FACTOR = 20.0f;

// Factor representing two-way signal travel path (out and back) (Value: 2.0f)
constexpr float ROUND_TRIP_FACTOR = 2.0f;

// Pulse repetition interval in seconds, converted from milliseconds (Value: 0.03f)
constexpr float PRI_SECONDS = static_cast<float>(PRI_SINGLE_MS) / 1000.0f;
} // namespace Constant

#endif // CONSTANT_HPP
