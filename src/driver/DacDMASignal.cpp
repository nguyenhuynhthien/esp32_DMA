#include "DacDMASignal.hpp"
#include <Constant.h>
#include <xtensa/hal.h>

DacDMASignal::DacDMASignal() {}

void DacDMASignal::init() {
    dac_output_enable(DAC_CHANNEL_1);
    dac_output_voltage(DAC_CHANNEL_1, Constant::DAC_DC_BIAS);
}

void DacDMASignal::setVoltage(uint8_t voltage) {
    dac_output_voltage(DAC_CHANNEL_1, voltage);
}

uint32_t IRAM_ATTR DacDMASignal::firePulse(const uint8_t* pulse, size_t length) {
    uint32_t cpu_freq_mhz = ESP.getCpuFreqMHz();
    uint32_t cycles_per_sample = (uint32_t)(cpu_freq_mhz * Constant::CPU_CYCLES_PER_SAMPLE_FACTOR);

    // Vô hiệu hóa ngắt cục bộ trên Core 0 (không dùng spinlock liên lõi để tránh treo Core 1 phục vụ I2S DMA)
    unsigned int old_int_level;
    __asm__ __volatile__("rsil %0, 3" : "=a" (old_int_level));

    uint32_t start_cycles = get_ccount();
    for (size_t i = 0; i < length; ++i) {
        dac_output_voltage(DAC_CHANNEL_1, pulse[i]);
        while ((int32_t)(get_ccount() - (start_cycles + (i + 1) * cycles_per_sample)) < 0) {
            // Spin-wait
        }
    }

    uint32_t elapsed_cycles = get_ccount() - start_cycles;

    // Return to bias after firing
    dac_output_voltage(DAC_CHANNEL_1, Constant::DAC_DC_BIAS);

    // Khôi phục lại mức ngắt cũ trên Core 0
    __asm__ __volatile__("wsr %0, ps; rsync" : : "a" (old_int_level));

#ifdef SHOW_SAMPLING_LOG
    return elapsed_cycles;
#else
    return 0;
#endif
}
