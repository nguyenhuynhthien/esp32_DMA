#include "AdcDMASignal.hpp"
#include "Constant.hpp"
#include "soc/syscon_struct.h"
#include "soc/syscon_reg.h"

AdcDMASignal::AdcDMASignal() {}

void AdcDMASignal::init() {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_12);
    adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_DB_12);

    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
        .sample_rate = static_cast<uint32_t>(Constant::SAMPLE_RATE * 2), // fs = 320 kHz stereo (640 kHz aggregate)
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT, // Use stereo to read both channels without duplicating/discarding slots
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 2, 0)
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
#else
        .communication_format = I2S_COMM_FORMAT_I2S_MSB,
#endif
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = Constant::I2S_DMA_BUF_COUNT,
        .dma_buf_len = Constant::I2S_DMA_BUF_LEN,
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0
    };

    esp_err_t err = i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    if (err != ESP_OK) {
        Serial.printf("Lỗi cài đặt driver I2S: %d\n", err);
    }

    err = i2s_set_adc_mode(ADC_UNIT_1, ADC1_CHANNEL_4);
    if (err != ESP_OK) {
        Serial.printf("Lỗi cấu hình kênh ADC cho I2S: %d\n", err);
    }

    err = i2s_adc_enable(I2S_NUM_0);
    if (err != ESP_OK) {
        Serial.printf("Lỗi kích hoạt I2S ADC: %d\n", err);
    }

    Serial.println("Cấu hình ADC I2S DMA hoàn tất.");
}

void IRAM_ATTR AdcDMASignal::start() {
    i2s_start(I2S_NUM_0);
    i2s_adc_enable(I2S_NUM_0);

    // Configure the ADC I2S pattern table directly via registers
    // Entry format: [Bits 7:4 - Channel] [Bits 3:2 - Bit Width (3 = 12bit)] [Bits 1:0 - Attenuation (3 = 11dB/12dB)]
    uint8_t entry0 = (ADC1_CHANNEL_4 << 4) | (3 << 2) | 3; // Channel 4, 12-bit, 11dB/12dB attenuation
    uint8_t entry1 = (ADC1_CHANNEL_5 << 4) | (3 << 2) | 3; // Channel 5, 12-bit, 11dB/12dB attenuation

    // Set pattern length (2 entries, value is length - 1)
    SYSCON.saradc_ctrl.sar1_patt_len = 1;
    // Pack both entries into the first pattern table register (MSB first)
    SYSCON.saradc_sar1_patt_tab[0] = (entry0 << 24) | (entry1 << 16) | 0xFFFF;

    SYSCON.saradc_ctrl.sar_clk_div = 4; // Giảm tỉ số chia xuống 4 để tăng tốc độ ADC lên 320 kHz tổng hợp
    SYSCON.saradc_fsm.sample_cycle = 9; // Giữ chu kỳ lấy mẫu bằng 9 để tụ nạp đầy đủ điện áp, tránh gọt đỉnh
}

void AdcDMASignal::stop() {
    i2s_adc_disable(I2S_NUM_0);
    i2s_stop(I2S_NUM_0);
}

esp_err_t AdcDMASignal::read(void* dest, size_t size, size_t* bytes_read, TickType_t ticks_to_wait) {
    return i2s_read(I2S_NUM_0, dest, size, bytes_read, ticks_to_wait);
}
