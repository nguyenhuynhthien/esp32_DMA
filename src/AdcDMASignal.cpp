#include "AdcDMASignal.hpp"
#include <soc/i2s_struct.h>
#include <soc/i2s_reg.h>

AdcDMASignal::AdcDMASignal() {}

void AdcDMASignal::init() {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_12);

    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
        .sample_rate = 160000, // fs = 160 kHz
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 2, 0)
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
#else
        .communication_format = I2S_COMM_FORMAT_I2S_MSB,
#endif
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len = 64,
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

    Serial.println("Cấu hình ADC I2S DMA hoàn tất.");
    Serial.printf("[I2S0 CLOCK] clkm_div_num: %d, clkm_div_b: %d, clkm_div_a: %d, clka_en: %d\n",
                  I2S0.clkm_conf.clkm_div_num,
                  I2S0.clkm_conf.clkm_div_b,
                  I2S0.clkm_conf.clkm_div_a,
                  I2S0.clkm_conf.clka_en);
}

void IRAM_ATTR AdcDMASignal::start() {
    // Reset RX FIFO và RX DMA phần cứng để tránh hoàn toàn lệch byte (byte swap)
    I2S0.conf.rx_fifo_reset = 1;
    I2S0.conf.rx_fifo_reset = 0;
    I2S0.lc_conf.in_rst = 1;
    I2S0.lc_conf.in_rst = 0;

    i2s_start(I2S_NUM_0);
    i2s_adc_enable(I2S_NUM_0);
}

void AdcDMASignal::stop() {
    i2s_adc_disable(I2S_NUM_0);
    i2s_stop(I2S_NUM_0);
}

esp_err_t AdcDMASignal::read(void* dest, size_t size, size_t* bytes_read, TickType_t ticks_to_wait) {
    return i2s_read(I2S_NUM_0, dest, size, bytes_read, ticks_to_wait);
}
