#include "test/profile.hpp"
#include "test/runner.hpp"
#include "utils/common.h"
#include "drivers/gpio.h"
#include "drivers/crc.h"
#include "drivers/adc.h"
#include "drivers/i2s.h"
#include "utils/tick.h"
#include "utils/log.h"

#include <array>


extern "C" {
int main() {
    // Run all the hardware driver tests
    test::runner();

    // Run the profile tests
    // profile::all();

    /*
    // Run some minor tests
    // I2S peripheral
    i2sx_clk_enable(I2S1, true);
    i2s_master_init(I2S1, nullptr);
    i2s_master_dma_init(I2S1, DMA_PRIORITY_VERY_HIGH);
    i2s_master_transmit(I2S1, nullptr, 0, nullptr, nullptr);

    // ADC peripheral
    adc_clk_configure(ADC_CLK_PRESCALER_4);
    adc_enable_nvic_irq(true);
    adc_power_on_temp_sensor(true);

    constexpr adc_config_t adc_cfg = {
        .alignment       = ADC_RIGHT_ALIGN,
        .resolution      = ADC_RES_12_BITS,
        .sampling_cycles = ADC_SAMPLE_15_CYCLES,
    };
    ASSERT(adcx_clk_enable(ADC1, true) == HAL_OK);
    ASSERT(adc_configure(ADC1, &adc_cfg) == HAL_OK);

    // Configure the analog watchdog
    constexpr adc_analog_wdg_config_t awdg_config = {
        .min_adc_value             = 800,
        .max_adc_value             = 3072,
        .monitor_regular_channels  = true,
        .monitor_injected_channels = true,
        .on_thresholds_violated =
            [](void*) {
                constexpr const char* TAG = "ADC_WDG";
                LOGE(TAG, "Voltage on an ADC channel past the valid range");
                LOGI(TAG, "Value of ADC1 regular group data register: %lu", ADC1->DR);
                LOGI(TAG, "Value of ADC1 first injected group data register: %lu", ADC1->JDR1);
                LOGI(TAG, "Value of ADC1 second injected group data register: %lu", ADC1->JDR2);
                LOGI(TAG, "Value of ADC1 third injected group data register: %lu", ADC1->JDR3);
                LOGI(TAG, "Value of ADC1 fourth injected group data register: %lu", ADC1->JDR4);
            },
        .arg = nullptr,
    };
    ASSERT(adc_analog_wdg_start(ADC1, &awdg_config) == HAL_OK);

    // Configure PA3 as analog since it is ADC channel 3
    ASSERT(gpiox_clk_enable(GPIOA, true) == HAL_OK);
    gpio_set_analog(GPIOA, GPIO_PIN_3);

    // Get the raw ADC data
    uint16_t raw_channel_3 = 0, raw_vbat = 0, raw_vref_int = 0;
    ASSERT(adc_get_v_bat(ADC1, &raw_vbat) == HAL_OK);
    ASSERT(adc_get_v_ref_internal(ADC1, &raw_vref_int) == HAL_OK);
    ASSERT(adc_regular_group_get_oneshot(ADC1, ADC_CHANNEL_3, &raw_channel_3) == HAL_OK);

    // Get the converted voltages
    float vdda = 0, temp_celsius = 0, channel_3 = 0, vbat = 0, vref_int = 0;
    ASSERT(adc_get_vdda(ADC1, &vdda) == HAL_OK);
    ASSERT(adc_get_temp_celsius(ADC1, &temp_celsius) == HAL_OK);
    ASSERT(adc_get_value_right_aligned(ADC1, raw_vbat, ADC_RES_12_BITS, &vbat) == HAL_OK);
    ASSERT(adc_get_value_right_aligned(ADC1, raw_vref_int, ADC_RES_12_BITS, &vref_int) == HAL_OK);
    ASSERT(adc_get_value_right_aligned(ADC1, raw_channel_3, ADC_RES_12_BITS, &channel_3) == HAL_OK);

    LOGI("ADC", "VDDA: %.3fV", (double)vdda);
    LOGI("ADC", "V_bat: %.3fV", (double)vbat);
    LOGI("ADC", "PA3: %.3fV", (double)channel_3);
    LOGI("ADC", "V_ref_int: %.3fV", (double)vref_int);
    LOGI("ADC", "Temperature: %.3fC", (double)temp_celsius);


    // CRC32 peripheral
    crc_clk_enable(true);

    constexpr auto buffer = std::array{1UL, 2UL, 3UL, 4UL, 5UL, 6UL, 7UL, 8UL, 9UL};
    uint32_t       crc32  = 0;

    ASSERT(crc_get(buffer.data(), buffer.size(), &crc32) == HAL_OK);
    ASSERT(crc_get_dma(
               buffer.data(),
               buffer.size(),
               DMA_PRIORITY_VERY_HIGH,
               [](void*, hal_err_t err, uint32_t crc_32) {
                   ASSERT(err == HAL_OK);
                   LOGI("CRC32_DMA", "CRC32 checksum of data = %lu", crc_32);
                   LOGI("CRC32_DMA", "CRC32 checksum of data = 0x%X", (size_t)crc_32);
               },
               nullptr) == HAL_OK);

    LOGI("CRC32", "CRC32 checksum of data = %lu", crc32);
    LOGI("CRC32", "CRC32 checksum of data = 0x%X", (size_t)crc32);
    */

    // Halt once tests are finished since nothing else to do.
    LOGI("Main", "Done with all tests. Halting...");
    HALT();
}
}
