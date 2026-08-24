#ifndef ADC_H_
#define ADC_H_


#include "utils/err.h"
#ifdef __cplusplus
extern "C" {
#endif


#include "stm32f411xe.h"
#include "drivers/dma.h"

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>


// External channels
typedef enum : uint8_t {
    // The external channels
    ADC_CHANNEL_0,  // PA0
    ADC_CHANNEL_1,  // PA1
    ADC_CHANNEL_2,  // PA2
    ADC_CHANNEL_3,  // PA3
    ADC_CHANNEL_4,  // PA4
    ADC_CHANNEL_5,  // PA5
    ADC_CHANNEL_6,  // PA6
    ADC_CHANNEL_7,  // PA7
    ADC_CHANNEL_8,  // PB0
    ADC_CHANNEL_9,  // PB1
    ADC_CHANNEL_10, // PC0
    ADC_CHANNEL_11, // PC1
    ADC_CHANNEL_12, // PC2
    ADC_CHANNEL_13, // PC3
    ADC_CHANNEL_14, // PC4
    ADC_CHANNEL_15, // PC5

    // The internal channels
    // They should not be used with any of regular or injected group
    // API as they require extra steps for their measurements than what
    // is needed for the external channels
    ADC_CHANNEL_TEMP, // Temperature sensor
    ADC_CHANNEL_VREF, // V_refint
    ADC_CHANNEL_VBAT, // V_bat
} adc_channels_t;

typedef enum : uint8_t {
    ADC_RES_6_BITS  = 0b11, // ADC resolution of 6 bits
    ADC_RES_8_BITS  = 0b10, // ADC resolution of 8 bits
    ADC_RES_10_BITS = 0b01, // ADC resolution of 10 bits
    ADC_RES_12_BITS = 0b00, // ADC resolution of 12 bits
} adc_resolution_t;

typedef enum : uint8_t {
    ADC_CLK_PRESCALER_2 = 0b00, // ADCCLK = (APB2 / 2)
    ADC_CLK_PRESCALER_4 = 0b01, // ADCCLK = (APB2 / 4)
    ADC_CLK_PRESCALER_6 = 0b10, // ADCCLK = (APB2 / 6)
    ADC_CLK_PRESCALER_8 = 0b11, // ADCCLK = (APB2 / 8)
} adc_prescaler_t;

typedef enum : uint8_t {
    ADC_SAMPLE_3_CYCLES   = 0b000, // ADC sampling time is 3 cycles
    ADC_SAMPLE_15_CYCLES  = 0b001, // ADC sampling time is 15 cycles
    ADC_SAMPLE_28_CYCLES  = 0b010, // ADC sampling time is 28 cycles
    ADC_SAMPLE_56_CYCLES  = 0b011, // ADC sampling time is 56 cycles
    ADC_SAMPLE_84_CYCLES  = 0b100, // ADC sampling time is 84 cycles
    ADC_SAMPLE_112_CYCLES = 0b101, // ADC sampling time is 112 cycles
    ADC_SAMPLE_144_CYCLES = 0b110, // ADC sampling time is 144 cycles
    ADC_SAMPLE_480_CYCLES = 0b111, // ADC sampling time is 480 cycles
} adc_sample_cycles_t;

typedef struct {
    adc_resolution_t    resolution;
    adc_prescaler_t     clk_prescaler;
    adc_sample_cycles_t sampling_cycles;
} adc_clk_config_t;


// General control of the ADC
void      adc_clk_enable(ADC_TypeDef* handle, bool enable);
void      adc_power_on(ADC_TypeDef* handle, bool on);
void      adc_enable_nvic_irq(void);
void      adc_disable_nvic_irq(void);
hal_err_t adc_configure_analog_clk(ADC_TypeDef* handle, const adc_clk_config_t* config);


// Number of regular and injected channels supported by the ADC peripheral
#define MAX_REGULAR_CHANNELS (16)
#define MAX_INJECTED_CHANNELS (4)

typedef void (*adc_callback_t)(void* arg);

// For use with the external channels
// The regular group
hal_err_t adc_regular_group_get_oneshot(ADC_TypeDef* handle, adc_channels_t channel, uint16_t* data);
hal_err_t adc_regular_group_start_conv(ADC_TypeDef* handle, const adc_channels_t* channels, size_t size, dma_trans_done_cb_t cb, void* arg);

// The injected group
hal_err_t adc_injected_group_start_conv(ADC_TypeDef* handle, const adc_channels_t* channels, size_t size, adc_callback_t cb, void* arg);
hal_err_t adc_injected_group_get_result(ADC_TypeDef* handle, uint16_t* buffer, size_t size);


// For operation of the internal channels.
// Only oneshot and conversion with the regular group are supported.
hal_err_t adc_get_v_bat(ADC_TypeDef* handle, uint16_t* raw_v_bat);
hal_err_t adc_get_temperature(ADC_TypeDef* handle, uint16_t* raw_temp);
hal_err_t adc_get_v_ref_internal(ADC_TypeDef* handle, uint16_t* raw_v_ref_int);


// For control of the analog watchdog control
hal_err_t adc_analog_wdg_start(ADC_TypeDef* handle, uint16_t min, uint16_t max, bool regular, bool injected, adc_callback_t cb, void* arg);
hal_err_t adc_analog_wdg_stop(ADC_TypeDef* handle);


#ifdef __cplusplus
}
#endif


#endif // ADC_H_