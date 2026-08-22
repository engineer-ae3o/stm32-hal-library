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


typedef enum : uint8_t {
    ADC_RESOLUTION_6_BITS = 0,
    ADC_RESOLUTION_8_BITS,
    ADC_RESOLUTION_10_BITS,
    ADC_RESOLUTION_12_BITS,
} adc_resolution_t;

// External channels
typedef enum : uint8_t {
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
} adc_ext_channel_t;

// Internal channels
typedef enum : uint8_t {
    ADC_CHANNEL_TEMP = 16, // Temperature sensor
    ADC_CHANNEL_VREF = 17, // V_refint
    ADC_CHANNEL_VBAT = 18, // V_bat
} adc_int_channel_t;


#define MAX_REGULAR_CHANNELS (16)
#define MAX_INJECTED_CHANNELS (4)

// General control of the ADC
void adc_clk_enable(bool enable);
void adc_power_on(bool on);
void adc_configure_analog_clk(uint8_t prescaler);
void adc_configure_sample_time(uint8_t cycles);
void adc_enable_nvic_irq(void);
void adc_disable_nvic_irq(void);

// For operation with the ADC regular mode
hal_err_t adc_regular_mode_get_oneshot(adc_ext_channel_t channel, uint16_t* data);
hal_err_t adc_regular_mode_start_conv(const adc_ext_channel_t* channels, size_t num_of_channels, dma_trans_done_cb_t cb, void* arg);
hal_err_t adc_regular_end_start_conv();

// For operation with the ADC injected mode
typedef void (*adc_injected_done_cb_t)(void* arg);
hal_err_t adc_injected_mode_start_conv(const adc_ext_channel_t* channels, size_t num_of_channels, adc_injected_done_cb_t cb, void* arg);
hal_err_t adc_injected_mode_get_result(uint16_t* buffer, size_t num_of_channels);

// For operation of the internal channels. Only oneshot and regular group are supported
hal_err_t get_v_bat(uint16_t* v_bat);
hal_err_t get_v_ref_int(uint16_t* v_ref);
hal_err_t get_temperature(uint16_t* temperature);

// Analog watchdog control
typedef void (*adc_awdg_isr_t)(void* arg);
hal_err_t adc_analog_wdg_start(uint16_t min_volt, uint16_t max_volt, bool monitor_regular, bool monitor_injected, adc_awdg_isr_t cb, void* arg);
void      adc_analog_wdg_stop(void);


#ifdef __cplusplus
}
#endif


#endif // ADC_H_