#ifndef ADC_H_
#define ADC_H_


#ifdef __cplusplus
extern "C" {
#endif


#include "stm32f411xe.h"
#include "drivers/adc_types.h"
#include "utils/err.h"


// Configure a specific ADC peripheral
hal_err_t adcx_clk_enable(ADC_TypeDef* handle, bool enable);
hal_err_t adc_configure(ADC_TypeDef* handle, const adc_config_t* config);
hal_err_t adc_power_on(ADC_TypeDef* handle, bool on);


// General control of all the ADCs. These functions affect all the ADC peripheral instances
void adc_clk_configure(adc_prescaler_t clk_prescaler);
void adc_enable_nvic_irq(bool enable);
void adc_power_on_temp_sensor(bool on);


// For use with the external channels
// The regular group with oneshot polling mode
hal_err_t adc_regular_group_get_oneshot(ADC_TypeDef* handle, adc_channels_t channel, uint16_t* raw_data);

// The regular group with continuous DMA mode
hal_err_t adc_regular_group_cont_start_conv(ADC_TypeDef* handle, const adc_continuous_config_t* config);
hal_err_t adc_regular_group_cont_end_conv(ADC_TypeDef* handle);

// The injected group with interrupts
hal_err_t adc_injected_group_start_conv(ADC_TypeDef* handle, const adc_injected_group_config_t* config);
hal_err_t adc_injected_group_get_result(ADC_TypeDef* handle, uint16_t* raw_data_buffer, size_t buffer_size);


// For operation of the internal channels.
// Only oneshot and conversion with the regular group are supported.
// These only return the raw ADC values. Pair with adc_get_value_right_aligned(...)
// (if you use ADC_RIGHT_ALIGN) to get the actual voltages read by thr ADC.
hal_err_t adc_get_v_bat(ADC_TypeDef* handle, uint16_t* raw_data);
hal_err_t adc_get_temperature(ADC_TypeDef* handle, uint16_t* raw_data);
hal_err_t adc_get_v_ref_internal(ADC_TypeDef* handle, uint16_t* raw_data);


// Get the actual voltage readings from the ADC
hal_err_t adc_get_vdda(ADC_TypeDef* handle, float* vdda);
hal_err_t adc_get_temp_celsius(ADC_TypeDef* handle, float* temp_celsius);
hal_err_t adc_get_value_right_aligned(ADC_TypeDef* handle, uint16_t raw_data, adc_resolution_t resolution, float* voltage);


// For control of the analog watchdog control
hal_err_t adc_analog_wdg_start(ADC_TypeDef* handle, const adc_analog_wdg_config_t* config);
hal_err_t adc_analog_wdg_stop(ADC_TypeDef* handle);


#ifdef __cplusplus
}
#endif


#endif // ADC_H_