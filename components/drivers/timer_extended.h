#ifndef TIMER_EXT_H_
#define TIMER_EXT_H_


#ifdef __cplusplus
extern "C" {
#endif


#include "stm32f411xe.h"
#include "drivers/timer_types.h"
#include "utils/err.h"

#include <stdint.h>


// The extended timer module
// Implements extra functionality from the timer(s)


// Output Compare
hal_err_t output_compare_init(TIM_TypeDef* handle, const output_compare_config_t* config);
hal_err_t output_compare_deinit(TIM_TypeDef* handle);

hal_err_t output_compare_start(TIM_TypeDef* handle, uint32_t period_hz, uint32_t* max_compare_level);
hal_err_t output_compare_set_compare(TIM_TypeDef* handle, timer_channel_t channel, uint32_t compare_level);

hal_err_t output_compare_pause_channel(TIM_TypeDef* handle, timer_channel_t channel);
hal_err_t output_compare_resume_channel(TIM_TypeDef* handle, timer_channel_t channel);

hal_err_t output_compare_freeze_timer(TIM_TypeDef* handle);
hal_err_t output_compare_unfreeze_timer(TIM_TypeDef* handle);


// Input Capture


// Pulse Counting
hal_err_t pcnt_init(TIM_TypeDef* handle, const pcnt_config_t* config);
hal_err_t pcnt_deinit(TIM_TypeDef* handle);

hal_err_t pcnt_start(TIM_TypeDef* handle);
hal_err_t pcnt_stop(TIM_TypeDef* handle);

hal_err_t pcnt_set_count(TIM_TypeDef* handle, uint32_t count);
hal_err_t pcnt_get_count(TIM_TypeDef* handle, uint32_t* count);


// Quadrature decoder (encoder mode)
hal_err_t encoder_init(TIM_TypeDef* handle, const encoder_config_t* config);
hal_err_t encoder_deinit(TIM_TypeDef* handle);

hal_err_t encoder_start(TIM_TypeDef* handle);
hal_err_t encoder_stop(TIM_TypeDef* handle);


#ifdef __cplusplus
}
#endif


#endif // TIMER_EXT_H_