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
hal_err_t timer_oc_init(TIM_TypeDef* handle, const timer_oc_config_t* config);
hal_err_t timer_oc_deinit(TIM_TypeDef* handle);
hal_err_t timer_oc_start(TIM_TypeDef* handle, uint32_t tick_rate_hz, uint32_t* max_compare_level);
hal_err_t timer_oc_set_compare(TIM_TypeDef* handle, timer_oc_channel_t channel, uint32_t compare_level);
hal_err_t timer_oc_pause_channel(TIM_TypeDef* handle, timer_oc_channel_t channel);
hal_err_t timer_oc_resume_channel(TIM_TypeDef* handle, timer_oc_channel_t channel);
hal_err_t timer_oc_freeze_timer(TIM_TypeDef* handle);
hal_err_t timer_oc_unfreeze_timer(TIM_TypeDef* handle);


// Input Capture


// Pulse Counter


// Quadrature decoder (encoder mode)


#ifdef __cplusplus
}
#endif


#endif // TIMER_EXT_H_