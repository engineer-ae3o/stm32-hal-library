#ifndef PWM_H_
#define PWM_H_


#ifdef __cplusplus
extern "C" {
#endif


#include "stm32f411xe.h"
#include "drivers/pwm_types.h"
#include "utils/err.h"

#include <stdint.h>


// To enable or disable the timer's clock, make use of timx_clk_enable(...) in timer.h.
// NOTE: These two modes are mutually exclusive with the other


// PWM Output mode
hal_err_t pwm_advanced_timer_init(TIM_TypeDef* handle, const pwm_advanced_timer_config_t* config);
hal_err_t pwm_gp_timer_init(TIM_TypeDef* handle, const pwm_gp_timer_config_t* config);
hal_err_t pwm_deinit(TIM_TypeDef* handle);

hal_err_t pwm_start(TIM_TypeDef* handle, uint32_t frequency_hz, uint32_t* max_duty_cycle);
hal_err_t pwm_set_duty_cycle(TIM_TypeDef* handle, timer_channel_t channel, uint32_t duty_cycle);

hal_err_t pwm_freeze_timer(TIM_TypeDef* handle);
hal_err_t pwm_unfreeze_timer(TIM_TypeDef* handle);

hal_err_t pwm_pause_channel(TIM_TypeDef* handle, timer_channel_t channel);
hal_err_t pwm_resume_channel(TIM_TypeDef* handle, timer_channel_t channel);

hal_err_t pwm_pause_complementary_channel(TIM_TypeDef* handle, timer_channel_t channel);
hal_err_t pwm_resume_complementary_channel(TIM_TypeDef* handle, timer_channel_t channel);


// PWM Input mode
hal_err_t pwm_input_init(TIM_TypeDef* handle, const pwm_input_config_t* config);
hal_err_t pwm_input_deinit(TIM_TypeDef* handle);

hal_err_t pwm_input_start(TIM_TypeDef* handle);
hal_err_t pwm_input_stop(TIM_TypeDef* handle);

hal_err_t pwm_input_get_data(TIM_TypeDef* handle, pwm_input_data_t* data);
hal_err_t pwm_input_get_timer_frequency(TIM_TypeDef* handle, uint32_t* frequency_hz);

// NOTE: It's only safe to call pwm_input_get_data(...) to get the captured data after
// the capture event interrupt for channel 1 and 2 have both fired for a given cycle.
// Anything else would result in the reading of garbage or stale data.


#ifdef __cplusplus
}
#endif


#endif // PWM_H_