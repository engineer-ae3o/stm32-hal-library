#ifndef PWM_H_
#define PWM_H_


#ifdef __cplusplus
extern "C" {
#endif


#include "stm32f411xe.h"
#include "drivers/pwm_types.h"
#include "utils/err.h"

#include <stdint.h>


hal_err_t pwm_advanced_timer_init(TIM_TypeDef* handle, const pwm_advanced_timer_config_t* config);
hal_err_t pwm_timer_init(TIM_TypeDef* handle, const pwm_timer_config_t* config);

hal_err_t pwm_deinit(TIM_TypeDef* handle);

hal_err_t pwm_start(TIM_TypeDef* handle, uint32_t frequency_hz, uint32_t* max_duty_cycle);
hal_err_t pwm_set_duty_cycle(TIM_TypeDef* handle, pwm_channel_t channel, uint32_t duty_cycle);

hal_err_t pwm_pause(TIM_TypeDef* handle);
hal_err_t pwm_resume(TIM_TypeDef* handle);


#ifdef __cplusplus
}
#endif


#endif // PWM_H_