#include "stm32f411xe.h"
#include "drivers/timer_internals.h"
#include "drivers/timer.h"
#include "utils/common.h"
#include "drivers/pwm.h"
#include "utils/err.h"


hal_err_t pwm_advanced_timers_init(TIM_TypeDef* handle, const pwm_advanced_timers_config_t* config) {
    if (handle == NULL || config == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    return HAL_OK;
}

hal_err_t pwm_general_timers_init(TIM_TypeDef* handle, const pwm_general_timers_config_t* config) {
    if (handle == NULL || config == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    return HAL_OK;
}

hal_err_t pwm_lite_timers_init(TIM_TypeDef* handle, const pwm_lite_timers_config_t* config) {
    if (handle == NULL || config == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    return HAL_OK;
}

hal_err_t pwm_deinit(TIM_TypeDef* handle) {
    TRY(timer_deinit(handle));
    return HAL_OK;
}

hal_err_t pwm_start(TIM_TypeDef* handle, uint32_t frequency_hz, uint32_t* max_duty_cycle) {
    if (handle == NULL || frequency_hz == 0 || max_duty_cycle == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    return HAL_OK;
}

hal_err_t pwm_set_duty_cycle(TIM_TypeDef* handle, pwm_channel_t channel, uint32_t duty_cycle) {
    (void)handle;
    (void)channel;
    (void)duty_cycle;
    return HAL_OK;
}

hal_err_t pwm_pause(TIM_TypeDef* handle) {
    if (handle == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    if (!(handle->CR1 & TIM_CR1_CEN)) {
        return HAL_ERR_INVALID_STATE;
    }

    // Pause the timer by disabling its counter
    handle->CR1 &= ~TIM_CR1_CEN;
    return HAL_OK;
}

hal_err_t pwm_resume(TIM_TypeDef* handle) {
    if (handle == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    if (handle->CR1 & TIM_CR1_CEN) {
        return HAL_ERR_INVALID_STATE;
    }

    // Resume the timer by re-enabling its counter
    handle->CR1 |= TIM_CR1_CEN;
    return HAL_OK;
}
