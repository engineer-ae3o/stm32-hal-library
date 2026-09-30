#include "stm32f411xe.h"
#include "drivers/timer_internals.h"
#include "drivers/timer.h"
#include "utils/common.h"
#include "drivers/pwm.h"
#include "utils/err.h"
#include <stdint.h>


hal_err_t pwm_advanced_timers_init(TIM_TypeDef* handle, const pwm_advanced_timer_config_t* config) {
    if (handle == NULL || config == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    // There's only one advanced timer on this hardware, that is TIM1
    if (handle != TIM1) {
        return HAL_ERR_NOT_SUPPORTED;
    }

    // Clear all remnant state before proceeding
    TRY(pwm_deinit(handle));

    return HAL_OK;
}

hal_err_t pwm_other_timers_init(TIM_TypeDef* handle, const pwm_other_timer_config_t* config) {
    if (handle == NULL || config == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    // Clear all remnant state before proceeding
    TRY(pwm_deinit(handle));

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

    uint32_t timer_freq_hz = 0;
    TRY(timer_get_frequency_hz(handle, &timer_freq_hz));
    if (frequency_hz > timer_freq_hz) {
        return HAL_ERR_NOT_SUPPORTED;
    }

    // Get the denominator as it has different meanings depending on whether its center or edge aligned PWM
    const uint32_t total_ticks_per_period = ((2 * timer_freq_hz) + frequency_hz) / (2 * frequency_hz);

    // Get the maximum width of the auto-reload registers
    const uint64_t max_arr_plus_1 = (uint64_t)(is_timer_32_bits(handle) ? UINT32_MAX : UINT16_MAX) + 1;

    // Compute suitable auto-reload and prescaler values
    // Ceiling division is used to get the minimum PSC value possible
    const uint32_t psc_plus_1 = (uint32_t)((total_ticks_per_period + max_arr_plus_1 - 1) / max_arr_plus_1);
    if (psc_plus_1 > (UINT16_MAX + 1)) {
        return HAL_ERR_NOT_SUPPORTED; // Frequency too low for timer clock
    }

    // The leftover after (PSC + 1) has been factored out depends on the PWM mode
    // Then a plain rounding integer division is used to find the leftover
    uint64_t arr_plus_1 = 0;

    const bool is_center_aligned_pwm = handle->CR1 & TIM_CR1_CMS;
    if (is_center_aligned_pwm) {
        // The denominator is equal to 2 * (PSC + 1) * ARR
        arr_plus_1 = ((total_ticks_per_period + psc_plus_1) / (psc_plus_1 * 2)) + 1;
    } else {
        // The denominator is equal to (PSC + 1) * (ARR * 1)
        arr_plus_1 = ((2ULL * total_ticks_per_period) + psc_plus_1) / (2ULL * psc_plus_1);
    }

    // Set the actual reload and prescaler values
    handle->ARR = (uint32_t)(arr_plus_1 - 1);
    handle->PSC = (uint32_t)(psc_plus_1 - 1);

    // Generate an update event after modifying the auto reload and prescaler registers
    handle->EGR |= TIM_EGR_UG;
    handle->SR &= ~TIM_SR_UIF;

    if (is_center_aligned_pwm) {
        *max_duty_cycle = handle->ARR;
    } else {
        // Doing it like this introduces an off by one at the edge, but it only happens
        // when the timer is 32 bits and and ARR happens to hold its maximum value. I do
        // it like this because using uint64_t would be slower for a case that will almost
        // never occur in any real usage. The maximum duty cycle is ARR + 1, hence the off
        // by one at the boundary of 32 bits. The off by one is a non factor regardless.
        if (gnu_unlikely(handle->ARR == UINT32_MAX)) {
            *max_duty_cycle = handle->ARR;
        } else {
            *max_duty_cycle = handle->ARR + 1;
        }
    }

    // Start the PWM output
    handle->CR1 |= TIM_CR1_CEN;

    return HAL_OK;
}

hal_err_t pwm_set_duty_cycle(TIM_TypeDef* handle, pwm_channel_t channel, uint32_t duty_cycle) {
    if (handle == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    // Get the maximum duty cycle to bounds check the passed in duty cycle
    uint32_t max_duty_cycle = 0;
    if (handle->CR1 & TIM_CR1_CMS) {
        max_duty_cycle = handle->ARR;
    } else {
        if (gnu_unlikely(handle->ARR == UINT32_MAX)) {
            max_duty_cycle = handle->ARR;
        } else {
            max_duty_cycle = handle->ARR + 1;
        }
    }
    if (duty_cycle > max_duty_cycle) {
        return HAL_ERR_INVALID_ARG;
    }

    switch (channel) {
        case PWM_CHANNEL_0:
            handle->CCR1 = duty_cycle;
            break;
        case PWM_CHANNEL_1:
            handle->CCR2 = duty_cycle;
            break;
        case PWM_CHANNEL_2:
            handle->CCR3 = duty_cycle;
            break;
        case PWM_CHANNEL_3:
            handle->CCR4 = duty_cycle;
            break;
        default:
            return HAL_ERR_INVALID_ARG;
    }

    return HAL_OK;
}

hal_err_t pwm_pause(TIM_TypeDef* handle) {
    if (handle == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    if (!(handle->CR1 & TIM_CR1_CEN)) {
        return HAL_ERR_INVALID_STATE;
    }

    // Pause the PWM output by disabling the timer's counter
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

    // Resume the PWM output by re-enabling the timer's counter
    handle->CR1 |= TIM_CR1_CEN;
    return HAL_OK;
}
