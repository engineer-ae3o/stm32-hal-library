#include "stm32f411xe.h"
#include "drivers/timer_internals.h"
#include "drivers/timer_ext.h"
#include "drivers/timer.h"
#include "utils/common.h"
#include "utils/err.h"

#include <stddef.h>


hal_err_t timer_oc_init(TIM_TypeDef* handle, const timer_oc_config_t* config) {
    if (handle == NULL || config == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    return HAL_OK;
}

hal_err_t timer_oc_deinit(TIM_TypeDef* handle) {
    return timer_deinit(handle);
}

hal_err_t timer_oc_start(TIM_TypeDef* handle, uint32_t tick_rate_hz, uint32_t* max_compare_level) {
    if (handle == NULL || tick_rate_hz == 0 || max_compare_level == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    const uint32_t timer_freq_hz = timer_get_frequency_hz(handle);
    if (tick_rate_hz > timer_freq_hz) {
        return HAL_ERR_NOT_SUPPORTED;
    }

    // Get the denominator as it has different meanings depending on whether its center or edge aligned Output Compare
    const uint32_t total_ticks_per_period = round_div_u32(timer_freq_hz, tick_rate_hz);

    // Get the maximum width of the auto-reload register since it varies per timer
    const uint32_t max_arr        = is_timer_32_bits(handle) ? UINT32_MAX : UINT16_MAX;
    const uint64_t max_arr_plus_1 = (uint64_t)max_arr + 1;

    // Compute suitable auto-reload and prescaler values
    uint64_t psc_plus_1 = 0;
    uint64_t arr_plus_1 = 0;

    // Ceiling division is used to get the minimum PSC value possible
    // Then a plain rounding integer division is used to find the leftover
    if (handle->CR1 & TIM_CR1_CMS) {
        // Center aligned Output Compare
        // The denominator is equal to 2 * (PSC + 1) * ARR, so we divide by (2 * ARR), where ARR is max_arr
        psc_plus_1 = ceil_div_u64(total_ticks_per_period, 2ULL * max_arr);
        // Then divide by 2 * (PSC + 1), where (PSC + 1) is the just gotten psc_plus_1
        arr_plus_1 = round_div_u64(total_ticks_per_period, 2ULL * psc_plus_1) + 1;
    } else {
        // Edge aligned Output Compare
        // The denominator is equal to (PSC + 1) * (ARR + 1), so we divide by (ARR + 1), where (ARR + 1) is max_arr_plus_1
        psc_plus_1 = ceil_div_u64(total_ticks_per_period, max_arr_plus_1);
        // Then divide by (PSC + 1), where (PSC + 1) is the just gotten psc_plus_1
        arr_plus_1 = round_div_u64(total_ticks_per_period, psc_plus_1);
    }

    // Bounds check the prescaler and reload values
    if ((psc_plus_1 > (UINT16_MAX + 1)) || (arr_plus_1 > max_arr_plus_1)) {
        return HAL_ERR_NOT_SUPPORTED; // Frequency too low for timer clock
    }

    // Freeze the timer's output before writing to any of its registers
    TRY(timer_oc_freeze_timer(handle));

    // Set the actual reload and prescaler values
    handle->ARR = (uint32_t)(arr_plus_1 - 1);
    handle->PSC = (uint32_t)(psc_plus_1 - 1);

    // Set all channels' duty cycles to 0 since starting afresh with a new frequency
    handle->CCR1 = 0;
    handle->CCR2 = 0;
    handle->CCR3 = 0;
    handle->CCR4 = 0;

    // Generate an update event after modifying the auto reload and prescaler registers
    handle->EGR = TIM_EGR_UG;
    handle->SR  = ~TIM_SR_UIF;

    // Enable the timer's output
    TRY(timer_oc_unfreeze_timer(handle));

    // Derive the compare level (doubles as max duty cycle in PWM mode) from the auto-reload register
    *max_compare_level = timer_get_max_duty_cycle(handle);

    return HAL_OK;
}

hal_err_t timer_oc_set_compare(TIM_TypeDef* handle, timer_oc_channel_t channel, uint32_t compare_level) {
    if (handle == NULL || compare_level > timer_get_max_duty_cycle(handle)) {
        return HAL_ERR_INVALID_ARG;
    }

    switch (channel) {
        case OC_CHANNEL_1:
            handle->CCR1 = compare_level;
            break;
        case OC_CHANNEL_2:
            handle->CCR2 = compare_level;
            break;
        case OC_CHANNEL_3:
            handle->CCR3 = compare_level;
            break;
        case OC_CHANNEL_4:
            handle->CCR4 = compare_level;
            break;
        default:
            return HAL_ERR_INVALID_ARG;
    }

    return HAL_OK;
}

hal_err_t timer_oc_pause_channel(TIM_TypeDef* handle, timer_oc_channel_t channel) {
    if (handle == NULL || channel > OC_CHANNEL_4) {
        return HAL_ERR_INVALID_ARG;
    }
    handle->CCER &= ~(1UL << (channel * 4));
    return HAL_OK;
}

hal_err_t timer_oc_resume_channel(TIM_TypeDef* handle, timer_oc_channel_t channel) {
    if (handle == NULL || channel > OC_CHANNEL_4) {
        return HAL_ERR_INVALID_ARG;
    }
    handle->CCER |= (1UL << (channel * 4));
    return HAL_OK;
}

hal_err_t timer_oc_freeze_timer(TIM_TypeDef* handle) {
    if (handle == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    // Freeze the counter and disable the timer's output
    handle->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E);
    handle->CR1 &= ~TIM_CR1_CEN;

    return HAL_OK;
}

hal_err_t timer_oc_unfreeze_timer(TIM_TypeDef* handle) {
    if (handle == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    // Unfreeze the counter and enable the timer's output
    handle->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E);
    handle->CR1 |= TIM_CR1_CEN;

    return HAL_OK;
}
