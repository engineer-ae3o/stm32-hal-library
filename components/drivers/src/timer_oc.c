#include "stm32f411xe.h"
#include "drivers/timer_internals.h"
#include "drivers/timer_ext.h"
#include "drivers/timer.h"
#include "drivers/gpio.h"
#include "utils/common.h"
#include "utils/err.h"

#include <stddef.h>


hal_err_t timer_oc_init(TIM_TypeDef* handle, const timer_oc_config_t* config) {
    if (handle == NULL || config == NULL || config->num_channels == 0) {
        return HAL_ERR_INVALID_ARG;
    }

    // TIM9-TIM11 only support upcounting edge aligned upcounting
    // TIM1-TIM5 all have 4 main PWM channels, TIM9 has 2 and TIM10-TIM11 have 1 each
    if (((handle == TIM9 || handle == TIM10 || handle == TIM11) && (config->count_mode != TIMER_OC_EDGE_LEFT_ALIGNED)) ||
        ((handle == TIM1 || handle == TIM2 || handle == TIM3 || handle == TIM4 || handle == TIM5) && (config->num_channels > MAX_TIM1_CHANNELS)) ||
        ((handle == TIM9) && (config->num_channels > MAX_TIM9_CHANNELS)) ||
        ((handle == TIM10 || handle == TIM11) && (config->num_channels > MAX_TIM10_CHANNELS))) {
        return HAL_ERR_INVALID_ARG;
    }

    // Refer to pwm_advanced_timer_init(...) (drivers/pwm.c) for the explanation for this
    if (is_timer_advanced(handle) && (handle->BDTR & TIM_BDTR_LOCK)) {
        return HAL_ERR_INVALID_STATE;
    }

    // Clear all residual state
    TRY(timer_oc_deinit(handle));

    // Set the output compare PWM mode characteristics
    uint32_t ccmr1 = handle->CCMR1;
    uint32_t ccmr2 = handle->CCMR2;
    uint32_t ccer  = handle->CCER;
    uint32_t dier  = handle->DIER;

    const bool enable_cc_irq = config->callback != NULL;

    for (size_t i = 0; i < config->num_channels; i++) {
        // Configure the channel in the CCMRx register
        switch (config->channels[i].channel) {
            case TIMER_OC_CHANNEL_1:
                ccmr1 |= (0b00U << TIM_CCMR1_CC1S_Pos) | (uint32_t)(config->buffer_compare_reload ? TIM_CCMR1_OC1PE : 0) |
                         (uint32_t)(config->mode << TIM_CCMR1_OC1M_Pos);
                ccer |= config->channels[i].output_polarity ? (TIM_CCER_CC1E | TIM_CCER_CC1P) : (TIM_CCER_CC1E);
                dier |= enable_cc_irq ? TIM_DIER_CC1IE : 0;
                break;
            case TIMER_OC_CHANNEL_2:
                ccmr1 |= (0b00U << TIM_CCMR1_CC2S_Pos) | (uint32_t)(config->buffer_compare_reload ? TIM_CCMR1_OC2PE : 0) |
                         (uint32_t)(config->mode << TIM_CCMR1_OC2M_Pos);
                ccer |= config->channels[i].output_polarity ? (TIM_CCER_CC2E | TIM_CCER_CC2P) : (TIM_CCER_CC2E);
                dier |= enable_cc_irq ? TIM_DIER_CC2IE : 0;
                break;
            case TIMER_OC_CHANNEL_3:
                ccmr2 |= (0b00U << TIM_CCMR2_CC3S_Pos) | (uint32_t)(config->buffer_compare_reload ? TIM_CCMR2_OC3PE : 0) |
                         (uint32_t)(config->mode << TIM_CCMR2_OC3M_Pos);
                ccer |= config->channels[i].output_polarity ? (TIM_CCER_CC3E | TIM_CCER_CC3P) : (TIM_CCER_CC3E);
                dier |= enable_cc_irq ? TIM_DIER_CC3IE : 0;
                break;
            case TIMER_OC_CHANNEL_4:
                ccmr2 |= (0b00U << TIM_CCMR2_CC4S_Pos) | (uint32_t)(config->buffer_compare_reload ? TIM_CCMR2_OC4PE : 0) |
                         (uint32_t)(config->mode << TIM_CCMR2_OC4M_Pos);
                ccer |= config->channels[i].output_polarity ? (TIM_CCER_CC4E | TIM_CCER_CC4P) : (TIM_CCER_CC4E);
                dier |= enable_cc_irq ? TIM_DIER_CC4IE : 0;
                break;
            default:
                return HAL_ERR_INVALID_ARG;
        }

        // Configure the physical GPIO pin for PWM alternate function on the main channel
        const board_pin_t gpio_pin = config->channels[i].gpio_pin;
        TRY(gpiox_clk_enable(gpio_pin.port, true));
        gpio_set_alternate_function(gpio_pin.port, gpio_pin.pin, gpio_pin.af);
        gpio_set_speed_mode(gpio_pin.port, gpio_pin.pin, GPIO_FULL_SPEED);
        gpio_set_output_type(gpio_pin.port, gpio_pin.pin, GPIO_PUSH_PULL);
    }

    // Final writeback
    handle->CCMR1 = ccmr1;
    handle->CCMR2 = ccmr2;
    handle->CCER  = ccer;
    handle->DIER  = dier;

    // Set the timer's counting mode, and enable auto-reload register buffering and interrupts only on update events
    handle->CR1 |= (config->count_mode | TIM_CR1_ARPE | TIM_CR1_URS);

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

    // Generate an update event after modifying the auto reload and prescaler registers
    handle->EGR = TIM_EGR_UG;
    handle->SR  = ~TIM_SR_UIF;

    // Enable the timer's output
    TRY(timer_oc_unfreeze_timer(handle));

    // Derive the compare level (doubles as max duty cycle in PWM mode) from the auto-reload register
    *max_compare_level = timer_get_max_compare_level(handle);

    return HAL_OK;
}

hal_err_t timer_oc_set_compare(TIM_TypeDef* handle, timer_oc_channel_t channel, uint32_t compare_level) {
    if (handle == NULL || compare_level > timer_get_max_compare_level(handle)) {
        return HAL_ERR_INVALID_ARG;
    }

    switch (channel) {
        case TIMER_OC_CHANNEL_1:
            handle->CCR1 = compare_level;
            break;
        case TIMER_OC_CHANNEL_2:
            handle->CCR2 = compare_level;
            break;
        case TIMER_OC_CHANNEL_3:
            handle->CCR3 = compare_level;
            break;
        case TIMER_OC_CHANNEL_4:
            handle->CCR4 = compare_level;
            break;
        default:
            return HAL_ERR_INVALID_ARG;
    }

    return HAL_OK;
}

hal_err_t timer_oc_pause_channel(TIM_TypeDef* handle, timer_oc_channel_t channel) {
    if (handle == NULL || channel > TIMER_OC_CHANNEL_4) {
        return HAL_ERR_INVALID_ARG;
    }
    handle->CCER &= ~(1UL << (channel * 4));
    return HAL_OK;
}

hal_err_t timer_oc_resume_channel(TIM_TypeDef* handle, timer_oc_channel_t channel) {
    if (handle == NULL || channel > TIMER_OC_CHANNEL_4) {
        return HAL_ERR_INVALID_ARG;
    }
    handle->CCER |= (1UL << (channel * 4));
    return HAL_OK;
}

hal_err_t timer_oc_freeze_timer(TIM_TypeDef* handle) {
    if (handle == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    // Freeze the counter and disable all the channels' output
    if (is_timer_advanced(handle)) {
        handle->BDTR &= ~TIM_BDTR_MOE;
    }
    handle->CR1 &= ~TIM_CR1_CEN;

    return HAL_OK;
}

hal_err_t timer_oc_unfreeze_timer(TIM_TypeDef* handle) {
    if (handle == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    // Unfreeze the counter and enable all the channels' output
    if (is_timer_advanced(handle)) {
        handle->BDTR |= TIM_BDTR_MOE;
    }
    handle->CR1 |= TIM_CR1_CEN;

    return HAL_OK;
}
