#include "drivers/pwm_types.h"
#include "stm32f411xe.h"
#include "drivers/timer_internals.h"
#include "drivers/timer.h"
#include "drivers/gpio.h"
#include "utils/common.h"
#include "drivers/pwm.h"
#include "utils/err.h"

#include <stdint.h>


// Helper
[[__gnu__::__always_inline__]] static inline uint32_t get_max_duty_cycle(TIM_TypeDef* handle) {
    if (handle->CR1 & TIM_CR1_CMS) {
        // Center aligned PWM
        return handle->ARR;
    } else {
        // Edge aligned PWM
        // Doing it like this introduces an off by one error at the edge, but it only happens
        // when the timer is 32 bits and and ARR happens to hold its maximum value. I do
        // it like this because using uint64_t would be much slower for a case that will almost
        // never occur in any real usage. The maximum duty cycle is ARR + 1, hence the off
        // by one at the boundary of 32 bits. The off by one error is a non factor regardless.
        if (gnu_unlikely(handle->ARR == UINT32_MAX)) {
            return handle->ARR;
        } else {
            return handle->ARR + 1;
        }
    }
}


// Public API
hal_err_t pwm_advanced_timer_init(TIM_TypeDef* handle, const pwm_advanced_timer_config_t* config) {
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

hal_err_t pwm_timer_init(TIM_TypeDef* handle, const pwm_timer_config_t* config) {
    if (handle == NULL || config == NULL || config->num_channels == 0) {
        return HAL_ERR_INVALID_ARG;
    }

    // TIM9-TIM11 only support upcounting edge aligned upcounting
    // TIM1-TIM5 all have 4 main PWM channels, TIM9 has 2 and TIM10-TIM11 have 1 each
    if (((handle == TIM9 || handle == TIM10 || handle == TIM11) && (config->pwm_count_mode != PWM_EDGE_ALIGNED_UPCOUNTING)) ||
        ((handle == TIM1 || handle == TIM2 || handle == TIM3 || handle == TIM4 || handle == TIM5) && (config->num_channels > MAX_TIM1_CHANNELS)) ||
        ((handle == TIM9) && (config->num_channels > MAX_TIM9_CHANNELS)) ||
        ((handle == TIM10 || handle == TIM11) && (config->num_channels > MAX_TIM10_CHANNELS))) {
        return HAL_ERR_NOT_SUPPORTED;
    }

    // Clear all residual state before proceeding
    TRY(pwm_deinit(handle));

    // Set the output compare PWM mode characteristics
    uint32_t ccmr1 = handle->CCMR1;
    uint32_t ccmr2 = handle->CCMR2;
    uint32_t ccer  = handle->CCER;

    for (size_t i = 0; i < config->num_channels; i++) {
        // Configure the channel in the CCMRx register
        switch (config->channels[i].channel) {
            case PWM_CHANNEL_0:
                ccmr1 |= (0b00U << TIM_CCMR1_CC1S_Pos) | TIM_CCMR1_OC1PE | (uint32_t)(config->pwm_mode << TIM_CCMR1_OC1M_Pos);
                ccer |= TIM_CCER_CC1E;
                if (config->channels[i].invert_output) {
                    ccer |= TIM_CCER_CC1P;
                }
                break;
            case PWM_CHANNEL_1:
                ccmr1 |= (0b00U << TIM_CCMR1_CC2S_Pos) | TIM_CCMR1_OC2PE | (uint32_t)(config->pwm_mode << TIM_CCMR1_OC2M_Pos);
                ccer |= TIM_CCER_CC2E;
                if (config->channels[i].invert_output) {
                    ccer |= TIM_CCER_CC2P;
                }
                break;
            case PWM_CHANNEL_2:
                ccmr2 |= (0b00U << TIM_CCMR2_CC3S_Pos) | TIM_CCMR2_OC3PE | (uint32_t)(config->pwm_mode << TIM_CCMR2_OC3M_Pos);
                ccer |= TIM_CCER_CC3E;
                if (config->channels[i].invert_output) {
                    ccer |= TIM_CCER_CC3P;
                }
                break;
            case PWM_CHANNEL_3:
                ccmr2 |= (0b00U << TIM_CCMR2_CC4S_Pos) | TIM_CCMR2_OC4PE | (uint32_t)(config->pwm_mode << TIM_CCMR2_OC4M_Pos);
                ccer |= TIM_CCER_CC4E;
                if (config->channels[i].invert_output) {
                    ccer |= TIM_CCER_CC4P;
                }
                break;
            default:
                return HAL_ERR_INVALID_ARG;
        }

        // Configure the physical GPIO pin for PWM alternate function
        const board_pin_t gpio = config->channels[i].gpio_pin;
        TRY(gpiox_clk_enable(gpio.port, true));
        gpio_set_alternate_function(gpio.port, gpio.pin, gpio.af);
        gpio_set_speed_mode(gpio.port, gpio.pin, GPIO_FULL_SPEED);
        gpio_set_output_type(gpio.port, gpio.pin, GPIO_PUSH_PULL);
    }

    // Final writeback
    handle->CCMR1 = ccmr1;
    handle->CCMR2 = ccmr2;
    handle->CCER  = ccer;

    // Set the timer's counting mode, and enable auto-reload register buffering and interrupts only on update events
    handle->CR1 |= (config->pwm_count_mode | TIM_CR1_ARPE | TIM_CR1_URS);

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

    // This requires that the timer be explicitly frozen/disabled
    if (handle->CR1 & TIM_CR1_CEN) {
        return HAL_ERR_INVALID_STATE;
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
    handle->EGR = TIM_EGR_UG;
    handle->SR &= ~TIM_SR_UIF;

    // Set all channels' duty cycles to 0 since starting afresh with a new frequency
    handle->CCR1 = 0;
    handle->CCR2 = 0;
    handle->CCR3 = 0;
    handle->CCR4 = 0;

    // Derive the maximum duty cycle from the auto-reload register
    *max_duty_cycle = get_max_duty_cycle(handle);

    // Enable the timer's output
    TRY(pwm_unfreeze_timer(handle));

    return HAL_OK;
}

hal_err_t pwm_set_duty_cycle(TIM_TypeDef* handle, pwm_channel_t channel, uint32_t duty_cycle) {
    if (handle == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    if (duty_cycle > get_max_duty_cycle(handle)) {
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

hal_err_t pwm_freeze_timer(TIM_TypeDef* handle) {
    if (handle == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    if (!(handle->CR1 & TIM_CR1_CEN)) {
        return HAL_ERR_INVALID_STATE;
    }

    // Freeze the counter and disable TIM1's main output
    if (handle == TIM1) {
        handle->BDTR &= ~TIM_BDTR_MOE;
    }
    handle->CR1 &= ~TIM_CR1_CEN;

    return HAL_OK;
}

hal_err_t pwm_unfreeze_timer(TIM_TypeDef* handle) {
    if (handle == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    if (handle->CR1 & TIM_CR1_CEN) {
        return HAL_ERR_INVALID_STATE;
    }

    // Unfreeze the counter and enable TIM1's main output
    if (handle == TIM1) {
        handle->BDTR |= TIM_BDTR_MOE;
    }
    handle->CR1 |= TIM_CR1_CEN;

    return HAL_OK;
}

hal_err_t pwm_pause_channel(TIM_TypeDef* handle, pwm_channel_t channel) {
    if (handle == NULL || channel > PWM_CHANNEL_3) {
        return HAL_ERR_INVALID_ARG;
    }

    // Disable the channel's output
    handle->CCER &= ~(1UL << (channel * 4));

    return HAL_OK;
}

hal_err_t pwm_resume_channel(TIM_TypeDef* handle, pwm_channel_t channel) {
    if (handle == NULL || channel > PWM_CHANNEL_3) {
        return HAL_ERR_INVALID_ARG;
    }

    // Enable the channel's output
    handle->CCER |= (1UL << (channel * 4));

    return HAL_OK;
}
