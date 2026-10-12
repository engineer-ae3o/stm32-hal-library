#include "stm32f411xe.h"
#include "drivers/src/timer_internals.h"
#include "drivers/timer_extended.h"
#include "drivers/timer_types.h"
#include "drivers/timer.h"
#include "drivers/gpio.h"
#include "utils/common.h"
#include "utils/err.h"

#include <stdint.h>
#include <stddef.h>
#include <string.h>


hal_err_t input_capture_init(TIM_TypeDef* handle, const input_capture_config_t* config) {
    if (handle == NULL || config == NULL || config->num_channels == 0 || config->num_channels > timer_get_num_channels(handle)) {
        return HAL_ERR_INVALID_ARG;
    }

    const bool timer_advanced   = is_timer_advanced(handle);
    const bool timer_extra_lite = is_timer_extra_lite(handle);
    const bool timer_lite       = is_timer_lite(handle);

    // Refer to pwm_advanced_timer_init(...) (drivers/pwm.c) for the explanation for this
    if (timer_advanced && (handle->BDTR & TIM_BDTR_LOCK)) {
        return HAL_ERR_INVALID_STATE;
    }

    // Clear all residual state
    TRY(input_capture_deinit(handle));

    // Set the output compare mode characteristics
    uint32_t ccmr1 = 0;
    uint32_t ccmr2 = 0;
    uint32_t ccer  = 0;
    uint32_t dier  = 0;

    const bool enable_cc_irq = (config->callback.cb != NULL);

    for (size_t i = 0; i < config->num_channels; i++) {
        const ic_channel_config_t channel = config->channels[i];

        // Calculate the digital filter code
        uint32_t filter_code = 0;
        TRY(timer_filter_ns_to_ic_code(handle, config->clk_div, channel.filter_ns, &filter_code));

        // Configure the channel in the CCMRx register
        switch (channel.channel) {
            case TIMER_CHANNEL_1: {
                ccmr1 |= (0b01U << TIM_CCMR1_CC1S_Pos) | (filter_code << TIM_CCMR1_IC1F_Pos) | (uint32_t)(channel.prescaler << TIM_CCMR1_IC1PSC_Pos);
                uint32_t polarity = 0;
                if (channel.edge == IC_EDGE_RISING_FALLING) {
                    polarity = TIM_CCER_CC1P | TIM_CCER_CC1NP;
                } else if (channel.edge == IC_EDGE_FALLING) {
                    polarity = TIM_CCER_CC1P;
                }
                ccer |= polarity | TIM_CCER_CC1E;
                dier |= enable_cc_irq ? TIM_DIER_CC1IE : 0;
                break;
            }
            case TIMER_CHANNEL_2: {
                if (timer_extra_lite) {
                    return HAL_ERR_INVALID_ARG;
                }
                ccmr1 |= (0b01U << TIM_CCMR1_CC2S_Pos) | (filter_code << TIM_CCMR1_IC2F_Pos) | (uint32_t)(channel.prescaler << TIM_CCMR1_IC2PSC_Pos);
                uint32_t polarity = 0;
                if (channel.edge == IC_EDGE_RISING_FALLING) {
                    polarity = TIM_CCER_CC2P | TIM_CCER_CC2NP;
                } else if (channel.edge == IC_EDGE_FALLING) {
                    polarity = TIM_CCER_CC2P;
                }
                ccer |= polarity | TIM_CCER_CC2E;
                dier |= enable_cc_irq ? TIM_DIER_CC2IE : 0;
                break;
            }
            case TIMER_CHANNEL_3: {
                if (timer_lite) {
                    return HAL_ERR_INVALID_ARG;
                }
                ccmr2 |= (0b01U << TIM_CCMR2_CC3S_Pos) | (filter_code << TIM_CCMR2_IC3F_Pos) | (uint32_t)(channel.prescaler << TIM_CCMR2_IC3PSC_Pos);
                uint32_t polarity = 0;
                if (channel.edge == IC_EDGE_RISING_FALLING) {
                    polarity = TIM_CCER_CC3P | TIM_CCER_CC3NP;
                } else if (channel.edge == IC_EDGE_FALLING) {
                    polarity = TIM_CCER_CC3P;
                }
                ccer |= polarity | TIM_CCER_CC3E;
                dier |= enable_cc_irq ? TIM_DIER_CC3IE : 0;
                break;
            }
            case TIMER_CHANNEL_4: {
                if (timer_lite) {
                    return HAL_ERR_INVALID_ARG;
                }
                ccmr2 |= (0b01U << TIM_CCMR2_CC4S_Pos) | (filter_code << TIM_CCMR2_IC4F_Pos) | (uint32_t)(channel.prescaler << TIM_CCMR2_IC4PSC_Pos);
                uint32_t polarity = 0;
                if (channel.edge == IC_EDGE_RISING_FALLING) {
                    polarity = TIM_CCER_CC4P | TIM_CCER_CC4NP;
                } else if (channel.edge == IC_EDGE_FALLING) {
                    polarity = TIM_CCER_CC4P;
                }
                ccer |= polarity | TIM_CCER_CC4E;
                dier |= enable_cc_irq ? TIM_DIER_CC4IE : 0;
                break;
            }
            default:
                return HAL_ERR_INVALID_ARG;
        }

        // Configure the input pulse pin
        const board_pin_t gpio = channel.gpio_pin;
        TRY(gpiox_clk_enable(gpio.port, true));
        gpio_set_alternate_function(gpio.port, gpio.pin, gpio.af);
        gpio_set_speed_mode(gpio.port, gpio.pin, GPIO_FULL_SPEED);

        if (channel.pull == TIMER_USE_PULLUP) {
            gpio_enable_pullup(gpio.port, gpio.pin, true);
        } else if (channel.pull == TIMER_USE_PULLDOWN) {
            gpio_enable_pulldown(gpio.port, gpio.pin, true);
        } else {
            gpio_enable_pullup(gpio.port, gpio.pin, false);
        }
    }

    // Final writeback
    handle->CCER |= ccer;
    handle->DIER |= dier;
    handle->CCMR1 |= ccmr1;

    if (!timer_lite) {
        // The lite timers do not have a CCMR2 register
        handle->CCMR2 |= ccmr2;
    }

    if (enable_cc_irq) {
        TRY(timer_register_callback(handle, config->callback, TIMER_IRQ_CAPTURE_COMPARE));
    }

    // Since the counter should be free running, the maximum reload value is used
    handle->ARR = is_timer_32_bits(handle) ? UINT32_MAX : UINT16_MAX;

    // Set the timer's counting mode, apply the clock divider, and enable auto-reload register buffering
    handle->CR1 |= (config->count_mode | (uint32_t)(config->clk_div << TIM_CR1_CKD_Pos) | TIM_CR1_ARPE);

    // Generate an update event so the timer's internal registers start afresh, and clear the update status flag
    handle->EGR = TIM_EGR_UG;
    handle->SR  = ~TIM_SR_UIF;

    return HAL_OK;
}

hal_err_t input_capture_deinit(TIM_TypeDef* handle) {
    return timer_deinit(handle);
}

hal_err_t input_capture_start(TIM_TypeDef* handle) {
    if (handle == NULL) {
        return HAL_ERR_INVALID_ARG;
    }
    handle->CR1 |= TIM_CR1_CEN;
    return HAL_OK;
}

hal_err_t input_capture_stop(TIM_TypeDef* handle) {
    if (handle == NULL) {
        return HAL_ERR_INVALID_ARG;
    }
    handle->CR1 &= ~TIM_CR1_CEN;
    return HAL_OK;
}

hal_err_t input_capture_get_sample(TIM_TypeDef* handle, timer_channel_t channel, uint32_t* result) {
    if (handle == NULL || result == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    hal_err_t error = HAL_OK;

    // Read the compare value and check for an ovecapture
    switch (channel) {
        case TIMER_CHANNEL_1:
            *result = handle->CCR1;
            if (handle->SR & TIM_SR_CC1OF) {
                handle->SR = ~(TIM_SR_CC1IF | TIM_SR_CC1OF);
                error      = HAL_ERR_TIM_OVER_CAPTURE;
            }
            break;
        case TIMER_CHANNEL_2:
            if (is_timer_extra_lite(handle)) {
                return HAL_ERR_NOT_SUPPORTED;
            }
            *result = handle->CCR2;
            if (handle->SR & TIM_SR_CC2OF) {
                handle->SR = ~(TIM_SR_CC2IF | TIM_SR_CC2OF);
                error      = HAL_ERR_TIM_OVER_CAPTURE;
            }
            break;
        case TIMER_CHANNEL_3:
            if (is_timer_lite(handle)) {
                return HAL_ERR_NOT_SUPPORTED;
            }
            *result = handle->CCR3;
            if (handle->SR & TIM_SR_CC3OF) {
                handle->SR = ~(TIM_SR_CC3IF | TIM_SR_CC3OF);
                error      = HAL_ERR_TIM_OVER_CAPTURE;
            }
            break;
        case TIMER_CHANNEL_4:
            if (is_timer_lite(handle)) {
                return HAL_ERR_NOT_SUPPORTED;
            }
            *result = handle->CCR4;
            if (handle->SR & TIM_SR_CC4OF) {
                handle->SR = ~(TIM_SR_CC4IF | TIM_SR_CC4OF);
                error      = HAL_ERR_TIM_OVER_CAPTURE;
            }
            break;
        default:
            return HAL_ERR_INVALID_ARG;
    }

    return error;
}

hal_err_t input_capture_get_timer_frequency(TIM_TypeDef* handle, uint32_t* frequency_hz) {
    if (handle == NULL || frequency_hz == NULL) {
        return HAL_ERR_INVALID_ARG;
    }
    *frequency_hz = timer_get_frequency_hz(handle);
    return HAL_OK;
}
