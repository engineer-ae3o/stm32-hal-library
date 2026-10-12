#include "stm32f411xe.h"
#include "drivers/src/timer_internals.h"
#include "drivers/timer_pcnt_types.h"
#include "drivers/timer_pcnt.h"
#include "drivers/timer.h"
#include "utils/common.h"
#include "drivers/gpio.h"
#include "utils/board.h"
#include "utils/err.h"

#include <stdint.h>


// Public API
hal_err_t pcnt_init(TIM_TypeDef* handle, const pcnt_config_t* config) {
    if (handle == NULL || config == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    if (is_timer_extra_lite(handle) || (config->mode == PCNT_MODE_2 && (handle == TIM5 || handle == TIM9))) {
        return HAL_ERR_NOT_SUPPORTED;
    }

    // Refer to pwm_advanced_timer_init(...) (drivers/pwm.c) for the explanation for this
    if (is_timer_advanced(handle) && (handle->BDTR & TIM_BDTR_LOCK)) {
        return HAL_ERR_INVALID_STATE;
    }

    // Calculate the digital filter code
    uint32_t filter_code = 0;
    TRY(timer_filter_ns_to_ic_code(handle, config->clk_div, config->filter_ns, &filter_code));

    // Clear all residual state
    TRY(pcnt_deinit(handle));

    switch (config->mode) {
        case PCNT_MODE_1: {
            // Map the channel as input and apply the input capture digital filter code
            handle->CCMR1 |= (0b01U << config->settings.mode_1.channel) | (filter_code << (config->settings.mode_1.channel + 4));

            // Set the polarity
            if (config->edge == PCNT_FALLING_EDGE) {
                // In slave external clock mode, the timer counts only on rising edges by default
                // To get counting on the falling edge, we have to invert the channel's polarity
                handle->CCER |= (config->settings.mode_1.channel == PCNT_CHANNEL_1) ? TIM_CCER_CC1P : TIM_CCER_CC2P;
            }

            // Select the trigger source and enable the slave external clock mode
            const uint32_t trigger_selection = (config->settings.mode_1.channel == PCNT_CHANNEL_1) ? 0b101U : 0b110U;
            handle->SMCR |= (trigger_selection << TIM_SMCR_TS_Pos) | (0b111U << TIM_SMCR_SMS_Pos);

            break;
        }
        case PCNT_MODE_2: {
            // Apply the digital filter, external signal prescaler, set the polarity and enable the use of the external clock
            handle->SMCR |= (filter_code << TIM_SMCR_ETF_Pos) | (uint32_t)(config->settings.mode_2.prescaler << TIM_SMCR_ETPS_Pos) |
                            ((config->edge == PCNT_FALLING_EDGE) ? TIM_SMCR_ETP : 0) | TIM_SMCR_ECE;

            break;
        }
        default:
            return HAL_ERR_INVALID_ARG;
    }

    // Apply the clock divisor
    handle->CR1 |= (uint32_t)(config->clk_div << TIM_CR1_CKD_Pos);

    // Since the counter should be free running, use the maximum reload value
    handle->ARR = is_timer_32_bits(handle) ? UINT32_MAX : UINT16_MAX;

    if (config->callback.cb) {
        TRY(timer_register_callback(handle, config->callback, TIMER_IRQ_UPDATE_EVENT));
        // Enable the update event callback and URS so only update events trigger an interrupt
        handle->DIER |= TIM_DIER_UIE;
        handle->CR1 |= TIM_CR1_URS;
    }

    // Generate an update event so the timer's internal registers start afresh, and clear the update status flag
    handle->EGR = TIM_EGR_UG;
    handle->SR  = ~TIM_SR_UIF;

    // Configure the input pulse pin
    const board_pin_t gpio = config->pulse_gpio;
    TRY(gpiox_clk_enable(gpio.port, true));
    gpio_set_alternate_function(gpio.port, gpio.pin, gpio.af);
    gpio_set_speed_mode(gpio.port, gpio.pin, GPIO_FULL_SPEED);

    if (config->pull == TIMER_USE_PULLUP) {
        gpio_enable_pullup(gpio.port, gpio.pin, true);
    } else if (config->pull == TIMER_USE_PULLDOWN) {
        gpio_enable_pulldown(gpio.port, gpio.pin, true);
    } else {
        gpio_enable_pullup(gpio.port, gpio.pin, false);
    }

    return HAL_OK;
}

hal_err_t qdec_init(TIM_TypeDef* handle, const qdec_config_t* config) {
    if (handle == NULL || config == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    if (is_timer_lite(handle)) {
        return HAL_ERR_NOT_SUPPORTED;
    }

    // Refer to pwm_advanced_timer_init(...) (drivers/pwm.c) for the explanation for this
    if (is_timer_advanced(handle) && (handle->BDTR & TIM_BDTR_LOCK)) {
        return HAL_ERR_INVALID_STATE;
    }

    // Calculate the digital filter code
    uint32_t ic_filter_code = 0;
    TRY(timer_filter_ns_to_ic_code(handle, config->clk_div, config->filter_ns, &ic_filter_code));

    // Clear all residual state
    TRY(pcnt_deinit(handle));

    // Set the timer clock divider
    handle->CR1 |= (uint32_t)(config->clk_div << TIM_CR1_CKD_Pos);

    // Set timer channels 1 and 2 to input mode and set the digital filters
    handle->CCMR1 |= ((0b01U << TIM_CCMR1_CC1S_Pos) | (ic_filter_code << TIM_CCMR1_IC1F_Pos)) |
                     ((0b01U << TIM_CCMR1_CC2S_Pos) | (ic_filter_code << TIM_CCMR1_IC2F_Pos));

    // Set the rotational polarity
    handle->CCER |= config->invert_direction ? (TIM_CCER_CC1P | TIM_CCER_CC2P) : 0;

    // Apply the encoder mode
    handle->SMCR |= config->mode;

    // Since the counter should be free running, the maximum reload value is used
    handle->ARR = is_timer_32_bits(handle) ? UINT32_MAX : UINT16_MAX;

    if (config->callback.cb) {
        TRY(timer_register_callback(handle, config->callback, TIMER_IRQ_UPDATE_EVENT));
        // Enable the update event callback and URS so only update events trigger an interrupt
        handle->DIER |= TIM_DIER_UIE;
        handle->CR1 |= TIM_CR1_URS;
    }

    // Generate an update event so the timer's internal registers start afresh, and clear the update status flag
    handle->EGR = TIM_EGR_UG;
    handle->SR  = ~TIM_SR_UIF;

    // Configure the physical gpio channels
    const board_pin_t channel_a = config->channel_a;
    TRY(gpiox_clk_enable(channel_a.port, true));
    gpio_set_alternate_function(channel_a.port, channel_a.pin, channel_a.af);
    gpio_set_speed_mode(channel_a.port, channel_a.pin, GPIO_FULL_SPEED);

    const board_pin_t channel_b = config->channel_b;
    TRY(gpiox_clk_enable(channel_b.port, true));
    gpio_set_alternate_function(channel_b.port, channel_b.pin, channel_b.af);
    gpio_set_speed_mode(channel_b.port, channel_b.pin, GPIO_FULL_SPEED);

    if (config->pull == TIMER_USE_PULLUP) {
        gpio_enable_pullup(channel_a.port, channel_a.pin, true);
        gpio_enable_pullup(channel_b.port, channel_b.pin, true);
    } else if (config->pull == TIMER_USE_PULLDOWN) {
        gpio_enable_pulldown(channel_a.port, channel_a.pin, true);
        gpio_enable_pulldown(channel_b.port, channel_b.pin, true);
    } else {
        // The false parameter to gpio_enable_pullup and gpio_enable_pulldown does the same thing: disables all pull resistors
        gpio_enable_pullup(channel_a.port, channel_a.pin, false);
        gpio_enable_pullup(channel_b.port, channel_b.pin, false);
    }

    return HAL_OK;
}

hal_err_t pcnt_deinit(TIM_TypeDef* handle) {
    return timer_deinit(handle);
}

hal_err_t pcnt_start(TIM_TypeDef* handle) {
    return timer_start_counter(handle);
}

hal_err_t pcnt_stop(TIM_TypeDef* handle) {
    return timer_stop_counter(handle);
}

hal_err_t pcnt_set_count(TIM_TypeDef* handle, uint32_t count) {
    if (handle == NULL || count > (is_timer_32_bits(handle) ? UINT32_MAX : UINT16_MAX)) {
        return HAL_ERR_INVALID_ARG;
    }
    handle->CNT = count;
    return HAL_OK;
}

hal_err_t pcnt_get_count(TIM_TypeDef* handle, uint32_t* count) {
    if (handle == NULL || count == NULL) {
        return HAL_ERR_INVALID_ARG;
    }
    *count = handle->CNT;
    return HAL_OK;
}
