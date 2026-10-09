#include "drivers/timer_types.h"
#include "stm32f411xe.h"
#include "drivers/timer_internals.h"
#include "drivers/timer_extended.h"
#include "drivers/timer.h"
#include "utils/common.h"
#include "drivers/gpio.h"
#include "utils/board.h"
#include "utils/err.h"

#include <stdint.h>


hal_err_t pcnt_init(TIM_TypeDef* handle, const pcnt_config_t* config) {
    if (handle == NULL || config == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    if ((handle == TIM10 || handle == TIM11) || (handle == TIM9 && config->mode != PCNT_MODE_1)) {
        return HAL_ERR_NOT_SUPPORTED;
    }

    switch (config->mode) {
        case PCNT_MODE_1: {
            const auto settings = config->settings.mode_1;
            if (config->edge == PCNT_RISING_FALLING_EDGE) {
                return HAL_ERR_NOT_SUPPORTED;
            }

            // Calculate the digital filter code
            uint32_t filter_code = 0;
            TRY(timer_filter_ns_to_ic_code(handle, config->clk_div, settings.filter_ns, &filter_code));

            // Clear all residual state
            TRY(pcnt_deinit(handle));

            // Map the channel as input and apply the input capture digital filter code
            handle->CCMR1 |= (0b01U << settings.channel) | (filter_code << (settings.channel + 4));

            // Set the polarity
            if (config->edge == PCNT_FALLING_EDGE) {
                // In slave external clock mode, the timer counts only on rising edges by default
                // To get counting on the falling edge, we have to invert the channel's polarity
                handle->CCER |= (settings.channel == PCNT_CHANNEL_1) ? TIM_CCER_CC1P : TIM_CCER_CC2P;
            }

            // Select the trigger source and enable slave external clock mode
            const uint32_t trigger_selection = (settings.channel == PCNT_CHANNEL_1) ? 0b101U : 0b110U;
            handle->SMCR |= (trigger_selection << TIM_SMCR_TS_Pos) | (0b111U << TIM_SMCR_SMS_Pos);

            break;
        }
        case PCNT_MODE_2: {
            //const auto settings = config->settings.mode_2;

            // Clear all residual state
            TRY(pcnt_deinit(handle));
            break;
        }
        default:
            return HAL_ERR_INVALID_ARG;
    }

    // Apply the clock divisor
    handle->CR1 |= (uint32_t)(config->clk_div << TIM_CR1_CKD_Pos);

    // Since the counter should be free running, use the maximum reload value
    handle->ARR = is_timer_32_bits(handle) ? UINT32_MAX : UINT16_MAX;

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

hal_err_t pcnt_deinit(TIM_TypeDef* handle) {
    return timer_deinit(handle);
}

hal_err_t pcnt_start(TIM_TypeDef* handle) {
    if (handle == NULL) {
        return HAL_ERR_INVALID_ARG;
    }
    handle->CR1 |= TIM_CR1_CEN;
    return HAL_OK;
}

hal_err_t pcnt_stop(TIM_TypeDef* handle) {
    if (handle == NULL) {
        return HAL_ERR_INVALID_ARG;
    }
    handle->CR1 &= ~TIM_CR1_CEN;
    return HAL_OK;
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
