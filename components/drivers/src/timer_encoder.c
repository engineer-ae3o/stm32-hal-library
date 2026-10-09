#include "stm32f411xe.h"
#include "drivers/timer_internals.h"
#include "drivers/timer_extended.h"
#include "drivers/timer_types.h"
#include "utils/common.h"
#include "drivers/gpio.h"
#include "utils/err.h"

#include <stdint.h>


hal_err_t encoder_init(TIM_TypeDef* handle, const encoder_config_t* config) {
    if (handle == NULL || config == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    if (is_timer_lite(handle)) {
        return HAL_ERR_NOT_SUPPORTED;
    }

    // Calculate the digital filter code
    uint32_t ic_filter_code = 0;
    TRY(timer_filter_ns_to_ic_code(handle, config->clk_div, config->filter_ns, &ic_filter_code));

    // Clear all residual state
    TRY(encoder_deinit(handle));

    // Set the timer clock divider
    handle->CR1 |= (uint32_t)(config->clk_div << TIM_CR1_CKD_Pos);

    // Set timer channels 1 and 2 to input mode and set the digital filters
    handle->CCMR1 |= ((0b01U << TIM_CCMR1_CC1S_Pos) | (ic_filter_code << TIM_CCMR1_IC1F_Pos)) |
                     ((0b01U << TIM_CCMR1_CC2S_Pos) | (ic_filter_code << TIM_CCMR1_IC2F_Pos));

    // Set the rotational polarity
    handle->CCER |= config->invert_direction ? (TIM_CCER_CC1P | TIM_CCER_CC2P) : 0;

    // Apply the encoder mode
    handle->SMCR |= config->mode;

    // Since the counter should be free running, use the maximum reload value
    handle->ARR = is_timer_32_bits(handle) ? UINT32_MAX : UINT16_MAX;

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

hal_err_t encoder_deinit(TIM_TypeDef* handle) {
    return pcnt_deinit(handle);
}

hal_err_t encoder_start(TIM_TypeDef* handle) {
    return pcnt_start(handle);
}

hal_err_t encoder_stop(TIM_TypeDef* handle) {
    return pcnt_stop(handle);
}

hal_err_t encoder_set_count(TIM_TypeDef* handle, uint32_t count) {
    return pcnt_set_count(handle, count);
}

hal_err_t encoder_get_count(TIM_TypeDef* handle, uint32_t* count) {
    return pcnt_get_count(handle, count);
}
