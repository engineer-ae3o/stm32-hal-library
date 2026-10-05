#include "stm32f411xe.h"
#include "drivers/timer_internals.h"
#include "drivers/pwm_types.h"
#include "drivers/timer.h"
#include "drivers/gpio.h"
#include "utils/common.h"
#include "utils/board.h"
#include "drivers/pwm.h"
#include "utils/err.h"

#include <stdint.h>


// Helper
static hal_err_t config_pwm_pin(board_pin_t gpio) {
    TRY(gpiox_clk_enable(gpio.port, true));
    gpio_set_alternate_function(gpio.port, gpio.pin, gpio.af);
    gpio_set_speed_mode(gpio.port, gpio.pin, GPIO_FULL_SPEED);
    gpio_set_output_type(gpio.port, gpio.pin, GPIO_PUSH_PULL);
    return HAL_OK;
}


// Public API
hal_err_t pwm_advanced_timer_init(TIM_TypeDef* handle, const pwm_advanced_timer_config_t* config) {
    if (handle == NULL || config == NULL || config->num_channels == 0 || config->num_channels > MAX_TIM1_CHANNELS) {
        return HAL_ERR_INVALID_ARG;
    }

    if (!is_timer_advanced(handle)) {
        return HAL_ERR_NOT_SUPPORTED;
    }

    // Once write protection has been set the first time and LOCK bits have been written to,
    // this function becomes unuseable as it has to write to most of the registers, but they
    // would have become write only because of the write protection in place.
    if (handle->BDTR & TIM_BDTR_LOCK) {
        return HAL_ERR_INVALID_STATE;
    }

    // Configure the dead time
    // Get the timer's input frequency from its bus and calculate the target tick rate
    const uint32_t timer_freq_hz = timer_get_frequency_hz(handle);
    const uint32_t target        = (uint32_t)ceil_div_u64((uint64_t)config->dead_time_ns * timer_freq_hz, (1 << config->clk_div) * 1000000000ULL);

    // Get the actual value for the dead time generator
    uint8_t dtg = 0;
    if (target <= 127U) {
        // Range 1: step = 1 tick, direct encoding
        dtg = (uint8_t)target;
    } else if (target <= 254U) {
        // Range 2: step = 2 ticks, DTG[5:0] = offset from 64
        const uint32_t field = ceil_div_u32(target, 2) - 64;
        dtg                  = (uint8_t)(0x80U | field);
    } else if (target <= 504U) {
        // Range 3: step = 8 ticks, DTG[4:0] = offset from 32
        const uint32_t field = ceil_div_u32(target, 8) - 32;
        dtg                  = (uint8_t)(0xC0U | field);
    } else if (target <= 1008U) {
        // Range 4: step = 16 ticks, DTG[4:0] = offset from 32
        const uint32_t field = ceil_div_u32(target, 16) - 32;
        dtg                  = (uint8_t)(0xE0U | field);
    } else {
        return HAL_ERR_NOT_SUPPORTED;
    }

    // Set the output compare PWM mode characteristics
    uint32_t ccmr1 = handle->CCMR1;
    uint32_t ccmr2 = handle->CCMR2;
    uint32_t ccer  = handle->CCER;
    uint32_t bdtr  = handle->BDTR;
    uint32_t cr2   = handle->CR2;

    // Configure the main and compementary PWM channels
    for (size_t i = 0; i < config->num_channels; i++) {
        const pwm_channel_t channel = config->channels[i].channel;
        switch (channel) {
            case PWM_CHANNEL_1:
                ccmr1 |= (0b00U << TIM_CCMR1_CC1S_Pos) | TIM_CCMR1_OC1PE | (uint32_t)(config->pwm_mode << TIM_CCMR1_OC1M_Pos);
                // Main channel
                ccer |= config->channels[i].invert_output ? (TIM_CCER_CC1E | TIM_CCER_CC1P) : (TIM_CCER_CC1E);
                cr2 |= config->channels[i].output_idle_state ? TIM_CR2_OIS1 : 0;
                // Complementary channel
                if (config->use_complementary_channels) {
                    ccer |= config->complementary_channels[channel].invert_output ? (TIM_CCER_CC1NE | TIM_CCER_CC1NP) : (TIM_CCER_CC1NE);
                    cr2 |= config->complementary_channels[channel].output_idle_state ? TIM_CR2_OIS1N : 0;
                }
                break;
            case PWM_CHANNEL_2:
                ccmr1 |= (0b00U << TIM_CCMR1_CC2S_Pos) | TIM_CCMR1_OC2PE | (uint32_t)(config->pwm_mode << TIM_CCMR1_OC2M_Pos);
                // Main channel
                ccer |= config->channels[i].invert_output ? (TIM_CCER_CC2E | TIM_CCER_CC2P) : (TIM_CCER_CC2E);
                cr2 |= config->channels[i].output_idle_state ? TIM_CR2_OIS2 : 0;
                // Complementary channel
                if (config->use_complementary_channels) {
                    ccer |= config->complementary_channels[channel].invert_output ? (TIM_CCER_CC2NE | TIM_CCER_CC2NP) : (TIM_CCER_CC2NE);
                    cr2 |= config->complementary_channels[channel].output_idle_state ? TIM_CR2_OIS2N : 0;
                }
                break;
            case PWM_CHANNEL_3:
                ccmr2 |= (0b00U << TIM_CCMR2_CC3S_Pos) | TIM_CCMR2_OC3PE | (uint32_t)(config->pwm_mode << TIM_CCMR2_OC3M_Pos);
                // Main channel
                ccer |= config->channels[i].invert_output ? (TIM_CCER_CC3E | TIM_CCER_CC3P) : (TIM_CCER_CC3E);
                cr2 |= config->channels[i].output_idle_state ? TIM_CR2_OIS3 : 0;
                // Complementary channel
                if (config->use_complementary_channels) {
                    ccer |= config->complementary_channels[channel].invert_output ? (TIM_CCER_CC3NE | TIM_CCER_CC3NP) : (TIM_CCER_CC3NE);
                    cr2 |= config->complementary_channels[channel].output_idle_state ? TIM_CR2_OIS3N : 0;
                }
                break;
            case PWM_CHANNEL_4:
                ccmr2 |= (0b00U << TIM_CCMR2_CC4S_Pos) | TIM_CCMR2_OC4PE | (uint32_t)(config->pwm_mode << TIM_CCMR2_OC4M_Pos);
                // Main channel
                ccer |= config->channels[i].invert_output ? (TIM_CCER_CC4E | TIM_CCER_CC4P) : (TIM_CCER_CC4E);
                cr2 |= config->channels[i].output_idle_state ? TIM_CR2_OIS4 : 0;
                // PWM channel 3 has no corresponding complementary channel
                break;
            default:
                return HAL_ERR_INVALID_ARG;
        }

        // Configure the physical GPIO pin for PWM alternate function on the main channel
        TRY(config_pwm_pin(config->channels[i].gpio_pin));

        if (config->use_complementary_channels && (channel != PWM_CHANNEL_4)) {
            // Configure the physical GPIO pin for PWM alternate function on the complementary channel
            TRY(config_pwm_pin(config->complementary_channels[channel].gpio_pin));
        }
    }

    // Configure the run and idle off-state selection of the channels and the write protection level
    bdtr |= (config->ossi ? TIM_BDTR_OSSI : 0) | (config->ossr ? TIM_BDTR_OSSR : 0) | (uint32_t)(config->wp_level << TIM_BDTR_LOCK_Pos) |
            (uint32_t)(dtg << TIM_BDTR_DTG_Pos);

    // Configure the break input
    if (config->break_input.use_break_input) {
        // Enable the break input, set the polarity and auto re-arm status
        bdtr |= TIM_BDTR_BKE | (config->break_input.active_low ? 0 : TIM_BDTR_BKP) | (config->break_input.auto_rearm ? TIM_BDTR_AOE : 0);

        // Enable interrupts on a break event and register the break event callback
        handle->DIER |= TIM_DIER_BIE;
        TRY(timer_register_callback(handle, config->break_input.callback, config->break_input.user, BREAK_EVENT));

        // Configure the physical break input GPIO pin
        const board_pin_t brk_gpio = config->break_input.gpio_pin;
        TRY(gpiox_clk_enable(brk_gpio.port, true));
        gpio_set_alternate_function(brk_gpio.port, brk_gpio.pin, brk_gpio.af);
        gpio_set_speed_mode(brk_gpio.port, brk_gpio.pin, GPIO_FULL_SPEED);
        if (config->break_input.active_low) {
            gpio_enable_pullups(brk_gpio.port, brk_gpio.pin, true);
        } else {
            gpio_enable_pulldowns(brk_gpio.port, brk_gpio.pin, true);
        }
    }

    // Final writeback
    handle->CCMR1 = ccmr1;
    handle->CCMR2 = ccmr2;
    handle->CCER  = ccer;
    handle->CR2   = cr2;

    // Set the timer's counting mode, and enable auto-reload register buffering,
    // interrupts only on update events and set the repition counter.
    handle->CR1 |= (config->pwm_count_mode | (uint32_t)(config->clk_div << TIM_CR1_CKD_Pos) | TIM_CR1_ARPE | TIM_CR1_URS);
    handle->RCR = config->repetition_cnt;

    // Configre the BDTR last, since the write protection could lock us out from modifying any of the other registers
    handle->BDTR = bdtr;

    return HAL_OK;
}

hal_err_t pwm_gp_timer_init(TIM_TypeDef* handle, const pwm_gp_timer_config_t* config) {
    if (handle == NULL || config == NULL || config->num_channels == 0) {
        return HAL_ERR_INVALID_ARG;
    }

    // TIM9-TIM11 only support upcounting edge aligned upcounting
    // TIM1-TIM5 all have 4 main PWM channels, TIM9 has 2 and TIM10-TIM11 have 1 each
    if (((handle == TIM9 || handle == TIM10 || handle == TIM11) && (config->pwm_count_mode != PWM_EDGE_LEFT_ALIGNED)) ||
        ((handle == TIM1 || handle == TIM2 || handle == TIM3 || handle == TIM4 || handle == TIM5) && (config->num_channels > MAX_TIM1_CHANNELS)) ||
        ((handle == TIM9) && (config->num_channels > MAX_TIM9_CHANNELS)) ||
        ((handle == TIM10 || handle == TIM11) && (config->num_channels > MAX_TIM10_CHANNELS))) {
        return HAL_ERR_INVALID_ARG;
    }

    // Refer to pwm_advanced_timer_init(...) for the explanation for this
    if (is_timer_advanced(handle) && (handle->BDTR & TIM_BDTR_LOCK)) {
        return HAL_ERR_INVALID_STATE;
    }

    // Set the output compare PWM mode characteristics
    uint32_t ccmr1 = handle->CCMR1;
    uint32_t ccmr2 = handle->CCMR2;
    uint32_t ccer  = handle->CCER;

    for (size_t i = 0; i < config->num_channels; i++) {
        // Configure the channel in the CCMRx register
        switch (config->channels[i].channel) {
            case PWM_CHANNEL_1:
                ccmr1 |= (0b00U << TIM_CCMR1_CC1S_Pos) | TIM_CCMR1_OC1PE | (uint32_t)(config->pwm_mode << TIM_CCMR1_OC1M_Pos);
                ccer |= config->channels[i].invert_output ? (TIM_CCER_CC1E | TIM_CCER_CC1P) : (TIM_CCER_CC1E);
                break;
            case PWM_CHANNEL_2:
                ccmr1 |= (0b00U << TIM_CCMR1_CC2S_Pos) | TIM_CCMR1_OC2PE | (uint32_t)(config->pwm_mode << TIM_CCMR1_OC2M_Pos);
                ccer |= config->channels[i].invert_output ? (TIM_CCER_CC2E | TIM_CCER_CC2P) : (TIM_CCER_CC2E);
                break;
            case PWM_CHANNEL_3:
                ccmr2 |= (0b00U << TIM_CCMR2_CC3S_Pos) | TIM_CCMR2_OC3PE | (uint32_t)(config->pwm_mode << TIM_CCMR2_OC3M_Pos);
                ccer |= config->channels[i].invert_output ? (TIM_CCER_CC3E | TIM_CCER_CC3P) : (TIM_CCER_CC3E);
                break;
            case PWM_CHANNEL_4:
                ccmr2 |= (0b00U << TIM_CCMR2_CC4S_Pos) | TIM_CCMR2_OC4PE | (uint32_t)(config->pwm_mode << TIM_CCMR2_OC4M_Pos);
                ccer |= config->channels[i].invert_output ? (TIM_CCER_CC4E | TIM_CCER_CC4P) : (TIM_CCER_CC4E);
                break;
            default:
                return HAL_ERR_INVALID_ARG;
        }

        // Configure the physical GPIO pin for PWM alternate function on the main channel
        TRY(config_pwm_pin(config->channels[i].gpio_pin));
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
    return timer_deinit(handle);
}

hal_err_t pwm_start(TIM_TypeDef* handle, uint32_t frequency_hz, uint32_t* max_duty_cycle) {
    if (handle == NULL || frequency_hz == 0 || max_duty_cycle == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    const uint32_t timer_freq_hz = timer_get_frequency_hz(handle);
    if (frequency_hz > timer_freq_hz) {
        return HAL_ERR_NOT_SUPPORTED;
    }

    // Get the denominator as it has different meanings depending on whether its center or edge aligned PWM
    const uint32_t total_ticks_per_period = round_div_u32(timer_freq_hz, frequency_hz);

    // Get the maximum width of the auto-reload register since it varies per timer
    const uint32_t max_arr        = is_timer_32_bits(handle) ? UINT32_MAX : UINT16_MAX;
    const uint64_t max_arr_plus_1 = (uint64_t)max_arr + 1;

    // Compute suitable auto-reload and prescaler values
    uint64_t psc_plus_1 = 0;
    uint64_t arr_plus_1 = 0;

    // Ceiling division is used to get the minimum PSC value possible
    // Then a plain rounding integer division is used to find the leftover
    if (handle->CR1 & TIM_CR1_CMS) {
        // Center aligned PWM
        // The denominator is equal to 2 * (PSC + 1) * ARR, so we divide by (2 * ARR), where ARR is max_arr
        psc_plus_1 = ceil_div_u64(total_ticks_per_period, 2ULL * max_arr);
        // Then divide by 2 * (PSC + 1), where (PSC + 1) is the just gotten psc_plus_1
        arr_plus_1 = round_div_u64(total_ticks_per_period, 2ULL * psc_plus_1) + 1;
    } else {
        // Edge aligned PWM
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
    TRY(pwm_freeze_timer(handle));

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
    TRY(pwm_unfreeze_timer(handle));

    // Derive the maximum duty cycle from the auto-reload register
    *max_duty_cycle = timer_get_max_compare_level(handle);

    return HAL_OK;
}

hal_err_t pwm_set_duty_cycle(TIM_TypeDef* handle, pwm_channel_t channel, uint32_t duty_cycle) {
    if (handle == NULL || duty_cycle > timer_get_max_compare_level(handle)) {
        return HAL_ERR_INVALID_ARG;
    }

    switch (channel) {
        case PWM_CHANNEL_1:
            handle->CCR1 = duty_cycle;
            break;
        case PWM_CHANNEL_2:
            handle->CCR2 = duty_cycle;
            break;
        case PWM_CHANNEL_3:
            handle->CCR3 = duty_cycle;
            break;
        case PWM_CHANNEL_4:
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

    // Freeze the counter and disable the timer's main output
    if (is_timer_advanced(handle)) {
        handle->BDTR &= ~TIM_BDTR_MOE;
    }
    handle->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E);
    handle->CR1 &= ~TIM_CR1_CEN;

    return HAL_OK;
}

hal_err_t pwm_unfreeze_timer(TIM_TypeDef* handle) {
    if (handle == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    // Unfreeze the counter and enable the timer's main output
    if (is_timer_advanced(handle)) {
        handle->BDTR |= TIM_BDTR_MOE;
    }
    handle->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E);
    handle->CR1 |= TIM_CR1_CEN;

    return HAL_OK;
}

hal_err_t pwm_pause_channel(TIM_TypeDef* handle, pwm_channel_t channel) {
    if (handle == NULL || channel > PWM_CHANNEL_4) {
        return HAL_ERR_INVALID_ARG;
    }

    // Disable the channel's output
    switch (channel) {
        case PWM_CHANNEL_1:
            handle->CCER &= ~(is_timer_advanced(handle) ? (TIM_CCER_CC1E | TIM_CCER_CC1NE) : TIM_CCER_CC1E);
            break;
        case PWM_CHANNEL_2:
            handle->CCER &= ~(is_timer_advanced(handle) ? (TIM_CCER_CC2E | TIM_CCER_CC2NE) : TIM_CCER_CC2E);
            break;
        case PWM_CHANNEL_3:
            handle->CCER &= ~(is_timer_advanced(handle) ? (TIM_CCER_CC3E | TIM_CCER_CC3NE) : TIM_CCER_CC3E);
            break;
        case PWM_CHANNEL_4:
            handle->CCER &= ~TIM_CCER_CC4E;
            break;
        default:
            return HAL_ERR_INVALID_ARG;
    }

    return HAL_OK;
}

hal_err_t pwm_resume_channel(TIM_TypeDef* handle, pwm_channel_t channel) {
    if (handle == NULL || channel > PWM_CHANNEL_4) {
        return HAL_ERR_INVALID_ARG;
    }

    // Enable the channel's output
    switch (channel) {
        case PWM_CHANNEL_1:
            handle->CCER |= (is_timer_advanced(handle) ? (TIM_CCER_CC1E | TIM_CCER_CC1NE) : TIM_CCER_CC1E);
            break;
        case PWM_CHANNEL_2:
            handle->CCER |= (is_timer_advanced(handle) ? (TIM_CCER_CC2E | TIM_CCER_CC2NE) : TIM_CCER_CC2E);
            break;
        case PWM_CHANNEL_3:
            handle->CCER |= (is_timer_advanced(handle) ? (TIM_CCER_CC3E | TIM_CCER_CC3NE) : TIM_CCER_CC3E);
            break;
        case PWM_CHANNEL_4:
            handle->CCER |= TIM_CCER_CC4E;
            break;
        default:
            return HAL_ERR_INVALID_ARG;
    }

    return HAL_OK;
}

// Internal helper
uint32_t timer_get_max_compare_level(TIM_TypeDef* handle) {
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
        if (gnu_unlikely(handle->ARR == (is_timer_32_bits(handle) ? UINT32_MAX : UINT16_MAX))) {
            return handle->ARR;
        } else {
            return handle->ARR + 1;
        }
    }
}
