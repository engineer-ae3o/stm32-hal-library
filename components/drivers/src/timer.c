#include "stm32f411xe.h"
#include "drivers/src/timer_internals.h"
#include "drivers/timer_types.h"
#include "drivers/timer.h"
#include "utils/common.h"
#include "utils/err.h"

#include <string.h>
#include <stdint.h>


// Helper
static inline hal_err_t timer_set_arr_and_psc(TIM_TypeDef* handle, uint32_t timeout_us) {
    // Set the (PSC + 1) * (ARR + 1) value from the timer's clock frequency and the timeout
    const uint64_t psc_times_arr = ((uint64_t)timer_get_frequency_hz(handle) * timeout_us) / 1'000'000U;

    // Get the maximum width of the prescaler and auto-reload registers
    const uint32_t max_psc_plus_1 = UINT16_MAX + 1;
    const uint64_t max_arr_plus_1 = (uint64_t)(is_timer_32_bits(handle) ? UINT32_MAX : UINT16_MAX) + 1;

    // Bounds check the arguments against the width of the timers' registers
    const uint64_t max_psc_times_arr = max_psc_plus_1 * max_arr_plus_1;
    if (psc_times_arr > max_psc_times_arr) {
        return HAL_ERR_NOT_SUPPORTED;
    }

    // Compute suitable auto-reload and prescaler values
    // Minimum PSC such that ARR can cover the remainder: ceiling division to find PSC
    // Round to nearest instead of floor to halve the worst case error to get the ARR
    const uint64_t psc_plus_1 = ceil_div_u64(psc_times_arr, max_arr_plus_1);
    const uint64_t arr_plus_1 = round_div_u64(psc_times_arr, psc_plus_1);

    // Set the actual prescaler and auto-reload values
    handle->ARR = (uint32_t)(arr_plus_1 - 1);
    handle->PSC = (uint32_t)(psc_plus_1 - 1);

    // Clear existing state
    handle->CNT  = 0;
    handle->CCR1 = 0;
    // TIM10 and TIM11 only have CCR1
    if (!is_timer_extra_lite(handle)) {
        handle->CCR2 = 0;
        // TIM9 only has CCR1 and CCR2
        if (handle != TIM9) {
            handle->CCR3 = 0;
            handle->CCR4 = 0;
        }
    }

    // Generate an update event and clear the update interrupt
    // flag since the ARR and PSC registers contain new values
    handle->EGR = TIM_EGR_UG;
    handle->SR  = ~TIM_SR_UIF;

    return HAL_OK;
}


// Public API
hal_err_t timx_clk_enable(TIM_TypeDef* handle, bool enable) {
    if (enable) {
        if (handle == TIM1) {
            RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
        } else if (handle == TIM2) {
            RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
        } else if (handle == TIM3) {
            RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
        } else if (handle == TIM4) {
            RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
        } else if (handle == TIM5) {
            RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
        } else if (handle == TIM9) {
            RCC->APB2ENR |= RCC_APB2ENR_TIM9EN;
        } else if (handle == TIM10) {
            RCC->APB2ENR |= RCC_APB2ENR_TIM10EN;
        } else if (handle == TIM11) {
            RCC->APB2ENR |= RCC_APB2ENR_TIM11EN;
        } else {
            return HAL_ERR_INVALID_ARG;
        }

    } else {
        if (handle == TIM1) {
            RCC->APB2ENR &= ~RCC_APB2ENR_TIM1EN;
        } else if (handle == TIM2) {
            RCC->APB1ENR &= ~RCC_APB1ENR_TIM2EN;
        } else if (handle == TIM3) {
            RCC->APB1ENR &= ~RCC_APB1ENR_TIM3EN;
        } else if (handle == TIM4) {
            RCC->APB1ENR &= ~RCC_APB1ENR_TIM4EN;
        } else if (handle == TIM5) {
            RCC->APB1ENR &= ~RCC_APB1ENR_TIM5EN;
        } else if (handle == TIM9) {
            RCC->APB2ENR &= ~RCC_APB2ENR_TIM9EN;
        } else if (handle == TIM10) {
            RCC->APB2ENR &= ~RCC_APB2ENR_TIM10EN;
        } else if (handle == TIM11) {
            RCC->APB2ENR &= ~RCC_APB2ENR_TIM11EN;
        } else {
            return HAL_ERR_INVALID_ARG;
        }
    }

    __DSB();
    return HAL_OK;
}

hal_err_t timer_init(TIM_TypeDef* handle, timer_count_dir_t direction, timer_cb_t callback) {
    if (handle == NULL || callback.cb == NULL || direction > TIMER_COUNTER_DIR_DOWN) {
        return HAL_ERR_INVALID_ARG;
    }

    if (is_timer_lite(handle) && direction != TIMER_COUNTER_DIR_UP) {
        return HAL_ERR_NOT_SUPPORTED;
    }

    // Refer to pwm_advanced_timer_init(...) (drivers/pwm.c) for the explanation for this
    if (is_timer_advanced(handle) && (handle->BDTR & TIM_BDTR_LOCK)) {
        return HAL_ERR_INVALID_STATE;
    }

    // Fail if the timer is active. Caller should explictly call
    // timer_deinit(...) or timer_pause(...) before calling this
    if (handle->CR1 & TIM_CR1_CEN) {
        return HAL_ERR_INVALID_STATE;
    }

    // Clear all residual state
    TRY(timer_deinit(handle));

    // Register the callback
    TRY(timer_register_callback(handle, callback, TIMER_IRQ_UPDATE_EVENT));

    // Enable the update event interrupt
    handle->DIER |= TIM_DIER_UIE;

    // Set the counting direction. Edge aligned (up or downcounting)
    // Enable the ARR preload, and set URS so only a UEV triggers an interrupt
    handle->CR1 |= (uint32_t)direction | TIM_CR1_ARPE | TIM_CR1_URS;

    return HAL_OK;
}

hal_err_t timer_deinit(TIM_TypeDef* handle) {
    if (handle == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    // Clear all the mapped bits of the timer's registers to 0
    handle->CR1 &= ~(TIM_CR1_CEN | TIM_CR1_UDIS | TIM_CR1_URS | TIM_CR1_OPM | TIM_CR1_DIR | TIM_CR1_CMS | TIM_CR1_ARPE | TIM_CR1_CKD);
    handle->DIER &=
        ~(TIM_DIER_UIE | TIM_DIER_CC1IE | TIM_DIER_CC2IE | TIM_DIER_CC3IE | TIM_DIER_CC4IE | TIM_DIER_COMIE | TIM_DIER_TIE | TIM_DIER_BIE |
          TIM_DIER_UDE | TIM_DIER_CC1DE | TIM_DIER_CC2DE | TIM_DIER_CC3DE | TIM_DIER_CC4DE | TIM_DIER_COMDE | TIM_DIER_TDE);
    handle->SR = ~(TIM_SR_UIF | TIM_SR_CC1IF | TIM_SR_CC2IF | TIM_SR_CC3IF | TIM_SR_CC4IF | TIM_SR_COMIF | TIM_SR_TIF | TIM_SR_BIF | TIM_SR_CC1OF |
                   TIM_SR_CC2OF | TIM_SR_CC3OF | TIM_SR_CC4OF);
    handle->CCER &=
        ~(TIM_CCER_CC1E | TIM_CCER_CC1P | TIM_CCER_CC1NE | TIM_CCER_CC1NP | TIM_CCER_CC2E | TIM_CCER_CC2P | TIM_CCER_CC2NP | TIM_CCER_CC2NE |
          TIM_CCER_CC3E | TIM_CCER_CC3P | TIM_CCER_CC3NE | TIM_CCER_CC3NP | TIM_CCER_CC4E | TIM_CCER_CC4P | TIM_CCER_CC4NP);
    handle->CCMR1 &= ~(TIM_CCMR1_CC1S | TIM_CCMR1_OC1FE | TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M | TIM_CCMR1_OC1CE | TIM_CCMR1_CC2S | TIM_CCMR1_OC2FE |
                       TIM_CCMR1_OC2PE | TIM_CCMR1_OC2M | TIM_CCMR1_OC2CE);
    handle->CNT  = 0;
    handle->PSC  = 0;
    handle->ARR  = 0;
    handle->CCR1 = 0;

    // TIM10 and TIM11 only have CCR1
    if (!is_timer_extra_lite(handle)) {
        // TIM10 and TIM11 don't have a CR2 and SMCR register whilst TIM9 does
        handle->CCR2 = 0;
        handle->SMCR &= ~(TIM_SMCR_SMS | TIM_SMCR_TS | TIM_SMCR_MSM | TIM_SMCR_ETF | TIM_SMCR_ETPS | TIM_SMCR_ECE | TIM_SMCR_ETP);
        if (handle != TIM9) {
            // The lite timers don't have a CR2, CCMR2, DCR, CCR3, CCR4 or DMAR register
            handle->CR2 &= ~(TIM_CR2_CCPC | TIM_CR2_CCUS | TIM_CR2_CCDS | TIM_CR2_MMS | TIM_CR2_TI1S | TIM_CR2_OIS1 | TIM_CR2_OIS1N | TIM_CR2_OIS2 |
                             TIM_CR2_OIS2N | TIM_CR2_OIS3 | TIM_CR2_OIS3N | TIM_CR2_OIS4);
            handle->CCMR2 &= ~(TIM_CCMR2_CC3S | TIM_CCMR2_OC3FE | TIM_CCMR2_OC3PE | TIM_CCMR2_OC3M | TIM_CCMR2_OC3CE | TIM_CCMR2_CC4S |
                               TIM_CCMR2_OC4FE | TIM_CCMR2_OC4PE | TIM_CCMR2_OC4M | TIM_CCMR2_OC4CE);
            handle->DCR &= ~(TIM_DCR_DBA | TIM_DCR_DBL);
            handle->DMAR &= ~TIM_DMAR_DMAB;
            handle->CCR3 = 0;
            handle->CCR4 = 0;
        }
    }

    if (is_timer_advanced(handle)) {
        handle->RCR &= ~TIM_RCR_REP;
        // The BDTR register cannot be cleared because the LOCK bits in it can only be written once.
        // To clear it, make use of the time_reset(...) function since it performs a bus level reset
        // of the given timer handle.
    }

    // Clear the registered callback for the current timer instance
    const timer_cb_t callback = {};
    TRY(timer_register_callback(handle, callback, TIMER_IRQ_CLEAR_ALL));

    return HAL_OK;
}

hal_err_t timer_reset(TIM_TypeDef* handle) {
    if (handle == TIM1) {
        reset_peripheral(RCC->APB2RSTR, RCC_APB2RSTR_TIM1RST);
    } else if (handle == TIM2) {
        reset_peripheral(RCC->APB1RSTR, RCC_APB1RSTR_TIM2RST);
    } else if (handle == TIM3) {
        reset_peripheral(RCC->APB1RSTR, RCC_APB1RSTR_TIM3RST);
    } else if (handle == TIM4) {
        reset_peripheral(RCC->APB1RSTR, RCC_APB1RSTR_TIM4RST);
    } else if (handle == TIM5) {
        reset_peripheral(RCC->APB1RSTR, RCC_APB1RSTR_TIM5RST);
    } else if (handle == TIM9) {
        reset_peripheral(RCC->APB2RSTR, RCC_APB2RSTR_TIM9RST);
    } else if (handle == TIM10) {
        reset_peripheral(RCC->APB2RSTR, RCC_APB2RSTR_TIM10RST);
    } else if (handle == TIM11) {
        reset_peripheral(RCC->APB2RSTR, RCC_APB2RSTR_TIM11RST);
    } else {
        return HAL_ERR_INVALID_ARG;
    }
    return HAL_OK;
}

hal_err_t timer_start_oneshot(TIM_TypeDef* handle, uint32_t timeout_us) {
    if (handle == NULL || timeout_us == 0) {
        return HAL_ERR_INVALID_ARG;
    }

    // Fail if the timer is already running
    // If needed to set a new timeout while the timer is still active, use timer_restart(...)
    if (handle->CR1 & TIM_CR1_CEN) {
        return HAL_ERR_INVALID_STATE;
    }

    // Set the auto-reload and prescaler values
    TRY(timer_set_arr_and_psc(handle, timeout_us));

    // Enable OPM and start the counter
    handle->CR1 |= (TIM_CR1_CEN | TIM_CR1_OPM);

    return HAL_OK;
}

hal_err_t timer_start_periodic(TIM_TypeDef* handle, uint32_t timeout_us) {
    if (handle == NULL || timeout_us == 0) {
        return HAL_ERR_INVALID_ARG;
    }

    // Fail if the timer is already running
    // If needed to set a new timeout while the timer is still active, use timer_restart(...)
    if (handle->CR1 & TIM_CR1_CEN) {
        return HAL_ERR_INVALID_STATE;
    }

    // Set the auto-reload and prescaler values
    TRY(timer_set_arr_and_psc(handle, timeout_us));

    // Disable OPM and start the counter
    handle->CR1 = TIM_CR1_CEN | (handle->CR1 & ~TIM_CR1_OPM);

    return HAL_OK;
}

hal_err_t timer_pause(TIM_TypeDef* handle) {
    return timer_stop_counter(handle);
}

hal_err_t timer_resume(TIM_TypeDef* handle) {
    return timer_start_counter(handle);
}

hal_err_t timer_restart(TIM_TypeDef* handle, uint32_t timeout_us) {
    // Pause the timer first
    TRY(timer_pause(handle));

    // Then restart based on whatever mode it was counting in previously
    if (handle->CR1 & TIM_CR1_OPM) {
        TRY(timer_start_oneshot(handle, timeout_us));
    } else {
        TRY(timer_start_periodic(handle, timeout_us));
    }

    return HAL_OK;
}
