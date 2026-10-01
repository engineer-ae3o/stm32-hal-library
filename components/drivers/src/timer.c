#include "stm32f411xe.h"
#include "drivers/timer_internals.h"
#include "drivers/timer.h"
#include "utils/common.h"
#include "utils/clock.h"
#include "utils/err.h"

#include <stddef.h>
#include <stdint.h>


// Table of registered callbacks and state for the timer instance
typedef struct {
    timer_cb_t      callback;
    void*           arg;
    const IRQn_Type irq_type;
} timer_ctx_t;

static timer_ctx_t s_timer_cb_ctx[] = {
    // TIM1
    {.callback = NULL, .arg = NULL, .irq_type = TIM1_CC_IRQn},
    // TIM2
    {.callback = NULL, .arg = NULL, .irq_type = TIM2_IRQn},
    // TIM3
    {.callback = NULL, .arg = NULL, .irq_type = TIM3_IRQn},
    // TIM4
    {.callback = NULL, .arg = NULL, .irq_type = TIM4_IRQn},
    // TIM5
    {.callback = NULL, .arg = NULL, .irq_type = TIM5_IRQn},
    // TIM9
    {.callback = NULL, .arg = NULL, .irq_type = TIM1_BRK_TIM9_IRQn},
    // TIM10
    {.callback = NULL, .arg = NULL, .irq_type = TIM1_UP_TIM10_IRQn},
    // TIM11
    {.callback = NULL, .arg = NULL, .irq_type = TIM1_TRG_COM_TIM11_IRQn},
};


// Helpers
[[__gnu__::__always_inline__]] static inline uint8_t get_index(TIM_TypeDef* handle) {
    if (handle == TIM1) {
        return 0U;
    } else if (handle == TIM2) {
        return 1U;
    } else if (handle == TIM3) {
        return 2U;
    } else if (handle == TIM4) {
        return 3U;
    } else if (handle == TIM5) {
        return 4U;
    } else if (handle == TIM9) {
        return 5U;
    } else if (handle == TIM10) {
        return 6U;
    } else if (handle == TIM11) {
        return 7U;
    } else {
        return 0xFFU;
    }
}

[[__gnu__::__always_inline__]] static inline void timer_isr_helper(TIM_TypeDef* handle) {
    const uint8_t idx = get_index(handle);
    ASSERT(idx != 0xFFU);

    // Update event interrupt: underflow or overflow
    if (handle->SR & TIM_SR_UIF) {
        handle->SR &= ~TIM_SR_UIF;
        if (s_timer_cb_ctx[idx].callback) {
            s_timer_cb_ctx[idx].callback(s_timer_cb_ctx[idx].arg);
        }
    }

    // Only available on the advanced timers
    if (is_timer_advanced(handle)) {
        // Break event interrupt
        if (handle->SR & TIM_SR_BIF) {
            handle->SR &= ~TIM_SR_BIF;
            if (s_timer_cb_ctx[idx].callback) {
                s_timer_cb_ctx[idx].callback(s_timer_cb_ctx[idx].arg);
            }
        }
    }
}


// Public API
hal_err_t timer_clock_enable(TIM_TypeDef* handle, bool enable) {
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

hal_err_t timer_init(TIM_TypeDef* handle, timer_count_dir_t direction, timer_cb_t callback, void* arg) {
    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU || callback == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    if ((handle == TIM9 || handle == TIM10 || handle == TIM11) && direction != TIMER_COUNTER_UP) {
        return HAL_ERR_NOT_SUPPORTED;
    }

    if (handle->CR1 & TIM_CR1_CEN) {
        return HAL_ERR_INVALID_STATE;
    }

    // Clear all residual state
    TRY(timer_deinit(handle));

    // Register the callback
    TRY(timer_register_callback(handle, callback, arg, UPDATE_EVENT));

    // Set the counting direction. Edge aligned (up or downcounting)
    handle->CR1 = (uint32_t)(direction << TIM_CR1_DIR_Pos);

    return HAL_OK;
}

hal_err_t timer_deinit(TIM_TypeDef* handle) {
    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU) {
        return HAL_ERR_INVALID_ARG;
    }

    // Clear all state
    handle->CR1 &= ~(TIM_CR1_CEN | TIM_CR1_UDIS | TIM_CR1_URS | TIM_CR1_OPM | TIM_CR1_DIR | TIM_CR1_CMS | TIM_CR1_ARPE | TIM_CR1_CKD);
    handle->CR2 &= ~(TIM_CR2_CCPC | TIM_CR2_CCUS | TIM_CR2_CCDS | TIM_CR2_MMS | TIM_CR2_TI1S | TIM_CR2_OIS1 | TIM_CR2_OIS1N | TIM_CR2_OIS2 |
                     TIM_CR2_OIS2N | TIM_CR2_OIS3 | TIM_CR2_OIS3N | TIM_CR2_OIS4);
    handle->DIER &=
        ~(TIM_DIER_UIE | TIM_DIER_CC1IE | TIM_DIER_CC2IE | TIM_DIER_CC3IE | TIM_DIER_CC4IE | TIM_DIER_COMIE | TIM_DIER_TIE | TIM_DIER_BIE |
          TIM_DIER_UDE | TIM_DIER_CC1DE | TIM_DIER_CC2DE | TIM_DIER_CC3DE | TIM_DIER_CC4DE | TIM_DIER_COMDE | TIM_DIER_TDE);
    handle->SR &= ~(TIM_SR_UIF | TIM_SR_CC1IF | TIM_SR_CC2IF | TIM_SR_CC3IF | TIM_SR_CC4IF | TIM_SR_COMIF | TIM_SR_TIF | TIM_SR_BIF | TIM_SR_CC1OF |
                    TIM_SR_CC2OF | TIM_SR_CC3OF | TIM_SR_CC4OF);
    handle->CCER &=
        ~(TIM_CCER_CC1E | TIM_CCER_CC1P | TIM_CCER_CC1NE | TIM_CCER_CC1NP | TIM_CCER_CC2E | TIM_CCER_CC2P | TIM_CCER_CC2NP | TIM_CCER_CC2NE |
          TIM_CCER_CC3E | TIM_CCER_CC3P | TIM_CCER_CC3NE | TIM_CCER_CC3NP | TIM_CCER_CC4E | TIM_CCER_CC4P | TIM_CCER_CC4NP);
    handle->CCMR1 &= ~(TIM_CCMR1_CC1S | TIM_CCMR1_OC1FE | TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M | TIM_CCMR1_OC1CE | TIM_CCMR1_CC2S | TIM_CCMR1_OC2FE |
                       TIM_CCMR1_OC2PE | TIM_CCMR1_OC2M | TIM_CCMR1_OC2CE);
    handle->CCMR2 &= ~(TIM_CCMR2_CC3S | TIM_CCMR2_OC3FE | TIM_CCMR2_OC3PE | TIM_CCMR2_OC3M | TIM_CCMR2_OC3CE | TIM_CCMR2_CC4S | TIM_CCMR2_OC4FE |
                       TIM_CCMR2_OC4PE | TIM_CCMR2_OC4M | TIM_CCMR2_OC4CE);
    handle->CNT  = 0;
    handle->PSC  = 0;
    handle->ARR  = 0;
    handle->CCR1 = 0;
    handle->CCR2 = 0;
    handle->CCR3 = 0;
    handle->CCR4 = 0;

    if (handle == TIM1) {
        handle->RCR &= ~TIM_RCR_REP;
        handle->BDTR &= ~(TIM_BDTR_DTG | TIM_BDTR_LOCK | TIM_BDTR_OSSI | TIM_BDTR_OSSR | TIM_BDTR_BKE | TIM_BDTR_BKP | TIM_BDTR_AOE | TIM_BDTR_MOE);
    }

    // Disable the timer's NVIC interrupt
    if (handle == TIM1 || handle == TIM9 || handle == TIM10 || handle == TIM11) {
        // Since these timers share an NVIC line amongst themselves, we can't
        // knowingly disable the NVIC irq line since we could take out the other
        // timer. So its left up to the user to disable it as they please.
    } else {
        NVIC_DisableIRQ(s_timer_cb_ctx[idx].irq_type);
    }

    // Clear the registered callback for the current timer instance
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_timer_cb_ctx[idx].callback = NULL;
    s_timer_cb_ctx[idx].arg      = NULL;
    __set_PRIMASK(primask);

    return HAL_OK;
}

hal_err_t timer_start_oneshot(TIM_TypeDef* handle, uint32_t timeout_us) {
    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU || timeout_us == 0) {
        return HAL_ERR_INVALID_ARG;
    }

    if (handle->CR1 & TIM_CR1_CEN) {
        return HAL_ERR_INVALID_STATE;
    }

    // Set the auto-reload and prescaler values
    TRY(timer_set_arr_and_psc(handle, timeout_us));

    // Enable the update event interrupt
    handle->DIER |= TIM_DIER_UIE;

    // Set OPM mode, and enable the counter and auto-reload preload, and set URS so only a UEV triggers an interrupt
    handle->CR1 |= (TIM_CR1_CEN | TIM_CR1_OPM | TIM_CR1_ARPE | TIM_CR1_URS);
    return HAL_OK;
}

hal_err_t timer_start_periodic(TIM_TypeDef* handle, uint32_t timeout_us) {
    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU || timeout_us == 0) {
        return HAL_ERR_INVALID_ARG;
    }

    if (handle->CR1 & TIM_CR1_CEN) {
        return HAL_ERR_INVALID_STATE;
    }

    // Set the auto-reload and prescaler values
    TRY(timer_set_arr_and_psc(handle, timeout_us));

    // Enable the update event interrupt
    handle->DIER |= TIM_DIER_UIE;

    // Disable OPM mode, and enable the counter and auto-reload preload, and set URS so only a UEV triggers an interrupt
    handle->CR1 = (TIM_CR1_CEN | TIM_CR1_ARPE | TIM_CR1_URS) | (handle->CR1 & ~TIM_CR1_OPM);
    return HAL_OK;
}

hal_err_t timer_pause(TIM_TypeDef* handle) {
    if (handle == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    if (!(handle->CR1 & TIM_CR1_CEN)) {
        return HAL_ERR_INVALID_STATE;
    }

    // Pause the timer by disabling its counter
    handle->CR1 &= ~TIM_CR1_CEN;
    return HAL_OK;
}

hal_err_t timer_resume(TIM_TypeDef* handle) {
    if (handle == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    if (handle->CR1 & TIM_CR1_CEN) {
        return HAL_ERR_INVALID_STATE;
    }

    // Resume the timer by re-enabling its counter
    handle->CR1 |= TIM_CR1_CEN;
    return HAL_OK;
}

hal_err_t timer_restart(TIM_TypeDef* handle, uint32_t timeout_us) {
    // Pause the timer first
    TRY(timer_pause(handle));

    // Then restart based on what mode it was counting in previously
    if (handle->CR1 & TIM_CR1_OPM) {
        TRY(timer_start_oneshot(handle, timeout_us));
    } else {
        TRY(timer_start_periodic(handle, timeout_us));
    }

    return HAL_OK;
}


// Internal helpers
bool is_timer_on_apb1(TIM_TypeDef* handle) {
    bool result = true;
    if (handle == TIM2 || handle == TIM3 || handle == TIM4 || handle == TIM5) {
        // result = true; // Already true. Just no-op
    } else if (handle == TIM1 || handle == TIM9 || handle == TIM10 || handle == TIM11) {
        result = false;
    } else {
        ASSERT(false);
    }
    return result;
}

bool is_timer_32_bits(TIM_TypeDef* handle) {
    bool result = true;
    if (handle == TIM2 || handle == TIM5) {
        // result = true; // Already true. Just no-op
    } else if (handle == TIM1 || handle == TIM3 || handle == TIM4 || handle == TIM9 || handle == TIM10 || handle == TIM11) {
        result = false;
    } else {
        ASSERT(false);
    }
    return result;
}

bool is_timer_advanced(TIM_TypeDef* handle) {
    return handle == TIM1;
}

hal_err_t timer_set_arr_and_psc(TIM_TypeDef* handle, uint32_t timeout_us) {
    // Set the (PSC + 1) * (ARR + 1) value from the timer's clock frequency and the timeout
    uint32_t timer_freq_hz = 0;
    TRY(timer_get_frequency_hz(handle, &timer_freq_hz));
    const uint64_t psc_times_arr = ((uint64_t)timer_freq_hz * timeout_us) / 1'000'000U;

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
    handle->CCR2 = 0;
    handle->CCR3 = 0;
    handle->CCR4 = 0;

    // Generate an update event and clear the update interrupt
    // flag since the ARR and PSC registers contain new values
    handle->EGR = TIM_EGR_UG;
    handle->SR &= ~TIM_SR_UIF;

    return HAL_OK;
}

hal_err_t timer_get_frequency_hz(TIM_TypeDef* handle, uint32_t* timer_freq_hz) {
    if (handle == NULL || timer_freq_hz == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    const uint32_t system_core_clock = get_system_core_clock();
    const uint32_t apb_clock_hz      = is_timer_on_apb1(handle) ? get_apb1_core_clock() : get_apb2_core_clock();

    if (RCC->DCKCFGR & RCC_DCKCFGR_TIMPRE) {
        // High frequency timer mode
        if ((system_core_clock == apb_clock_hz) || ((system_core_clock / apb_clock_hz) == 2)) {
            // The APB prescaler is either 1 or 2
            *timer_freq_hz = system_core_clock;
        } else {
            // The APB prescaler is greater than 2
            *timer_freq_hz = apb_clock_hz * 4;
        }
    } else {
        // Standard frequency timer mode
        if (system_core_clock == apb_clock_hz) {
            // The APB prescaler is 1
            *timer_freq_hz = apb_clock_hz;
        } else {
            // The APB prescaler is greater than 1
            *timer_freq_hz = apb_clock_hz * 2;
        }
    }

    return HAL_OK;
}

hal_err_t timer_register_callback(TIM_TypeDef* handle, timer_cb_t callback, void* arg, advanced_timer_irq_type_t type) {
    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU) {
        return HAL_ERR_INVALID_ARG;
    }

    // Register the callback
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_timer_cb_ctx[idx].callback = callback;
    s_timer_cb_ctx[idx].arg      = arg;
    __set_PRIMASK(primask);

    // Get the irq type for the current timer instance
    IRQn_Type irq_type = 0;
    if (handle == TIM1) {
        switch (type) {
            case UPDATE_EVENT:
                irq_type = TIM1_UP_TIM10_IRQn;
                break;
            case BREAK_EVENT:
                irq_type = TIM1_BRK_TIM9_IRQn;
                break;
            case CAPTURE_COMPARE:
                irq_type = TIM1_CC_IRQn;
                break;
            case TRG_COM_EVENT:
                irq_type = TIM1_TRG_COM_TIM11_IRQn;
                break;
        }
    } else {
        irq_type = s_timer_cb_ctx[idx].irq_type;
    }

    // Enable the timer's NVIC interrupt
    NVIC_SetPriority(irq_type, TIMER_NVIC_IRQ_PRIORITY);
    NVIC_ClearPendingIRQ(irq_type);
    NVIC_EnableIRQ(irq_type);

    return HAL_OK;
}


// Interrupt handlers
void TIM1_CC_IRQHandler(void) {
    timer_isr_helper(TIM1);
}

void TIM2_IRQHandler(void) {
    timer_isr_helper(TIM2);
}

void TIM3_IRQHandler(void) {
    timer_isr_helper(TIM3);
}

void TIM4_IRQHandler(void) {
    timer_isr_helper(TIM4);
}

void TIM5_IRQHandler(void) {
    timer_isr_helper(TIM5);
}

void TIM1_BRK_TIM9_IRQHandler(void) {
    timer_isr_helper(TIM1);
    timer_isr_helper(TIM9);
}

void TIM1_UP_TIM10_IRQHandler(void) {
    timer_isr_helper(TIM1);
    timer_isr_helper(TIM10);
}

void TIM1_TRG_COM_TIM11_IRQHandler(void) {
    timer_isr_helper(TIM1);
    timer_isr_helper(TIM11);
}
