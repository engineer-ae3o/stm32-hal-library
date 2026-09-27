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

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const timer_cb_t local_cb  = s_timer_cb_ctx[idx].callback;
    void* const      local_arg = s_timer_cb_ctx[idx].arg;

    // Clear only if in one pulse mode
    if (handle->CR1 & TIM_CR1_OPM) {
        s_timer_cb_ctx[idx].callback = NULL;
        s_timer_cb_ctx[idx].arg      = NULL;
    }
    __set_PRIMASK(primask);

    // Update event interrupt: underflow or overflow
    if (handle->SR & TIM_SR_UIF) {
        handle->SR &= ~TIM_SR_UIF;
        if (local_cb) {
            local_cb(local_arg);
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

hal_err_t timer_init(TIM_TypeDef* handle, timer_counter_dir_t direction, timer_cb_t callback, void* arg) {
    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU || callback == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    if (handle->CR1 & TIM_CR1_CEN) {
        return HAL_ERR_INVALID_STATE;
    }

    if ((handle == TIM9 || handle == TIM10 || handle == TIM11) && direction != TIMER_COUNTER_UP) {
        return HAL_ERR_NOT_SUPPORTED;
    }

    // Set the counting direction
    handle->CR1 = (uint32_t)(direction << TIM_CR1_DIR_Pos) | (handle->CR1 & ~TIM_CR1_DIR);

    // Register the callback for the current timer instance
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_timer_cb_ctx[idx].callback = callback;
    s_timer_cb_ctx[idx].arg      = arg;
    __set_PRIMASK(primask);

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
    handle->EGR &= ~(TIM_EGR_UG | TIM_EGR_CC1G | TIM_EGR_CC2G | TIM_EGR_CC3G | TIM_EGR_CC4G | TIM_EGR_COMG | TIM_EGR_TG | TIM_EGR_BG);
    handle->CNT  = 0;
    handle->PSC  = 0;
    handle->ARR  = 0;
    handle->CCR1 = 0;
    handle->CCR2 = 0;
    handle->CCR3 = 0;
    handle->CCR4 = 0;
    if (handle == TIM1) {
        handle->RCR &= ~TIM_RCR_REP;
    }

    // Disable the timer's NVIC interrupt
    if (handle == TIM1 || handle == TIM10) {
        NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn);
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

    // Enable update generation and the update event interrupt, and clear the update interrupt flag
    handle->EGR |= TIM_EGR_UG;
    handle->SR &= ~TIM_SR_UIF;
    handle->DIER |= TIM_DIER_UIE;

    // Enable the timer's NVIC interrupt
    IRQn_Type irq_type = 0;
    if (handle == TIM1 || handle == TIM10) {
        irq_type = TIM1_UP_TIM10_IRQn;
    } else {
        irq_type = s_timer_cb_ctx[idx].irq_type;
    }
    NVIC_SetPriority(irq_type, TIMER_NVIC_IRQ_PRIORITY);
    NVIC_ClearPendingIRQ(irq_type);
    NVIC_EnableIRQ(irq_type);

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

    // Enable update generation and the update event interrupt, and clear the update interrupt flag
    handle->EGR |= TIM_EGR_UG;
    handle->SR &= ~TIM_SR_UIF;
    handle->DIER |= TIM_DIER_UIE;

    // Enable the timer's NVIC interrupt
    IRQn_Type irq_type = 0;
    if (handle == TIM1 || handle == TIM10) {
        irq_type = TIM1_UP_TIM10_IRQn;
    } else {
        irq_type = s_timer_cb_ctx[idx].irq_type;
    }
    NVIC_SetPriority(irq_type, TIMER_NVIC_IRQ_PRIORITY);
    NVIC_ClearPendingIRQ(irq_type);
    NVIC_EnableIRQ(irq_type);

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

hal_err_t timer_set_arr_and_psc(TIM_TypeDef* handle, uint32_t timeout_us) {
    // Get the (PSC + 1) * (ARR + 1) value from the timer's clock frequency and the timeout
    uint32_t timer_freq_hz = 0;
    TRY(timer_get_frequency_hz(handle, &timer_freq_hz));
    const uint64_t psc_times_arr = ((uint64_t)timer_freq_hz * timeout_us) / 1'000'000U;

    // Get the maximum width of the prescaler and auto-reload registers
    const uint32_t max_psc_plus_1 = UINT16_MAX + 1;
    const uint64_t max_arr_plus_1 = (is_timer_32_bits(handle) ? UINT32_MAX : UINT16_MAX) + 1;

    // Bounds check the arguments against the width of the timers' registers
    const uint64_t max_psc_times_arr = max_psc_plus_1 * max_arr_plus_1;
    if (psc_times_arr > max_psc_times_arr) {
        return HAL_ERR_NOT_SUPPORTED;
    }

    // Compute suitable auto-reload and prescaler values
    // Minimum PSC such that ARR can cover the remainder: ceiling division
    uint64_t psc_plus_1 = (psc_times_arr + max_arr_plus_1 - 1) / max_arr_plus_1;
    if (psc_plus_1 == 0) {
        psc_plus_1 = 1; // Clamp to 1
    }

    // Round to nearest instead of floor to halve the worst case error
    uint64_t arr_plus_1 = (psc_times_arr + (psc_plus_1 / 2)) / psc_plus_1;
    if (gnu_unlikely(arr_plus_1 == 0)) {
        arr_plus_1 = 1; // Clamp to 1
    }

    // Set the actual prescaler and auto-reload values
    handle->ARR = (uint32_t)(arr_plus_1 - 1);
    handle->PSC = (uint32_t)(psc_plus_1 - 1);

    // Clear existing state
    handle->CNT  = 0;
    handle->CCR1 = 0;
    handle->CCR2 = 0;
    handle->CCR3 = 0;
    handle->CCR4 = 0;

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

hal_err_t timer_register_callback(timer_cb_t callback, void* arg, uint8_t idx) {
    if (idx >= ARRAY_SIZE(s_timer_cb_ctx)) {
        return HAL_ERR_INVALID_ARG;
    }
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_timer_cb_ctx[idx].callback = callback;
    s_timer_cb_ctx[idx].arg      = arg;
    __set_PRIMASK(primask);
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
