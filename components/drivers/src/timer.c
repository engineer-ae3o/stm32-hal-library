#include "stm32f411xe.h"
#include "drivers/timer_internals.h"
#include "drivers/timer.h"
#include "utils/common.h"
#include "utils/clock.h"
#include "utils/err.h"

#include <stddef.h>
#include <stdint.h>


// Table of registered callbacks
typedef struct {
    timer_cb_t      callback;
    void*           arg;
    const IRQn_Type irq_type;
} cb_ctx_t;

// There are 8 TIMx peripherals
static cb_ctx_t s_timer_cb_ctx[8] = {
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

    // TODO: Handle the timer interrupts
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

hal_err_t timer_init(TIM_TypeDef* handle, const timer_config_t* config, timer_cb_t callback, void* arg) {
    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU || config == NULL || callback == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    if ((handle == TIM9 || handle == TIM10 || handle == TIM11) && config->mode != TIMER_COUNTER_UP) {
        return HAL_ERR_NOT_SUPPORTED;
    }

    // Register the callback for the current timer instance
    __disable_irq();
    s_timer_cb_ctx[idx].callback = callback;
    s_timer_cb_ctx[idx].arg      = arg;
    __enable_irq();

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
    handle->CNT &= ~TIM_CNT_CNT;
    handle->PSC &= ~TIM_PSC_PSC;
    handle->ARR &= ~TIM_ARR_ARR;
    handle->RCR &= ~TIM_RCR_REP;

    // Clear the registered callback for the current timer instance
    __disable_irq();
    s_timer_cb_ctx[idx].callback = NULL;
    s_timer_cb_ctx[idx].arg      = NULL;
    __enable_irq();

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

    // Get the (PSC + 1) * (ARR + 1) value from the timer's clock frequency and the timeout
    uint32_t timer_freq_hz = 0;
    TRY(timer_get_frequency_hz(handle, &timer_freq_hz));
    const uint32_t psc_times_arr = (timer_freq_hz / 1'000'000U) * timeout_us;

    // Get the maximum width of the prescaler and auto-reload registers
    const uint32_t max_psc_plus_1 = UINT16_MAX + 1;
    const uint64_t max_arr_plus_1 = (is_timer_32_bits(handle) ? UINT32_MAX : UINT16_MAX) + 1;

    // Bounds check the arguments against the width of the timers' registers
    const uint64_t max_psc_times_arr = max_psc_plus_1 * max_arr_plus_1;
    if (psc_times_arr > max_psc_times_arr) {
        return HAL_ERR_NOT_SUPPORTED;
    }

    // Compute suitable auto-reload and prescaler values
    uint32_t arr_plus_1 = 0;
    uint32_t psc_plus_1 = 0;

    // Get the actual prescaler and auto-rload values
    handle->ARR = arr_plus_1 - 1;
    handle->PSC = psc_plus_1 - 1;

    // Enable the timer's NVIC interrupt
    IRQn_Type irq_type = 0;
    if (handle == TIM1 || handle == TIM10) {
        irq_type = TIM1_UP_TIM10_IRQn;
    } else {
        irq_type = s_timer_cb_ctx[idx].irq_type;
    }
    NVIC_SetPriority(irq_type, TIMERS_NVIC_IRQ_PRIORITY);
    NVIC_ClearPendingIRQ(irq_type);
    NVIC_EnableIRQ(irq_type);

    // Enable the update event interrupt
    handle->DIER |= TIM_DIER_UIE;

    // Set OPM mode and enable the counter, and disable auto-reload preload
    handle->CR1 = (TIM_CR1_CEN | TIM_CR1_OPM) | (handle->CR1 & ~TIM_CR1_ARPE);

    return HAL_OK;
}

hal_err_t timer_start_periodic(TIM_TypeDef* handle, uint32_t timeout_us) {
    if (handle == NULL || timeout_us == 0) {
        return HAL_ERR_INVALID_ARG;
    }

    if (handle->CR1 & TIM_CR1_CEN) {
        return HAL_ERR_INVALID_STATE;
    }

    // TODO: Handle the starting of the timer in periodic mode

    // Disable OPM mode and enable the counter
    handle->CR1 = TIM_CR1_CEN | (handle->CR1 & ~(TIM_CR1_OPM | TIM_CR1_ARPE));

    return HAL_OK;
}

hal_err_t timer_stop(TIM_TypeDef* handle) {
    if (handle == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    if (!(handle->CR1 & TIM_CR1_CEN)) {
        return HAL_ERR_INVALID_STATE;
    }

    // Disable the counter. Pretty straightforward
    handle->CR1 &= ~TIM_CR1_CEN;

    return HAL_OK;
}

hal_err_t timer_restart(TIM_TypeDef* handle, uint32_t timeout_us) {
    // Stop the timer first
    TRY(timer_stop(handle));

    // Then restart based on what mode it was counting in previously
    if (handle->CR1 & TIM_CR1_OPM) {
        TRY(timer_start_oneshot(handle, timeout_us));
    } else {
        TRY(timer_start_periodic(handle, timeout_us));
    }

    return HAL_OK;
}

bool is_timer_32_bits(TIM_TypeDef* handle) {
    bool result = true;
    if (handle == TIM2 || handle == TIM3 || handle == TIM4 || handle == TIM5) {
        result = true;
    } else if (handle == TIM1 || handle == TIM9 || handle == TIM10 || handle == TIM11) {
        result = false;
    } else {
        ASSERT(false);
    }
    return result;
}

hal_err_t timer_get_frequency_hz(TIM_TypeDef* handle, uint32_t* frequency) {
    if (handle == NULL || frequency == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    *frequency = 0;
    return HAL_OK;
}

hal_err_t timer_register_callback(timer_cb_t callback, void* arg, uint8_t idx) {
    if (idx >= ARRAY_SIZE(s_timer_cb_ctx)) {
        return HAL_ERR_INVALID_ARG;
    }
    __disable_irq();
    s_timer_cb_ctx[idx].callback = callback;
    s_timer_cb_ctx[idx].arg      = arg;
    __enable_irq();
    return HAL_OK;
}


// Interrupt handlers for all timers
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
