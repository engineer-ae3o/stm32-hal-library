#include "stm32f411xe.h"
#include "drivers/timer.h"
#include "utils/common.h"
#include "utils/err.h"

#include <stddef.h>


// Table of registered callbacks
typedef struct {
    timer_cb_t callback;
    void*      arg;
} cb_ctx_t;

// There are 8 TIMx peripherals
static cb_ctx_t s_cb_ctx[8] = {};


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

hal_err_t timer_init(TIM_TypeDef* handle, const timer_config_t* config, timer_cb_t cb, void* arg) {
    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU || config == NULL || cb == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    // Register the callback for the current timer instance
    __disable_irq();
    s_cb_ctx[idx].callback = cb;
    s_cb_ctx[idx].arg      = arg;
    __enable_irq();

    return HAL_OK;
}

hal_err_t timer_deinit(TIM_TypeDef* handle) {
    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU) {
        return HAL_ERR_INVALID_ARG;
    }

    // Clear the registered callback for the current timer instance
    __disable_irq();
    s_cb_ctx[idx].callback = NULL;
    s_cb_ctx[idx].arg      = NULL;
    __enable_irq();

    return HAL_OK;
}

hal_err_t timer_start_oneshot(TIM_TypeDef* handle, uint32_t timeout_us) {
    if (handle == NULL || timeout_us == 0) {
        return HAL_ERR_INVALID_ARG;
    }

    if (handle->CR1 & TIM_CR1_CEN) {
        return HAL_ERR_INVALID_STATE;
    }

    // TODO: Handle the starting of the timer in OPM

    // Set OPM mode and enable the counter
    handle->CR1 |= (TIM_CR1_CEN | TIM_CR1_OPM);

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
    handle->CR1 = TIM_CR1_CEN | (handle->CR1 & ~TIM_CR1_OPM);

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
