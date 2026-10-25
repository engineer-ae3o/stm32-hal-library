#include "stm32f411xe.h"
#include "drivers/src/timer_internals.h"
#include "drivers/timer_types.h"
#include "drivers/timer.h"
#include "utils/common.h"
#include "utils/clock.h"
#include "utils/err.h"

#include <string.h>
#include <stdint.h>


// Table of registered callbacks and state for the timer instance
typedef struct {
    timer_cb_t      callback;
    const IRQn_Type irq_type;
} timer_ctx_t;

static timer_ctx_t s_timer_cb_ctx[] = {
    // TIM1
    // NOTE: None of TIM1's callbacks are stored in this table. Refer below
    [0] = {.callback = {}, .irq_type = TIM1_CC_IRQn},
    // TIM2
    [1] = {.callback = {}, .irq_type = TIM2_IRQn},
    // TIM3
    [2] = {.callback = {}, .irq_type = TIM3_IRQn},
    // TIM4
    [3] = {.callback = {}, .irq_type = TIM4_IRQn},
    // TIM5
    [4] = {.callback = {}, .irq_type = TIM5_IRQn},
    // TIM9
    // NOTE: Only TIM9's callbacks are stored here
    [5] = {.callback = {}, .irq_type = TIM1_BRK_TIM9_IRQn},
    // TIM10
    // NOTE: Only TIM10's callbacks are stored here
    [6] = {.callback = {}, .irq_type = TIM1_UP_TIM10_IRQn},
    // TIM11
    // NOTE: Only TIM11's callbacks are stored here
    [7] = {.callback = {}, .irq_type = TIM1_TRG_COM_TIM11_IRQn},
};

// Since the interrupt vectors are shared, we have to find
// and store the interrupt callbacks separately
typedef struct {
    timer_cb_t update;
    timer_cb_t trigger;
    timer_cb_t commutation;
    timer_cb_t break_input;
    timer_cb_t capture_compare;
} advanced_tim_cb_ctx_t;

static advanced_tim_cb_ctx_t s_tim1_cb_ctx = {};


// Helpers
[[__gnu__::__always_inline__]] static inline uint8_t
get_index(TIM_TypeDef* handle) {
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

[[__gnu__::__always_inline__]] static inline void
timer_isr_helper(TIM_TypeDef* handle) {
    // Only available on the advanced timers
    if (is_timer_advanced(handle)) {
        // Update event interrupt
        if ((handle->DIER & TIM_DIER_UIE) && (handle->SR & TIM_SR_UIF)) {
            handle->SR = ~TIM_SR_UIF;
            if (s_tim1_cb_ctx.update.cb) {
                s_tim1_cb_ctx.update.cb(s_tim1_cb_ctx.update.arg,
                                        TIMER_CHANNEL_NC,
                                        handle->CR1 & TIM_CR1_DIR);
            }
        }
        // Break event interrupt
        if ((handle->DIER & TIM_DIER_BIE) && (handle->SR & TIM_SR_BIF)) {
            handle->SR = ~TIM_SR_BIF;
            if (s_tim1_cb_ctx.break_input.cb) {
                s_tim1_cb_ctx.break_input.cb(s_tim1_cb_ctx.break_input.arg,
                                             TIMER_CHANNEL_NC,
                                             TIMER_COUNTER_DIR_NONE);
            }
        }
        // Commutation event interrupt
        if ((handle->DIER & TIM_DIER_COMIE) && (handle->SR & TIM_SR_COMIF)) {
            handle->SR = ~TIM_SR_COMIF;
            if (s_tim1_cb_ctx.commutation.cb) {
                s_tim1_cb_ctx.commutation.cb(s_tim1_cb_ctx.commutation.arg,
                                             TIMER_CHANNEL_NC,
                                             TIMER_COUNTER_DIR_NONE);
            }
        }
        // Trigger event interrupt
        if ((handle->DIER & TIM_DIER_TIE) && (handle->SR & TIM_SR_TIF)) {
            handle->SR = ~TIM_SR_TIF;
            if (s_tim1_cb_ctx.trigger.cb) {
                s_tim1_cb_ctx.trigger.cb(s_tim1_cb_ctx.trigger.arg,
                                         TIMER_CHANNEL_NC,
                                         TIMER_COUNTER_DIR_NONE);
            }
        }
        // Capture compare interrupt(s)
        if ((handle->DIER & TIM_DIER_CC1IE) && (handle->SR & TIM_SR_CC1IF)) {
            handle->SR = ~TIM_SR_CC1IF;
            if (s_tim1_cb_ctx.capture_compare.cb) {
                s_tim1_cb_ctx.capture_compare.cb(
                    s_tim1_cb_ctx.capture_compare.arg,
                    TIMER_CHANNEL_1,
                    TIMER_COUNTER_DIR_NONE);
            }
        }
        if ((handle->DIER & TIM_DIER_CC2IE) && (handle->SR & TIM_SR_CC2IF)) {
            handle->SR = ~TIM_SR_CC2IF;
            if (s_tim1_cb_ctx.capture_compare.cb) {
                s_tim1_cb_ctx.capture_compare.cb(
                    s_tim1_cb_ctx.capture_compare.arg,
                    TIMER_CHANNEL_2,
                    TIMER_COUNTER_DIR_NONE);
            }
        }
        if ((handle->DIER & TIM_DIER_CC3IE) && (handle->SR & TIM_SR_CC3IF)) {
            handle->SR = ~TIM_SR_CC3IF;
            if (s_tim1_cb_ctx.capture_compare.cb) {
                s_tim1_cb_ctx.capture_compare.cb(
                    s_tim1_cb_ctx.capture_compare.arg,
                    TIMER_CHANNEL_3,
                    TIMER_COUNTER_DIR_NONE);
            }
        }
        if ((handle->DIER & TIM_DIER_CC4IE) && (handle->SR & TIM_SR_CC4IF)) {
            handle->SR = ~TIM_SR_CC4IF;
            if (s_tim1_cb_ctx.capture_compare.cb) {
                s_tim1_cb_ctx.capture_compare.cb(
                    s_tim1_cb_ctx.capture_compare.arg,
                    TIMER_CHANNEL_4,
                    TIMER_COUNTER_DIR_NONE);
            }
        }
        return;
    }

    // Other non-advanced timers
    const uint8_t idx = get_index(handle);
    ASSERT(idx != 0xFFU);

    // Update event interrupt
    if ((handle->DIER & TIM_DIER_UIE) && (handle->SR & TIM_SR_UIF)) {
        handle->SR = ~TIM_SR_UIF;
        if (s_timer_cb_ctx[idx].callback.cb) {
            s_timer_cb_ctx[idx].callback.cb(s_timer_cb_ctx[idx].callback.arg,
                                            TIMER_CHANNEL_NC,
                                            handle->CR1 & TIM_CR1_DIR);
        }
    }
    // Capture compare interrupt(s)
    if ((handle->DIER & TIM_DIER_CC1IE) && (handle->SR & TIM_SR_CC1IF)) {
        handle->SR = ~TIM_SR_CC1IF;
        if (s_timer_cb_ctx[idx].callback.cb) {
            s_timer_cb_ctx[idx].callback.cb(s_timer_cb_ctx[idx].callback.arg,
                                            TIMER_CHANNEL_1,
                                            TIMER_COUNTER_DIR_NONE);
        }
    }
    if ((handle->DIER & TIM_DIER_CC2IE) && (handle->SR & TIM_SR_CC2IF)) {
        handle->SR = ~TIM_SR_CC2IF;
        if (s_timer_cb_ctx[idx].callback.cb) {
            s_timer_cb_ctx[idx].callback.cb(s_timer_cb_ctx[idx].callback.arg,
                                            TIMER_CHANNEL_2,
                                            TIMER_COUNTER_DIR_NONE);
        }
    }
    if ((handle->DIER & TIM_DIER_CC3IE) && (handle->SR & TIM_SR_CC3IF)) {
        handle->SR = ~TIM_SR_CC3IF;
        if (s_timer_cb_ctx[idx].callback.cb) {
            s_timer_cb_ctx[idx].callback.cb(s_timer_cb_ctx[idx].callback.arg,
                                            TIMER_CHANNEL_3,
                                            TIMER_COUNTER_DIR_NONE);
        }
    }
    if ((handle->DIER & TIM_DIER_CC4IE) && (handle->SR & TIM_SR_CC4IF)) {
        handle->SR = ~TIM_SR_CC4IF;
        if (s_timer_cb_ctx[idx].callback.cb) {
            s_timer_cb_ctx[idx].callback.cb(s_timer_cb_ctx[idx].callback.arg,
                                            TIMER_CHANNEL_4,
                                            TIMER_COUNTER_DIR_NONE);
        }
    }
}


// Internal helpers
bool is_timer_lite(TIM_TypeDef* handle) {
    return (handle == TIM9 || handle == TIM10 || handle == TIM11);
}

bool is_timer_advanced(TIM_TypeDef* handle) {
    return (handle == TIM1);
}

bool is_timer_extra_lite(TIM_TypeDef* handle) {
    return (handle == TIM10 || handle == TIM11);
}

bool is_timer_on_apb1(TIM_TypeDef* handle) {
    bool result = true;
    if (handle == TIM2 || handle == TIM3 || handle == TIM4 || handle == TIM5) {
        // result = true; // Already true. Just no-op
    } else if (handle == TIM1 || handle == TIM9 || handle == TIM10 ||
               handle == TIM11) {
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
    } else if (handle == TIM1 || handle == TIM3 || handle == TIM4 ||
               handle == TIM9 || handle == TIM10 || handle == TIM11) {
        result = false;
    } else {
        ASSERT(false);
    }
    return result;
}

uint32_t timer_get_num_channels(TIM_TypeDef* handle) {
    if (handle == TIM1 || handle == TIM2 || handle == TIM3 || handle == TIM4 ||
        handle == TIM5) {
        return MAX_TIM1_CHANNELS;
    } else if (handle == TIM9) {
        return MAX_TIM9_CHANNELS;
    } else if (handle == TIM10 || handle == TIM11) {
        return MAX_TIM10_CHANNELS;
    } else {
        ASSERT(false);
        return 0;
    }
}

uint32_t timer_get_frequency_hz(TIM_TypeDef* handle) {
    ASSERT(handle);

    const uint32_t system_core_clock = get_system_core_clock();
    const uint32_t apb_clock_hz      = is_timer_on_apb1(handle)
                                           ? get_apb1_core_clock()
                                           : get_apb2_core_clock();
    const uint32_t prescaler         = system_core_clock / apb_clock_hz;

    if (RCC->DCKCFGR & RCC_DCKCFGR_TIMPRE) {
        // High frequency timer mode
        return (prescaler > 2) ? (apb_clock_hz * 4) : system_core_clock;
    } else {
        // Standard frequency timer mode
        return (prescaler == 1) ? apb_clock_hz : (apb_clock_hz * 2);
    }
}

hal_err_t timer_stop_counter(TIM_TypeDef* handle) {
    if (handle == NULL) {
        return HAL_ERR_INVALID_ARG;
    }
    handle->CR1 &= ~TIM_CR1_CEN;
    return HAL_OK;
}

hal_err_t timer_start_counter(TIM_TypeDef* handle) {
    if (handle == NULL) {
        return HAL_ERR_INVALID_ARG;
    }
    handle->CR1 |= TIM_CR1_CEN;
    return HAL_OK;
}

hal_err_t timer_register_callback(TIM_TypeDef*              handle,
                                  timer_cb_t                callback,
                                  advanced_timer_irq_type_t type) {
    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU) {
        return HAL_ERR_INVALID_ARG;
    }

    // Disable all interrupts
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    // Get the irq type for the current timer instance
    IRQn_Type irq_type = 0;
    if (is_timer_advanced(handle)) {
        switch (type) {
            case TIMER_IRQ_UPDATE_EVENT:
                irq_type = TIM1_UP_TIM10_IRQn;
                // Register the update event callback
                s_tim1_cb_ctx.update = callback;
                break;
            case TIMER_IRQ_BREAK_EVENT:
                irq_type = TIM1_BRK_TIM9_IRQn;
                // Register the break event callback
                s_tim1_cb_ctx.break_input = callback;
                break;
            case TIMER_IRQ_CAPTURE_COMPARE:
                irq_type = TIM1_CC_IRQn;
                // Register the capture compare event callback
                s_tim1_cb_ctx.capture_compare = callback;
                break;
            case TIMER_IRQ_TRIGGER_EVENT:
                irq_type = TIM1_TRG_COM_TIM11_IRQn;
                // Register the trigger event callback
                s_tim1_cb_ctx.trigger = callback;
                break;
            case TIMER_IRQ_COMMUTATION_EVENT:
                irq_type = TIM1_TRG_COM_TIM11_IRQn;
                // Register the commutation event callback
                s_tim1_cb_ctx.commutation = callback;
                break;
            case TIMER_IRQ_CLEAR_ALL:
                // Clear all state regardless of the callback passed in
                memset(&s_tim1_cb_ctx, 0, sizeof(s_tim1_cb_ctx));
                goto disable_nvic_irq;
                break;
            default:
                __set_PRIMASK(primask);
                return HAL_ERR_INVALID_ARG;
        }
    } else {
        irq_type = s_timer_cb_ctx[idx].irq_type;
        if (type == TIMER_IRQ_CLEAR_ALL) {
            // Clear all state regardless of the callback passed in
            memset(&s_timer_cb_ctx[idx].callback,
                   0,
                   sizeof(s_timer_cb_ctx[idx].callback));
            goto disable_nvic_irq;
        } else {
            // Register the irq callback for the non advanced timers
            s_timer_cb_ctx[idx].callback = callback;
        }
    }

    // Enable the timer's NVIC interrupt only if the callback is valid. Disable otherwise
    if (callback.cb != NULL) {
        NVIC_SetPriority(irq_type, TIMER_NVIC_IRQ_PRIORITY);
        NVIC_ClearPendingIRQ(irq_type);
        NVIC_EnableIRQ(irq_type);
    } else {
    disable_nvic_irq:
        if (handle == TIM1 || handle == TIM9 || handle == TIM10 ||
            handle == TIM11) {
            // Since these timers share an NVIC line amongst themselves, we can't
            // knowingly disable the NVIC irq line since we could take out the other
            // timer. So its left up to the caller to disable it as they see fit.
        } else {
            NVIC_DisableIRQ(s_timer_cb_ctx[idx].irq_type);
        }
    }

    // Restore the interrupts to their previous state
    __set_PRIMASK(primask);

    return HAL_OK;
}

hal_err_t timer_filter_ns_to_ic_code(TIM_TypeDef*  handle,
                                     tim_clk_div_t clk_div,
                                     uint32_t      filter_ns,
                                     uint32_t*     code) {
    if (handle == NULL || clk_div > TIM_CLK_DIV_4 || code == NULL) {
        return HAL_ERR_INVALID_ARG;
    }
    if (filter_ns == 0) {
        *code = 0;
        return HAL_OK;
    }

    // The filter multiplier table. Gotten from the TRM. The needed code value is the index into this array whose filter value
    // meets or just exceeds the number of ticks (derived from the timer's frequency and the requested filter time in ns)
    static const uint16_t filter_multipliers[] = {
        1, 2, 4, 8, 12, 16, 24, 32, 48, 64, 80, 96, 128, 160, 192, 256};

    // Get the target ticks from the requested filter time, the timer frequency and clock divider
    // Target number of ticks directly from the timer clock
    const uint32_t target_cnt_ticks = (uint32_t)ceil_div_u64(
        (uint64_t)timer_get_frequency_hz(handle) * filter_ns, 1'000'000'000ULL);
    // Target number of ticks from the divided down timer clock
    const uint32_t target_dts_ticks =
        ceil_div_u32(target_cnt_ticks, (1UL << clk_div));

    // We start at 1, otherwise, we risk using a filter code of 0 even when the requested filter time (ns) isn't 0
    for (uint32_t i = 1; i < ARRAY_SIZE(filter_multipliers); i++) {
        // Code values (indexes) 0-3 are mapped to a prescaler of 1, which uses the timer clock directly
        if (filter_multipliers[i] >=
            ((i <= 3) ? target_cnt_ticks : target_dts_ticks)) {
            *code = i;
            return HAL_OK;
        }
    }

    // No viable filter multiplier
    return HAL_ERR_NOT_SUPPORTED;
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
