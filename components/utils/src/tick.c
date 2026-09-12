#include "stm32f411xe.h"
#include "utils/common.h"
#include "utils/clock.h"
#include "utils/tick.h"
#include "utils/log.h"

#include <stdatomic.h>


static atomic_uint s_tick_counter = 0;

void systick_init(void) {
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;

    static bool first_init = true;
    if (gnu_unlikely(first_init)) {
        LOGI("Tick",
             "Initializing the SysTick as the tick timer source with an HCLK of %luMHz from the %s",
             get_system_core_clock() / 1'000'000U,
             is_sysclk_on_hse() ? "HSE" : "HSI");
        NVIC_SetPriority(SysTick_IRQn, SysTick_NVIC_IRQ_PRIORITY);
        first_init = false;
    } else {
        LOGI("Tick",
             "Reinitializing the SysTick as the tick timer source with an HCLK of %luMHz from the %s",
             get_system_core_clock() / 1'000'000U,
             is_sysclk_on_hse() ? "HSE" : "HSI");
    }

    SysTick->VAL  = 0;
    SysTick->LOAD = (((get_system_core_clock() / 8) / TICK_RATE_Hz) - 1) & SysTick_LOAD_RELOAD_Msk;
    SysTick->CTRL = (SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk) | (SysTick->CTRL & ~SysTick_CTRL_CLKSOURCE_Msk);
}

void dwt_cnt_init(void) {
    // Enable the Debug interface
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    // Check if the DWT->CYCCNT is supported on the running microcontroller
    if ((DWT->CTRL >> DWT_CTRL_NOCYCCNT_Pos) & 1) {
        LOGW("Tick", "The cycle counter on the data watchpoint and tracing subsystem not supported on given target.");
        LOGW("Tick", "Profiling facilities and the delay_us(...) macro will not be available.");
        CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk;
        return;
    }

    LOGI("Tick", "The cycle counter on the data watchpoint and tracing subsystem supported.");

    // Enable the DWT->CYCCNT counter
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t ms_since_boot(void) {
    // Convert to milliseconds
    return (atomic_load_explicit(&s_tick_counter, memory_order_relaxed) * 1000U) / TICK_RATE_Hz;
}

void SysTick_Handler(void) {
    atomic_fetch_add_explicit(&s_tick_counter, 1, memory_order_relaxed);
}
