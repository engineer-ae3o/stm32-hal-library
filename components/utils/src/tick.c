#include "stm32f411xe.h"
#include "utils/common.h"
#include "utils/clock.h"
#include "utils/tick.h"
#include "utils/log.h"

#include <stdatomic.h>


static atomic_uint s_tick_counter = 0;

void systick_init(void) {
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
    NVIC_SetPriority(SysTick_IRQn, SysTick_NVIC_IRQ_PRIORITY);
    SysTick->VAL  = 0;
    SysTick->LOAD = (((SystemCoreClock / 8) / TICK_RATE_Hz) - 1) & SysTick_LOAD_RELOAD_Msk;
    SysTick->CTRL = (SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk) | (SysTick->CTRL & ~SysTick_CTRL_CLKSOURCE_Msk);
}

void dwt_cnt_init(void) {
    LOGI("Tick", "Initializing the SysTick as the tick timer source.");

    // Check if the DWT->CYCCNT is supported on the running microcontroller
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

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

uint32_t ticks_since_boot(void) {
    return atomic_load_explicit(&s_tick_counter, memory_order_relaxed);
}

void SysTick_Handler(void) {
    atomic_fetch_add_explicit(&s_tick_counter, 1, memory_order_relaxed);
}
