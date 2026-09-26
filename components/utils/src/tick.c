#include "stm32f411xe.h"
#include "utils/common.h"
#include "utils/clock.h"
#include "utils/tick.h"
#include "utils/log.h"


void systick_init(void) {
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
    const uint32_t system_core_clock = get_system_core_clock();

    static bool first_init = true;
    if (gnu_unlikely(first_init)) {
        LOGI("Tick",
             "Initializing the SysTick timer as the tick timer source with an HCLK of %luMHz from the %s",
             system_core_clock / 1'000'000U,
             is_sysclk_on_hse() ? "HSE" : "HSI");
        NVIC_SetPriority(SysTick_IRQn, SysTick_NVIC_IRQ_PRIORITY);
        first_init = false;
    } else {
        LOGI("Tick",
             "Reinitializing the SysTick timer with an HCLK of %luMHz from the %s",
             system_core_clock / 1'000'000U,
             is_sysclk_on_hse() ? "HSE" : "HSI");
    }

    SysTick->VAL  = 0;
    SysTick->LOAD = (((system_core_clock / 8) / TICK_RATE_Hz) - 1) & SysTick_LOAD_RELOAD_Msk;
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

static volatile uint64_t s_tick_counter = 0;

uint64_t ms_since_boot(void) {
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const uint64_t ticks = s_tick_counter;
    __set_PRIMASK(primask);
    return (ticks * 1000ULL) / TICK_RATE_Hz;
}

void SysTick_Handler(void) {
    s_tick_counter++;
}
