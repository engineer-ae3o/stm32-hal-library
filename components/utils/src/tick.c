#include "stm32f411xe.h"
#include "utils/common.h"
#include "utils/tick.h"
#include "utils/log.h"

#include <stdatomic.h>


#define TIME_PER_TICK_MS (1 * 1'000 / TICK_RATE_HZ)

// Initialize the tick timer before main runs
[[__gnu__::__constructor__]] static void tick_init(void) {
    LOGI("Tick", "Initializing TIM2 as the tick timer source.");

    // Configure TIM2 as our tick source
    // Enable TIM2 clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    // Configure TIM2 for 1ms (1kHz) interrupts at 100MHz
    // Prescaler: 100MHz / 100 = 1MHz, so a PSC of 100 - 1 = 99
    // Auto reload: we need 1kHz, so an ARR of 1000 - 1 = 999

    // If the prescaler on the APB1 bus is 1, the TIM2 clock is equal to
    // the APB1 clock, which in turn is equal to the HCLK (prescaler is 1,
    // so the APB1 bus frequency isn't divided) since the APB1 bus feeds
    // off of HCLK directly. If the prescaler is greater than 1, the TIM2
    // clock is equal to 2 times the APB1 bus clock.
    system_core_clock_update();
    const uint32_t tim2_clk_freq_hz = ((RCC->CFGR & RCC_CFGR_PPRE1) == RCC_CFGR_PPRE1_DIV1) ? APB1CoreClock : APB1CoreClock * 2;

    const uint32_t psc = (TIME_PER_TICK_MS * tim2_clk_freq_hz) - 1;
    const uint32_t arr = (TIME_PER_TICK_MS * tim2_clk_freq_hz) - 1;

    TIM2->PSC = psc;
    TIM2->ARR = arr;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->SR &= ~TIM_SR_UIF;
    TIM2->DIER |= TIM_DIER_UIE;
    TIM2->CR1 |= TIM_CR1_CEN;

    // Configure NVIC settings for TIM2
    NVIC_SetPriority(TIM2_IRQn, TIM2_NVIC_IRQ_PRIORITY);
    NVIC_EnableIRQ(TIM2_IRQn);

#ifdef USE_DWT_CYCCNT
    // Check if the DWT->CYCCNT is supported on running microcontroller
    if ((DWT->CTRL >> DWT_CTRL_NOCYCCNT_Pos) & 1U) {
        LOGW("Tick", "The cycle counter on the data watchpoint and tracing subsystem not supported on given target.");
        LOGW("Tick", "Profiling facilities and the delay_us(...) function will not be available.");
        return;
    }

    LOGI("Tick", "The cycle counter on the data watchpoint and tracing subsystem supported.");

    // Enable the DWT->CYCCNT counter
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

#endif
}

// The tick counter
static atomic_uint s_tick_counter = 0;

// Getter for the tick counter
uint32_t ticks_since_boot(void) {
    return atomic_load_explicit(&s_tick_counter, memory_order_relaxed);
}

// TIM2 irq handler to increment the tick counter every milisecond
void TIM2_IRQHandler(void) {
    if (TIM2->SR & TIM_SR_UIF) {
        // Clear the update interrupt flag and increment the tick counter
        TIM2->SR &= ~TIM_SR_UIF;
        atomic_fetch_add_explicit(&s_tick_counter, 1, memory_order_relaxed);
    }
}
