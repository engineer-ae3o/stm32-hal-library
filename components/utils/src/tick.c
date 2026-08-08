#include "stm32f411xe.h"
#include "utils/common.h"
#include "utils/tick.h"
#include "utils/log.h"

#include <stdatomic.h>


// Initialize the tick timer before main runs
__attribute__((constructor)) static void tick_init(void) {
    LOGI_ISR("Initializing TIM2 as the tick timer source");

    // Configure TIM2 as our tick source
    // Enable TIM2 clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    // Configure TIM2 for 1ms (1kHz) interrupts at 100MHz
    // Prescaler: 100MHz / 100 = 1MHz, so a PSC of 100 - 1 = 99
    // Auto reload: we need 1kHz, so an ARR of 1000 - 1 = 999
    TIM2->PSC = 99;
    TIM2->ARR = 999;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->SR &= ~TIM_SR_UIF;
    TIM2->DIER |= TIM_DIER_UIE;
    TIM2->CR1 |= TIM_CR1_CEN;

    // Configure NVIC settings for TIM2
    NVIC_SetPriority(TIM2_IRQn, 15);
    NVIC_EnableIRQ(TIM2_IRQn);

#ifdef USE_DWT_CYCCNT
    const bool is_dwt_cycnt_supported = ((DWT->CTRL >> 25) & 1U) == 0;
    if (!is_dwt_cycnt_supported) {
        LOGW_ISR("The cycle counter on the data watchpoint and tracing subsystem not supported on given target.");
        LOGI_ISR("Profiling facilities and delay_us(...) will not be available.");
        return;
    } else {
        LOGI_ISR("The cycle counter on the data watchpoint and tracing subsystem fully supported.");
    }

    // Enable the DWT->CYCCNT counter
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
#endif
}

// The tick counter
static atomic_uint s_tick_counter_ms = 0;

// Getter for the tick counter
uint32_t ticks_since_boot_ms(void) {
    return atomic_load_explicit(&s_tick_counter_ms, memory_order_relaxed);
}

// TIM2 irq handler to increment the tick counter every milisecond
void TIM2_IRQHandler(void) {
    if (TIM2->SR & TIM_SR_UIF) {
        // Clear the update interrupt flag and increment the tick counter
        TIM2->SR &= ~TIM_SR_UIF;
        atomic_fetch_add_explicit(&s_tick_counter_ms, 1, memory_order_relaxed);
    }
}
