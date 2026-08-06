#include "stm32f411xe.h"
#include "extra/tick.h"

#include <stdatomic.h>


// Initialize the tick timer before main runs
void tick_init(void) {
    // Configure TIM2 as our tick source
    // Enable TIM2 clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    // Configure TIM2 for 1ms interrupts at 100MHz
    // Prescaler: 100MHz / 100 = 1MHz, so a PSC of 100 - 1 = 99
    // Auto reload: 1kHz, so an ARR of 1000 - 1 = 999
    TIM2->PSC = 99;
    TIM2->ARR = 999;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->SR &= ~TIM_SR_UIF;
    TIM2->DIER |= TIM_DIER_UIE;
    TIM2->CR1 |= TIM_CR1_CEN;

    // Configure NVIC settings for TIM2
    NVIC_SetPriority(TIM2_IRQn, 15);
    NVIC_EnableIRQ(TIM2_IRQn);
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

// Polling delay accurate to the milisecond
void delay_ms(uint32_t ms) {
    uint32_t start = ticks_since_boot_ms();
    while ((ticks_since_boot_ms() - start) < ms);
}

// Polling delay accurate to the microsecond
void delay_us(uint32_t us) {
    (void)us;
}
