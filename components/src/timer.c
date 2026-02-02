#include "timer.h"


void timer_init(void) {
    // Enable TIM2 clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    // Disable timer counter
    TIM2->CR1 &= ~TIM_CR1_CEN;
    // Buffer ARR
    TIM2->CR1 |= TIM_CR1_ARPE;
    // Set timer in edge aligned mode
    TIM2->CR1 &= ~TIM_CR1_CMS;
    // Set timer as an upcounter
    TIM2->CR1 &= ~TIM_CR1_DIR;
    // Disable one pulse mode
    TIM2->CR1 &= ~TIM_CR1_OPM;
    // Update Event Settings
    TIM2->CR1 &= ~(TIM_CR1_URS | TIM_CR1_OPM);
}

void timer_set_reload_value(uint32_t value) {
    TIM2->ARR = value;
}

void timer_start(void) {
    TIM2->CR1 |= TIM_CR1_CEN;
}

void timer_end(void) {
    TIM2->CR1 &= ~TIM_CR1_CEN;
}
