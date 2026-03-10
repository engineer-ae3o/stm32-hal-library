#include "timer.h"

void timer_init(TIM_TypeDef* timer, uint16_t prescaler) {
    // Enable timer clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    // Disable timer counter
    timer->CR1 &= ~TIM_CR1_CEN;
    // Buffer ARR
    timer->CR1 |= TIM_CR1_ARPE;
    // Set timer in edge aligned mode
    timer->CR1 &= ~TIM_CR1_CMS;
    // Set timer as an upcounter
    timer->CR1 &= ~TIM_CR1_DIR;
    // Disable one pulse mode
    timer->CR1 &= ~TIM_CR1_OPM;
    // Update Event Settings
    timer->CR1 &= ~(TIM_CR1_URS | TIM_CR1_OPM);
    // Enable interrupts
    timer->DIER |= TIM_DIER_TIE;

    // Set prescaler
    timer->PSC = --prescaler;
}

void timer_set_reload_value(TIM_TypeDef* timer, uint32_t value) {
    timer->ARR = --value;
}

void timer_start(TIM_TypeDef* timer) {
    timer->CR1 |= TIM_CR1_CEN;
}

void timer_end(TIM_TypeDef* timer) {
    timer->CR1 &= ~TIM_CR1_CEN;
}
