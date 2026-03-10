#include "stm32f411xe.h"
#include "system_stm32f4xx.h"

#include "gpio.h"
#include "timer.h"

#include <stdint.h>


#define LED_PIN 13
#define LED_PORT GPIOC

#define BUTTON_PIN 0
#define BUTTON_PORT GPIOA

#define TIMER_PORT TIM2


// Defined here and used in the startup code
void system_init(void) {
    // FPU settings
    SCB->CPACR |= ((3UL << (10 * 2)) | (3UL << (11 * 2)));

    // Enable HSE
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY));

    // Set flash latency
    FLASH->ACR |= FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_3WS;

    // Configure voltage regulator
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    PWR->CR |= PWR_CR_VOS;

    // Configure PLL
    RCC->PLLCFGR = (25 << RCC_PLLCFGR_PLLM_Pos) |
                   (200 << RCC_PLLCFGR_PLLN_Pos) |
                   (0 << RCC_PLLCFGR_PLLP_Pos) |
                   RCC_PLLCFGR_PLLSRC_HSE |
                   (4 << 24);

    // Enable PLL
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));

    // Bus prescaler
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;

    // Switch to PLL
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
}

void TIM2_IRQHandler(void) {
    if (TIMER_PORT->SR & TIM_SR_TIF) {
        TIMER_PORT->SR &= ~TIM_SR_TIF;
        gpio_toggle(LED_PORT, LED_PIN);
    }
}

void EXTI0_IRQHandler(void) {
    if (EXTI->PR & (1UL << BUTTON_PIN)) {
        EXTI->PR |= (1UL << BUTTON_PIN);
        gpio_toggle(LED_PORT, LED_PIN);
    }
}

int main(void) {

    const uint16_t prescaler = 25;
    const uint32_t reload_val = 4'000'000;
    
    gpiox_clk_enable(LED_PORT);
    gpio_set_output(LED_PORT, LED_PIN);
    gpio_set(LED_PORT, LED_PIN);
    
    gpio_enable_sys_clk();
    gpiox_clk_enable(BUTTON_PORT);
    gpio_set_input(BUTTON_PORT, BUTTON_PIN);
    gpio_set_interupt(BUTTON_PORT, BUTTON_PIN, 0b10);

    timer_init(TIMER_PORT, prescaler);
    timer_set_reload_value(TIMER_PORT, reload_val);
    timer_start(TIMER_PORT);

    NVIC_EnableIRQ(EXTI0_IRQn);
    NVIC_EnableIRQ(TIM2_IRQn);

    while (1) {
        __WFI();
    }
}
