#include "stm32f411xe.h"
#include "system_stm32f4xx.h"

#include "gpio.h"

#include <stdint.h>


void sys_clk_init(void) {

    // Enable HSE
    RCC->CR |= RCC_CR_HSERDY;
    while (!(RCC->CR & RCC_CR_HSERDY));

    // Configure voltage regulator
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    PWR->CR |= PWR_CR_VOS;

    // Set flash latency
    FLASH->ACR |= FLASH_ACR_LATENCY_3WS;

    // Configure PLL
    RCC->PLLCFGR = (25 << RCC_PLLCFGR_PLLM_Pos) |
                   (200 << RCC_PLLCFGR_PLLN_Pos) |
                   (0 << RCC_PLLCFGR_PLLP_Pos) |
                   RCC_PLLCFGR_PLLSRC_HSE;

    // Bus prescaler
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;

    // Enable PLL
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));

    // Switch to PLL
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while (!(RCC->CFGR & RCC_CFGR_SW_PLL));

    // Update clock value
    SystemCoreClock = 100'000'000;

}

// Defined here and used in startup code
void SystemInit(void) {
    // FPU settings
    SCB->CPACR |= ((3UL << (10 * 2)) | (3UL << (11 * 2)));
}

int main() {

    sys_clk_init();
    gpio_init();

    while (1) {

    }
}
