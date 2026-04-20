#include "stm32f411xe.h"

#include <stddef.h>
#include <sys/stat.h>
#include <sys/types.h>


void system_init(void) {
    // FPU settings
    SCB->CPACR |= ((3UL << (10 * 2)) | (3UL << (11 * 2)));

    // Enable HSE
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY));

    // Set flash latency, enable I and D caches, as well as enable instruction prefetching
    FLASH->ACR |= (FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_3WS | FLASH_ACR_PRFTEN);

    // Configure voltage regulator
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    PWR->CR |= PWR_CR_VOS;

    // Configure PLL
    RCC->PLLCFGR = (25 << RCC_PLLCFGR_PLLM_Pos)  |
                   (200 << RCC_PLLCFGR_PLLN_Pos) |
                   (0 << RCC_PLLCFGR_PLLP_Pos)   |
                   (RCC_PLLCFGR_PLLSRC_HSE)      |
                   (4 << RCC_PLLCFGR_PLLQ_Pos);

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

    // Update system clock
    SystemCoreClock = 100'000'000UL;
}

int _close(int) {
    return 0;
}

int _lseek(int, int, int) {
    return 0;
}

int _read(int, char*, int) {
    return 0;
}

int _write(int, char*, int) {
    return 0;
}

caddr_t _sbrk(int) {
    return 0;
}

int _fstat(int, struct stat*) {
    return 0;
}

int _isatty(int) {
    return 0;
}
