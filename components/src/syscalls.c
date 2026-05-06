#include "stm32f411xe.h"
#include "common.h"

#include <errno.h>
#include <stddef.h>
#include <sys/stat.h>
#include <sys/types.h>


void system_init(void) {
    // Enable the FPU
    SCB->CPACR |= ((3UL << (10 * 2)) | (3UL << (11 * 2)));

    // Barriers to ensure all memory accesses are completed
    __DSB();
    __ISB();

    // Set flash latency, enable I and D caches, as well as enable instruction prefetching
    FLASH->ACR |= (FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_3WS | FLASH_ACR_PRFTEN);

    // Configure voltage regulator
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    __DSB();
    PWR->CR |= PWR_CR_VOS;
    while (!(PWR->CSR & PWR_CSR_VOSRDY));

    // Disable the PLLs
    RCC->CR &= ~(RCC_CR_PLLON | RCC_CR_PLLI2SON);
    while (RCC->CR & RCC_CR_PLLRDY);

#ifdef USE_HSE
    // Enable HSE
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY));

    // Configure PLL
    RCC->PLLCFGR = (HSE_VALUE_MHZ << RCC_PLLCFGR_PLLM_Pos) |
                   (200 << RCC_PLLCFGR_PLLN_Pos)           |
                   (0 << RCC_PLLCFGR_PLLP_Pos)             |
                   (RCC_PLLCFGR_PLLSRC_HSE)                |
                   (4 << RCC_PLLCFGR_PLLQ_Pos);
#else
    // Enable HSI
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY));

    // Configure PLL
    RCC->PLLCFGR = (HSI_VALUE_MHZ << RCC_PLLCFGR_PLLM_Pos) |
                   (200 << RCC_PLLCFGR_PLLN_Pos)           |
                   (0 << RCC_PLLCFGR_PLLP_Pos)             |
                   (RCC_PLLCFGR_PLLSRC_HSI)                |
                   (4 << RCC_PLLCFGR_PLLQ_Pos);
#endif

    // Bus prescaler
    RCC->CFGR |= (RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PPRE2_DIV1);

    // Enable the PLL
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));

    // Switch to PLL
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

    __DSB();
    __ISB();

    // Update system clock
    SystemCoreClock = CLOCK_SPEED_HZ;
}

const char* hal_err_to_string(hal_err_t err) {
    switch (err) {
        case HAL_OK:                      return "HAL_OK";
        case HAL_FAIL:                    return "HAL_FAIL";
        case HAL_INVALID_ARG:             return "HAL_INVALID_ARG";
        case HAL_INVALID_STATE:           return "HAL_INVALID_STATE";
        case HAL_TIMEOUT:                 return "HAL_TIMEOUT";
        case HAL_TX_ERROR:                return "HAL_TX_ERROR";
        case HAL_RX_ERROR:                return "HAL_RX_ERROR";
        case HAL_I2C_DEVICE_NOT_FOUND:    return "HAL_I2C_DEVICE_NOT_FOUND";
        case HAL_I2C_ARBITRATION_LOST:    return "HAL_I2C_ARBITRATION_LOST";
        case HAL_SPI_TXE_FAILED_TO_SET:   return "HAL_SPI_TXE_FAILED_TO_SET";
        case HAL_SPI_BSY_FAILED_TO_CLEAR: return "HAL_SPI_BSY_FAILED_TO_CLEAR";
        case HAL_UART_TC_FAILED_TO_SET:   return "HAL_UART_TC_FAILED_TO_SET";
        case HAL_DMA_TE:                  return "HAL_DMA_TE";
        case HAL_DMA_DME:                 return "HAL_DMA_DME";
        case HAL_DMA_HTE:                 return "HAL_DMA_HTE";
        case HAL_DMA_ERR_UNKNOWN:         return "HAL_DMA_ERR_UNKNOWN";
        default:                          return "";
    }
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
    errno = ENOMEM;
    return (caddr_t)-1;;
}

int _fstat(int, struct stat*) {
    return 0;
}

int _isatty(int) {
    return 0;
}
