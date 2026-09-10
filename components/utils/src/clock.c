#include "stm32f411xe.h"
#include "utils/common.h"
#include "utils/clock.h"

#include <stdint.h>


// These are extern declared in the CMSIS headers. Need to be defined here.
// At startup, the HSI feeds the SYSCLK, and since there are no prescalers or divider
// active at boot, the values of the HCLK, PCLK1 and PCLK2 are equal to the HSI value
// Whereas, the audio PLL starts disabled
uint32_t SystemCoreClock   = HSI_VALUE_MHZ * 1'000'000;
uint32_t APB1CoreClock     = HSI_VALUE_MHZ * 1'000'000;
uint32_t APB2CoreClock     = HSI_VALUE_MHZ * 1'000'000;
uint32_t AudioPLLCoreClock = 0;

const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
const uint8_t APBPrescTable[8]  = {0, 0, 0, 0, 1, 2, 3, 4};


// Update the buses' clock frequency variables
void system_core_clock_update(void) {
    uint32_t sysclk = 0;

    // Get the SYSCLK clock source
    switch (RCC->CFGR & RCC_CFGR_SWS) {
        case RCC_CFGR_SWS_HSI:
            sysclk = HSI_VALUE_MHZ * 1'000'000;
            break;

        case RCC_CFGR_SWS_HSE:
            sysclk = HSE_VALUE_MHZ * 1'000'000;
            break;

        case RCC_CFGR_SWS_PLL:
            // Get the PLL clock source
            if ((RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) >> RCC_PLLCFGR_PLLSRC_Pos) {
                // The HSE is the PLL clock source
                const uint32_t pllm   = (RCC->PLLCFGR & RCC_PLLCFGR_PLLM) >> RCC_PLLCFGR_PLLM_Pos;
                const uint32_t plln   = (RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> RCC_PLLCFGR_PLLN_Pos;
                const uint32_t pllp   = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >> RCC_PLLCFGR_PLLP_Pos) + 1) * 2;
                const uint32_t pllvco = (HSE_VALUE_MHZ * 1'000'000 / pllm) * plln;
                sysclk                = pllvco / pllp;
            } else {
                // The HSI is the PLL clock source
                const uint32_t pllm   = (RCC->PLLCFGR & RCC_PLLCFGR_PLLM) >> RCC_PLLCFGR_PLLM_Pos;
                const uint32_t plln   = (RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> RCC_PLLCFGR_PLLN_Pos;
                const uint32_t pllp   = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >> RCC_PLLCFGR_PLLP_Pos) + 1) * 2;
                const uint32_t pllvco = (HSI_VALUE_MHZ * 1'000'000 / pllm) * plln;
                sysclk                = pllvco / pllp;
            }
            break;

        default:
            sysclk = HSI_VALUE_MHZ * 1'000'000;
            break;
    }

    // Compute the HCLK, APB1 and APB2 bus frequencies
    __disable_irq();
    SystemCoreClock   = sysclk >> AHBPrescTable[(RCC->CFGR & RCC_CFGR_HPRE) >> RCC_CFGR_HPRE_Pos];
    APB1CoreClock     = SystemCoreClock >> APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos];
    APB2CoreClock     = SystemCoreClock >> APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos];
    AudioPLLCoreClock = 0;
    __enable_irq();
}
