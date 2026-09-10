#include "stm32f411xe.h"
#include "utils/common.h"
#include "utils/clock.h"

#include <stdint.h>


// At startup, the HSI feeds the SYSCLK, and since there are no prescalers or divider
// active at boot, the values of the HCLK, PCLK1 and PCLK2 are equal to the HSI value
// Whereas, the audio PLL starts disabled
system_clock_t SystemCoreClockType   = HSI_DIRECT;
audio_clock_t  AudioPLLCoreClockType = AUDIO_PLL_DISABLE;

uint32_t SystemCoreClock   = HSI_VALUE_MHz * 1'000'000;
uint32_t APB1CoreClock     = HSI_VALUE_MHz * 1'000'000;
uint32_t APB2CoreClock     = HSI_VALUE_MHz * 1'000'000;
float    AudioPLLCoreClock = 0;

// Lookup tables for the HCLK and APB prescalers
const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
const uint8_t APBPrescTable[8]  = {0, 0, 0, 0, 1, 2, 3, 4};


// Update the buses' clock frequency variables
void system_core_clock_config(system_clock_t system_clock) {

    SystemCoreClockType = system_clock;
    system_core_clock_update();
}

void system_core_clock_update(void) {
    uint32_t sysclk = 0;

    // Get the SYSCLK clock source
    switch (RCC->CFGR & RCC_CFGR_SWS) {
        case RCC_CFGR_SWS_HSI:
            sysclk = HSI_VALUE_Hz;
            break;

        case RCC_CFGR_SWS_HSE:
            sysclk = HSE_VALUE_Hz;
            break;

        case RCC_CFGR_SWS_PLL:
            // Get the PLL clock source
            if ((RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) >> RCC_PLLCFGR_PLLSRC_Pos) {
                // The HSE is the PLL clock source
                const uint32_t pllm   = (RCC->PLLCFGR & RCC_PLLCFGR_PLLM) >> RCC_PLLCFGR_PLLM_Pos;
                const uint32_t plln   = (RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> RCC_PLLCFGR_PLLN_Pos;
                const uint32_t pllp   = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >> RCC_PLLCFGR_PLLP_Pos) + 1) * 2;
                const uint32_t pllvco = (HSE_VALUE_Hz / pllm) * plln;
                sysclk                = pllvco / pllp;
            } else {
                // The HSI is the PLL clock source
                const uint32_t pllm   = (RCC->PLLCFGR & RCC_PLLCFGR_PLLM) >> RCC_PLLCFGR_PLLM_Pos;
                const uint32_t plln   = (RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> RCC_PLLCFGR_PLLN_Pos;
                const uint32_t pllp   = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >> RCC_PLLCFGR_PLLP_Pos) + 1) * 2;
                const uint32_t pllvco = (HSI_VALUE_Hz / pllm) * plln;
                sysclk                = pllvco / pllp;
            }
            break;

        default:
            sysclk = HSI_VALUE_Hz;
            break;
    }

    // Compute the HCLK, APB1 and APB2 bus frequencies
    __disable_irq();
    SystemCoreClock = sysclk >> AHBPrescTable[(RCC->CFGR & RCC_CFGR_HPRE) >> RCC_CFGR_HPRE_Pos];
    APB1CoreClock   = SystemCoreClock >> APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos];
    APB2CoreClock   = SystemCoreClock >> APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos];
    __enable_irq();
}

void audio_pll_clock_config(audio_clock_t audio_clock) {
    // Disable the audio PLL
    RCC->CR &= ~RCC_CR_PLLI2SON;

    // Clear all state
    RCC->PLLI2SCFGR &= ~(RCC_PLLI2SCFGR_PLLI2SM | RCC_PLLI2SCFGR_PLLI2SN | RCC_PLLI2SCFGR_PLLI2SR);
    RCC->CFGR &= ~RCC_CFGR_I2SSRC;

    if (audio_clock == AUDIO_PLL_DISABLE) {
        AudioPLLCoreClock     = 0;
        AudioPLLCoreClockType = AUDIO_PLL_DISABLE;
        return;
    }

#ifdef USE_HSE
    const uint32_t clock_mhz = HSE_VALUE_MHz;
#else
    const uint32_t clock_mhz = HSI_VALUE_MHz;
#endif

    // Divide the HSE or HSI clock by its value in MHz to get a Vco of 1MHz regardless of its value
    RCC->PLLI2SCFGR |= (clock_mhz << RCC_PLLI2SCFGR_PLLI2SM_Pos) | // PLLI2SM of the main PLL: Divides either the HSE or the HSI to get a 1MHz Vco
                       (384UL << RCC_PLLI2SCFGR_PLLI2SN_Pos) |     // PLLI2SN of 384: Multiplies Vco by 384 to get 384MHz
                       (5UL << RCC_PLLI2SCFGR_PLLI2SR_Pos);        // PLLI2SR of 5: Divides the 384MHz Vco by 5 to get us 76.8MHz

    // Enable the audio PLL
    RCC->CR |= RCC_CR_PLLI2SON;
    while (!(RCC->CR & RCC_CR_PLLI2SRDY));

    __DSB();
    __ISB();

    // Use the internal I2S PLL for the I2S peripherals
    RCC->CFGR &= ~RCC_CFGR_I2SSRC;

    AudioPLLCoreClockType = audio_clock;
    audio_pll_clock_update();
}

void audio_pll_clock_update() {
    AudioPLLCoreClock = 0;
}
