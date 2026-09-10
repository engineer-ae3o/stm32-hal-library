#include "stm32f411xe.h"
#include "utils/common.h"
#include "utils/clock.h"

#include <stdint.h>


// At startup, the HSI feeds the SYSCLK, and since there are no prescalers or divider
// active at boot, the values of the HCLK, PCLK1 and PCLK2 are equal to the HSI value
// Whereas, the audio PLL is disabled after reset.
volatile system_clock_t SystemCoreClockType   = HSI_PLL_DIRECT;
volatile audio_clock_t  AudioPLLCoreClockType = AUDIO_PLL_DISABLE;

volatile uint32_t SystemCoreClock   = HSI_VALUE_Hz;
volatile uint32_t APB1CoreClock     = HSI_VALUE_Hz;
volatile uint32_t APB2CoreClock     = HSI_VALUE_Hz;
volatile uint32_t AudioPLLCoreClock = 0;

// Lookup tables for the HCLK and APB prescalers
const static uint8_t s_ahb_presc_lut[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
const static uint8_t s_apb_presc_lut[8]  = {0, 0, 0, 0, 1, 2, 3, 4};


typedef struct {
    // General settings
    uint32_t flash_latency; // CMSIS defined macro
    uint32_t vos_scale;     // CMSIS defined macro
    uint32_t sysclk_source; // CMSIS defined macro
    // These only have any meaning if sysclk_source is RCC_CFGR_SWS_PLL
    uint32_t pll_source; // CMSIS defined macro
    uint32_t pllm;       // Raw calculated number
    uint32_t plln;       // Raw calculated number
    uint32_t pllp;       // Raw calculated number
    uint32_t pllq;       // Raw calculated number
    // Bus prescalers
    uint32_t ahb_prescaler;  // CMSIS defined macro
    uint32_t apb1_prescaler; // CMSIS defined macro
    uint32_t apb2_prescaler; // CMSIS defined macro
} system_clock_preset_t;

typedef struct {
    bool     disable;
    uint32_t plln; // Raw calculated number
    uint32_t pllr; // Raw calculated number
} audio_clock_preset_t;

// Tables mapping the different clock frequencies and sources to their respective configuration data
const static system_clock_preset_t s_system_clock_preset_lut[] = {
    [HSE_PLL_100MHz] = {},
    [HSE_PLL_96MHz]  = {},
    [HSE_PLL_84MHz]  = {},
    [HSE_PLL_64MHz]  = {},
    [HSE_PLL_48MHz]  = {},
    [HSE_PLL_DIRECT] = {},
    [HSI_PLL_100MHz] = {},
    [HSI_PLL_96MHz]  = {},
    [HSI_PLL_84MHz]  = {},
    [HSI_PLL_64MHz]  = {},
    [HSI_PLL_48MHz]  = {},
    [HSI_PLL_DIRECT] = {},
};

const static audio_clock_preset_t s_audio_clock_preset_lut[] = {
    [AUDIO_PLL_DISABLE]  = {},
    [AUDIO_PLL_76_8MHz]  = {},
    [AUDIO_PLL_135_5MHz] = {},
    [AUDIO_PLL_151MHz]   = {},
    [AUDIO_PLL_195_5MHz] = {},
};

// Helpers
static inline void system_core_clock_config_preset(const system_clock_preset_t* preset) {
    __disable_irq();

    // Set the required flash latency
    FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY) | preset->flash_latency;
    __DSB();
    __ISB();

    // Disable the main PLL before configuring any clock
    RCC->CR &= ~(RCC_CR_PLLON | RCC_CR_PLLI2SON);
    while (RCC->CR & (RCC_CR_PLLRDY | RCC_CR_PLLI2SRDY));

    // Configure the voltage regulator. Requires that the main PLL be disabled
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    __DSB();

    // Set the required voltage regulator scale
    PWR->CR = (PWR->CR & ~PWR_CR_VOS) | preset->vos_scale;

    // Set the bus prescalers
    RCC->CFGR &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);
    RCC->CFGR |= (preset->ahb_prescaler | preset->apb1_prescaler | preset->apb2_prescaler);

    switch (preset->sysclk_source) {
        case RCC_CFGR_SWS_HSI:
            // Enable the HSI
            RCC->CR |= RCC_CR_HSION;
            while (!(RCC->CR & RCC_CR_HSIRDY));

            // Use the HSI as the SYSCLK source
            RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_HSI;
            while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI);
            break;

        case RCC_CFGR_SWS_HSE:
            // Enable the HSE
            RCC->CR |= RCC_CR_HSEON;
            while (!(RCC->CR & RCC_CR_HSERDY));

            // Use the HSE as the SYSCLK source
            RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_HSE;
            while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSE);
            break;

        case RCC_CFGR_SWS_PLL:
            if (preset->pll_source == RCC_PLLCFGR_PLLSRC_HSI) {
                // Enable the HSI
                RCC->CR |= RCC_CR_HSION;
                while (!(RCC->CR & RCC_CR_HSIRDY));
            } else {
                // Enable the HSE
                RCC->CR |= RCC_CR_HSEON;
                while (!(RCC->CR & RCC_CR_HSERDY));
            }

            // Configure the PLL to provide the SYSCLK source, derived from the HSI
            RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLM | RCC_PLLCFGR_PLLN | RCC_PLLCFGR_PLLP | RCC_PLLCFGR_PLLQ | RCC_PLLCFGR_PLLSRC);
            RCC->PLLCFGR |= (preset->pllm << RCC_PLLCFGR_PLLM_Pos) | (preset->plln << RCC_PLLCFGR_PLLN_Pos) | (preset->pllp << RCC_PLLCFGR_PLLP_Pos) |
                            (preset->pllq << RCC_PLLCFGR_PLLQ_Pos) | (preset->pll_source);

            // Enable the PLL
            RCC->CR |= RCC_CR_PLLON;
            while (!(RCC->CR & RCC_CR_PLLRDY));

            // Use the PLL as the SYSCLK source
            RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
            while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
            break;

        default:
            // Should be unreachable
            ASSERT(0);
            break;
    }

    __DSB();
    __ISB();

    if (preset->sysclk_source == RCC_CFGR_SWS_HSI || preset->pll_source == RCC_PLLCFGR_PLLSRC_HSI) {
        // The HSI is used either as the SYSCLK source directly or as the PLL input, so we disable the HSE since not in use
        RCC->CR &= ~RCC_CR_HSEON;
        while (RCC->CR & RCC_CR_HSERDY);
    } else {
        // The HSE is used either as the SYSCLK source directly or as the PLL input, so we disable the HSI since not in use
        RCC->CR &= ~RCC_CR_HSION;
        while (RCC->CR & RCC_CR_HSIRDY);

        // Enable the Clock Security System to monitor the HSE
        RCC->CR |= RCC_CR_CSSON;
    }

    __enable_irq();

    // Update the global variables tracking the system clock and reconfigure the audio PLL to its old state
    system_core_clock_update();
    audio_pll_clock_config(AudioPLLCoreClockType);
}

static inline void audio_pll_clock_config_preset(const audio_clock_preset_t* preset) {
    __disable_irq();

    // Disable the audio PLL
    RCC->CR &= ~RCC_CR_PLLI2SON;

    // Clear all state
    RCC->PLLI2SCFGR &= ~(RCC_PLLI2SCFGR_PLLI2SM | RCC_PLLI2SCFGR_PLLI2SN | RCC_PLLI2SCFGR_PLLI2SR);
    RCC->CFGR &= ~RCC_CFGR_I2SSRC;

    if (preset->disable) {
        return;
    }

    // We use a PLLM value to get us a Vco of 1MHz regardless of whether the HSI or HSE is used
    uint32_t clock_mhz = 0;

    // Get the SYSCLK source.
    switch (RCC->CFGR & RCC_CFGR_SWS) {
        case RCC_CFGR_SWS_HSI:
            clock_mhz = HSI_VALUE_MHz;
            break;

        case RCC_CFGR_SWS_HSE:
            clock_mhz = HSE_VALUE_MHz;
            break;

        case RCC_CFGR_SWS_PLL:
            // Get the PLL clock source
            if ((RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) >> RCC_PLLCFGR_PLLSRC_Pos) {
                // The HSE is the PLL clock source
                clock_mhz = HSE_VALUE_MHz;
            } else {
                // The HSI is the PLL clock source
                clock_mhz = HSI_VALUE_MHz;
            }
            break;

        default:
            clock_mhz = HSI_VALUE_MHz;
            break;
    }

    // Divide the HSE or HSI clock by its value in MHz to get a Vco of 1MHz regardless of its value
    RCC->PLLI2SCFGR |=
        (clock_mhz << RCC_PLLI2SCFGR_PLLI2SM_Pos) | (preset->plln << RCC_PLLI2SCFGR_PLLI2SN_Pos) | (preset->pllr << RCC_PLLI2SCFGR_PLLI2SR_Pos);

    // Enable the audio PLL
    RCC->CR |= RCC_CR_PLLI2SON;
    while (!(RCC->CR & RCC_CR_PLLI2SRDY));

    // Use the internal I2S PLL for the I2S peripherals
    RCC->CFGR &= ~RCC_CFGR_I2SSRC;

    __DSB();
    __ISB();

    __enable_irq();
    audio_pll_clock_update();
}


// System Clock Configuration
void system_core_clock_config(system_clock_t clock) {
    system_core_clock_config_preset(&s_system_clock_preset_lut[clock]);
    SystemCoreClockType = clock;
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
    SystemCoreClock = sysclk >> s_ahb_presc_lut[(RCC->CFGR & RCC_CFGR_HPRE) >> RCC_CFGR_HPRE_Pos];
    APB1CoreClock   = SystemCoreClock >> s_apb_presc_lut[(RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos];
    APB2CoreClock   = SystemCoreClock >> s_apb_presc_lut[(RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos];
    __enable_irq();
}


// Audio PLL Configuration
void audio_pll_clock_config(audio_clock_t clock) {
    audio_pll_clock_config_preset(&s_audio_clock_preset_lut[clock]);
    AudioPLLCoreClockType = clock;
}

void audio_pll_clock_update(void) {
    AudioPLLCoreClock = 0;
}
