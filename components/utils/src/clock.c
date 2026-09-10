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
static const uint8_t s_ahb_presc_lut[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
static const uint8_t s_apb_presc_lut[8]  = {0, 0, 0, 0, 1, 2, 3, 4};


typedef struct {
    // General settings
    uint32_t flash_latency; // CMSIS defined macro
    uint32_t vos_scale;     // CMSIS defined macro. Ignored if the sysclk_source is not RCC_CFGR_SWS_PLL
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
static const system_clock_preset_t s_system_clock_preset_lut[] = {
    [HSE_PLL_100MHz] =
        {
            .flash_latency  = FLASH_ACR_LATENCY_3WS,       // 3 flash wait states since the SYSCLK is at the highest value
            .vos_scale      = PWR_CR_VOS_0 | PWR_CR_VOS_1, // Voltage scale of 1 since high SYSCLK frequency
            .sysclk_source  = RCC_CFGR_SWS_PLL,            // SYSCLK source is the PLL
            .pll_source     = RCC_PLLCFGR_PLLSRC_HSE,      // PLL source is the HSE
            .pllm           = HSE_VALUE_MHz,
            .plln           = 200,
            .pllp           = 0,
            .pllq           = 4,
            .ahb_prescaler  = RCC_CFGR_HPRE_DIV1,  // SystemCoreClock (HCLK) = SYSCLK
            .apb1_prescaler = RCC_CFGR_PPRE1_DIV2, // APB1 = SystemCoreClock (HCLK) / 2
            .apb2_prescaler = RCC_CFGR_PPRE2_DIV1, // APB2 = SystemCoreClock (HCLK)
        },
    [HSE_PLL_96MHz] =
        {
            .flash_latency  = FLASH_ACR_LATENCY_3WS,
            .vos_scale      = PWR_CR_VOS_0 | PWR_CR_VOS_1,
            .sysclk_source  = RCC_CFGR_SWS_PLL,
            .pll_source     = RCC_PLLCFGR_PLLSRC_HSE,
            .pllm           = 0,
            .plln           = 0,
            .pllp           = 0,
            .pllq           = 0,
            .ahb_prescaler  = RCC_CFGR_HPRE_DIV1,
            .apb1_prescaler = RCC_CFGR_PPRE1_DIV2,
            .apb2_prescaler = RCC_CFGR_PPRE2_DIV1,
        },
    [HSE_PLL_84MHz] =
        {
            .flash_latency  = FLASH_ACR_LATENCY_3WS,
            .vos_scale      = PWR_CR_VOS_0 | PWR_CR_VOS_1,
            .sysclk_source  = RCC_CFGR_SWS_PLL,
            .pll_source     = RCC_PLLCFGR_PLLSRC_HSE,
            .pllm           = 0,
            .plln           = 0,
            .pllp           = 0,
            .pllq           = 0,
            .ahb_prescaler  = RCC_CFGR_HPRE_DIV1,
            .apb1_prescaler = RCC_CFGR_PPRE1_DIV2,
            .apb2_prescaler = RCC_CFGR_PPRE2_DIV1,
        },
    [HSE_PLL_64MHz] =
        {
            .flash_latency  = FLASH_ACR_LATENCY_3WS,
            .vos_scale      = PWR_CR_VOS_0 | PWR_CR_VOS_1,
            .sysclk_source  = RCC_CFGR_SWS_PLL,
            .pll_source     = RCC_PLLCFGR_PLLSRC_HSE,
            .pllm           = 0,
            .plln           = 0,
            .pllp           = 0,
            .pllq           = 0,
            .ahb_prescaler  = RCC_CFGR_HPRE_DIV1,
            .apb1_prescaler = RCC_CFGR_PPRE1_DIV2,
            .apb2_prescaler = RCC_CFGR_PPRE2_DIV1,
        },
    [HSE_PLL_48MHz] =
        {
            .flash_latency  = FLASH_ACR_LATENCY_3WS,
            .vos_scale      = PWR_CR_VOS_0 | PWR_CR_VOS_1,
            .sysclk_source  = RCC_CFGR_SWS_PLL,
            .pll_source     = RCC_PLLCFGR_PLLSRC_HSE,
            .pllm           = 0,
            .plln           = 0,
            .pllp           = 0,
            .pllq           = 0,
            .ahb_prescaler  = RCC_CFGR_HPRE_DIV1,
            .apb1_prescaler = RCC_CFGR_PPRE1_DIV2,
            .apb2_prescaler = RCC_CFGR_PPRE2_DIV1,
        },
    [HSE_PLL_DIRECT] =
        {
            .flash_latency  = FLASH_ACR_LATENCY_0WS, // 0 flash wait states since the SYSCLK frequency is low
            .vos_scale      = 0,                     // Ignored since PLL is disabled
            .sysclk_source  = RCC_CFGR_SWS_HSE,      // SYSCLK source is the HSE
            .ahb_prescaler  = RCC_CFGR_HPRE_DIV1,    // SystemCoreClock (HCLK) = SYSCLK
            .apb1_prescaler = RCC_CFGR_PPRE1_DIV1,   // APB1 = SystemCoreClock (HCLK)
            .apb2_prescaler = RCC_CFGR_PPRE2_DIV1,   // APB2 = SystemCoreClock (HCLK)
        },
    [HSI_PLL_100MHz] =
        {
            .flash_latency  = FLASH_ACR_LATENCY_3WS,       // 3 flash wait states since the SYSCLK is at the highest value
            .vos_scale      = PWR_CR_VOS_0 | PWR_CR_VOS_1, // Voltage scale of 1 since high SYSCLK frequency
            .sysclk_source  = RCC_CFGR_SWS_PLL,            // SYSCLK source is the PLL
            .pll_source     = RCC_PLLCFGR_PLLSRC_HSI,      // PLL source is the HSI
            .pllm           = HSI_VALUE_MHz,
            .plln           = 200,
            .pllp           = 0,
            .pllq           = 4,
            .ahb_prescaler  = RCC_CFGR_HPRE_DIV1,  // SystemCoreClock (HCLK) = SYSCLK
            .apb1_prescaler = RCC_CFGR_PPRE1_DIV2, // APB1 = SystemCoreClock (HCLK) / 2
            .apb2_prescaler = RCC_CFGR_PPRE2_DIV1, // APB2 = SystemCoreClock (HCLK)
        },
    [HSI_PLL_96MHz] =
        {
            .flash_latency  = FLASH_ACR_LATENCY_3WS,
            .vos_scale      = PWR_CR_VOS_0 | PWR_CR_VOS_1,
            .sysclk_source  = RCC_CFGR_SWS_PLL,
            .pll_source     = RCC_PLLCFGR_PLLSRC_HSE,
            .pllm           = 0,
            .plln           = 0,
            .pllp           = 0,
            .pllq           = 0,
            .ahb_prescaler  = RCC_CFGR_HPRE_DIV1,
            .apb1_prescaler = RCC_CFGR_PPRE1_DIV2,
            .apb2_prescaler = RCC_CFGR_PPRE2_DIV1,
        },
    [HSI_PLL_84MHz] =
        {
            .flash_latency  = FLASH_ACR_LATENCY_3WS,
            .vos_scale      = PWR_CR_VOS_0 | PWR_CR_VOS_1,
            .sysclk_source  = RCC_CFGR_SWS_PLL,
            .pll_source     = RCC_PLLCFGR_PLLSRC_HSE,
            .pllm           = 0,
            .plln           = 0,
            .pllp           = 0,
            .pllq           = 0,
            .ahb_prescaler  = RCC_CFGR_HPRE_DIV1,
            .apb1_prescaler = RCC_CFGR_PPRE1_DIV2,
            .apb2_prescaler = RCC_CFGR_PPRE2_DIV1,
        },
    [HSI_PLL_64MHz] =
        {
            .flash_latency  = FLASH_ACR_LATENCY_3WS,
            .vos_scale      = PWR_CR_VOS_0 | PWR_CR_VOS_1,
            .sysclk_source  = RCC_CFGR_SWS_PLL,
            .pll_source     = RCC_PLLCFGR_PLLSRC_HSE,
            .pllm           = 0,
            .plln           = 0,
            .pllp           = 0,
            .pllq           = 0,
            .ahb_prescaler  = RCC_CFGR_HPRE_DIV1,
            .apb1_prescaler = RCC_CFGR_PPRE1_DIV2,
            .apb2_prescaler = RCC_CFGR_PPRE2_DIV1,
        },
    [HSI_PLL_48MHz] =
        {
            .flash_latency  = FLASH_ACR_LATENCY_3WS,
            .vos_scale      = PWR_CR_VOS_0 | PWR_CR_VOS_1,
            .sysclk_source  = RCC_CFGR_SWS_PLL,
            .pll_source     = RCC_PLLCFGR_PLLSRC_HSE,
            .pllm           = 0,
            .plln           = 0,
            .pllp           = 0,
            .pllq           = 0,
            .ahb_prescaler  = RCC_CFGR_HPRE_DIV1,
            .apb1_prescaler = RCC_CFGR_PPRE1_DIV2,
            .apb2_prescaler = RCC_CFGR_PPRE2_DIV1,
        },
    [HSI_PLL_DIRECT] =
        {
            .flash_latency  = FLASH_ACR_LATENCY_0WS, // 0 flash wait states since the SYSCLK frequency is low
            .vos_scale      = 0,                     // Ignored since PLL is disabled
            .sysclk_source  = RCC_CFGR_SWS_HSI,      // SYSCLK source is the HSI
            .ahb_prescaler  = RCC_CFGR_HPRE_DIV1,    // SystemCoreClock (HCLK) = SYSCLK
            .apb1_prescaler = RCC_CFGR_PPRE1_DIV1,   // APB1 = SystemCoreClock (HCLK)
            .apb2_prescaler = RCC_CFGR_PPRE2_DIV1,   // APB2 = SystemCoreClock (HCLK)
        },
};

static const audio_clock_preset_t s_audio_clock_preset_lut[] = {
    [AUDIO_PLL_DISABLE]  = {.disable = true, .plln = 0, .pllr = 0},
    [AUDIO_PLL_76_8MHz]  = {.disable = false, .plln = 384, .pllr = 5},
    [AUDIO_PLL_135_5MHz] = {.disable = false, .plln = 271, .pllr = 2},
    [AUDIO_PLL_151MHz]   = {.disable = false, .plln = 302, .pllr = 2},
    [AUDIO_PLL_172MHz]   = {.disable = false, .plln = 344, .pllr = 2},
};

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
            // Set the PLL source
            if (preset->pll_source == RCC_PLLCFGR_PLLSRC_HSI) {
                // Enable the HSI
                RCC->CR |= RCC_CR_HSION;
                while (!(RCC->CR & RCC_CR_HSIRDY));
            } else if (preset->pll_source == RCC_PLLCFGR_PLLSRC_HSE) {
                // Enable the HSE
                RCC->CR |= RCC_CR_HSEON;
                while (!(RCC->CR & RCC_CR_HSERDY));
            } else {
                ASSERT(0);
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

    if (preset->sysclk_source == RCC_CFGR_SWS_HSE || preset->pll_source == RCC_PLLCFGR_PLLSRC_HSE) {
        // The HSE is used either as the SYSCLK source directly or as the PLL input, so we disable the HSI since not in use to save power
        RCC->CR &= ~RCC_CR_HSION;
        while (RCC->CR & RCC_CR_HSIRDY);

        // Enable the Clock Security System to monitor the HSE
        RCC->CR |= RCC_CR_CSSON;
    } else if (preset->sysclk_source == RCC_CFGR_SWS_HSI || preset->pll_source == RCC_PLLCFGR_PLLSRC_HSI) {
        // The HSI is used either as the SYSCLK source directly or as the PLL input, so we disable the HSE since not in use to save power
        RCC->CR &= ~RCC_CR_HSEON;
        while (RCC->CR & RCC_CR_HSERDY);
    } else {
        ASSERT(0);
    }

    __enable_irq();

    // Update the global variables tracking the system clock and reconfigure the audio PLL to its old state since it was disabled
    system_core_clock_update();
    audio_pll_clock_config(AudioPLLCoreClockType);
}

static inline void audio_pll_clock_config_preset(const audio_clock_preset_t* preset) {
    __disable_irq();

    // Disable the audio PLL and clear all state
    RCC->CR &= ~RCC_CR_PLLI2SON;
    RCC->PLLI2SCFGR &= ~(RCC_PLLI2SCFGR_PLLI2SM | RCC_PLLI2SCFGR_PLLI2SN | RCC_PLLI2SCFGR_PLLI2SR);

    if (preset->disable) {
        return;
    }

    // We use a PLLM value to get us a Vco of 1MHz regardless if the HSI or HSE is used.
    uint32_t pllm = 0;

    switch (RCC->CFGR & RCC_CFGR_SWS) {
        // If the SYSCLK source is either from the HSE or HSI directly, this implies the PLLSRC bit
        // was not set and/or ignored, since the PLLSRC bit gets set from the configuration process of
        // the PLL(s), which is bypassed if the HSE or HSI feeds the SYSCLK directly; so we cannot trust
        // its current value. So we set the PLLSRC to be whatever is driving the SYSCLK currently.
        case RCC_CFGR_SWS_HSI:
            RCC->CFGR = (RCC->CFGR & ~RCC_PLLCFGR_PLLSRC) | RCC_PLLCFGR_PLLSRC_HSI;
            pllm      = HSI_VALUE_MHz;
            break;

        case RCC_CFGR_SWS_HSE:
            RCC->CFGR = (RCC->CFGR & ~RCC_PLLCFGR_PLLSRC) | RCC_PLLCFGR_PLLSRC_HSE;
            pllm      = HSE_VALUE_MHz;
            break;

        // But if the PLL is the SYSCLK source, the PLLSRC bit has already been explicitly before now.
        // So we read it to get the source, and set the pllm to the value in MHz of that source's clock
        case RCC_CFGR_SWS_PLL:
            pllm = ((RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) == RCC_PLLCFGR_PLLSRC_HSE) ? HSE_VALUE_MHz : HSI_VALUE_MHz;
            break;

        default:
            pllm = HSI_VALUE_MHz;
            break;
    }

    RCC->PLLI2SCFGR |=
        (pllm << RCC_PLLI2SCFGR_PLLI2SM_Pos) | (preset->plln << RCC_PLLI2SCFGR_PLLI2SN_Pos) | (preset->pllr << RCC_PLLI2SCFGR_PLLI2SR_Pos);

    // Enable the audio PLL
    RCC->CR |= RCC_CR_PLLI2SON;
    while (!(RCC->CR & RCC_CR_PLLI2SRDY));

    // Use the just configured I2S PLL for the I2S peripheral(s)
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
            if ((RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) == RCC_PLLCFGR_PLLSRC_HSE) {
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
    const uint32_t source = (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) == RCC_PLLCFGR_PLLSRC_HSE ? HSE_VALUE_Hz : HSI_VALUE_Hz;
    const uint32_t pllm   = (RCC->PLLI2SCFGR & RCC_PLLI2SCFGR_PLLI2SM) >> RCC_PLLI2SCFGR_PLLI2SM_Pos;
    const uint32_t plln   = (RCC->PLLI2SCFGR & RCC_PLLI2SCFGR_PLLI2SN) >> RCC_PLLI2SCFGR_PLLI2SN_Pos;
    const uint32_t pllr   = (RCC->PLLI2SCFGR & RCC_PLLI2SCFGR_PLLI2SR) >> RCC_PLLI2SCFGR_PLLI2SR_Pos;
    AudioPLLCoreClock     = ((source / pllm) * plln) / pllr;
}
