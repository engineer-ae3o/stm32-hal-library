#include "stm32f411xe.h"
#include "utils/common.h"
#include "utils/board.h"
#include "utils/clock.h"
#include "utils/tick.h"

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
    uint32_t flash_latency; // CMSIS defined macro. See FLASH_ACR_LATENCY_xxx
    uint32_t vos_scale;     // CMSIS defined macro. See PWR_CR_VOS_xxx. Ignored if sysclk_source is RCC_CFGR_SWS_HSI or RCC_CFGR_SWS_HSE
    uint32_t sysclk_source; // CMSIS defined macro. See RCC_CFGR_SWS_xxx
    // These only have any meaning if sysclk_source is RCC_CFGR_SWS_PLL
    uint32_t pll_source; // CMSIS defined macro
    uint32_t pllm;       // Regular number. Allowed range is 2 - 63
    uint32_t plln;       // Regular number. Allowed range is 50 - 432
    uint32_t pllp;       // Regular number. Allowed values are 2, 4, 6 and 8 (encoded as 0b00, 0b01, 0b10 and 0b11 respectively)
    uint32_t pllq;       // Regular number. Allowed range is 2 - 15
    // Bus prescalers
    uint32_t ahb_prescaler;  // CMSIS defined macro. See RCC_CFGR_HPRE_DIVx
    uint32_t apb1_prescaler; // CMSIS defined macro. See RCC_CFGR_PPRE1_DIVx
    uint32_t apb2_prescaler; // CMSIS defined macro. See RCC_CFGR_PPRE2_DIVx
} system_clock_preset_t;

typedef struct {
    bool     disable; // Set to power down the audio PLL
    uint32_t plln;    // Regular number. Allowed range is 50 - 432
    uint32_t pllr;    // Regular number. Allowed range is 2 - 7
} audio_clock_preset_t;

// Tables mapping the different clock frequencies and sources to their respective configuration data
static const system_clock_preset_t s_system_clock_preset_lut[] = {
    [HSE_PLL_100MHz] =
        {
            .flash_latency  = FLASH_ACR_LATENCY_3WS,  // 3 flash wait states since the SYSCLK is at the highest value
            .vos_scale      = 0b11 << PWR_CR_VOS_Pos, // Voltage scale of 1 since high SYSCLK frequency
            .sysclk_source  = RCC_CFGR_SWS_PLL,       // SYSCLK source is the PLL
            .pll_source     = RCC_PLLCFGR_PLLSRC_HSE, // PLL source is the HSE
            .pllm           = HSE_VALUE_MHz,          // Divides the HSE by its value in MHz to give us 1MHz regardless of its starting value
            .plln           = 200,                    // Multiplies the resultant 1MHz by 200 to give 200MHz
            .pllp           = 0b00,                   // Divides the 200MHz by 2 to provide 100MHz for the SYSCLK. For more details, refer above
            .pllq           = 4,                      // Divides the 200MHz by 4 to provide 50MHz for the USB and SDIO clocks
            .ahb_prescaler  = RCC_CFGR_HPRE_DIV1,     // SystemCoreClock = SYSCLK = 100MHz
            .apb1_prescaler = RCC_CFGR_PPRE1_DIV2,    // APB1 = SystemCoreClock / 2 = 50MHz
            .apb2_prescaler = RCC_CFGR_PPRE2_DIV1,    // APB2 = SystemCoreClock = 100MHz
        },
    [HSE_PLL_96MHz] =
        {
            .flash_latency  = FLASH_ACR_LATENCY_3WS,  // 3 flash wait states since the SYSCLK is at a very high value
            .vos_scale      = 0b11 << PWR_CR_VOS_Pos, // Voltage scale of 1 since high SYSCLK frequency
            .sysclk_source  = RCC_CFGR_SWS_PLL,       // SYSCLK source is the PLL
            .pll_source     = RCC_PLLCFGR_PLLSRC_HSE, // PLL source is the HSE
            .pllm           = HSE_VALUE_MHz,          // Divides the HSE by its value in MHz to give us 1MHz regardless of its starting value
            .plln           = 192,                    // Multiplies the resultant 1MHz by 192 to give 192MHz
            .pllp           = 0b00,                   // Divides the 192MHz by 2 to provide 96MHz for the SYSCLK. For more details, refer above
            .pllq           = 4,                      // Divides the 192MHz by 4 to provide 48MHz for the USB and SDIO clocks
            .ahb_prescaler  = RCC_CFGR_HPRE_DIV1,     // SystemCoreClock = SYSCLK = 96MHz
            .apb1_prescaler = RCC_CFGR_PPRE1_DIV2,    // APB1 = SystemCoreClock / 2 = 48MHz
            .apb2_prescaler = RCC_CFGR_PPRE2_DIV1,    // APB2 = SystemCoreClock = 96MHz
        },
    [HSE_PLL_84MHz] =
        {
            .flash_latency  = FLASH_ACR_LATENCY_2WS,  // 2 flash wait states since the SYSCLK is at a fairly high value
            .vos_scale      = 0b10 << PWR_CR_VOS_Pos, // Voltage scale of 2 since fairly high SYSCLK frequency
            .sysclk_source  = RCC_CFGR_SWS_PLL,       // SYSCLK source is the PLL
            .pll_source     = RCC_PLLCFGR_PLLSRC_HSE, // PLL source is the HSE
            .pllm           = HSE_VALUE_MHz,          // Divides the HSE by its value in MHz to give us 1MHz regardless of its starting value
            .plln           = 168,                    // Multiplies the resultant 1MHz by 168 to give 168MHz
            .pllp           = 0b00,                   // Divides the 168MHz by 2 to provide 84MHz for the SYSCLK. For more details, refer above
            .pllq           = 4,                      // Divides the 168MHz by 4 to provide 42MHz for the USB and SDIO clocks
            .ahb_prescaler  = RCC_CFGR_HPRE_DIV1,     // SystemCoreClock = SYSCLK = 84MHz
            .apb1_prescaler = RCC_CFGR_PPRE1_DIV2,    // APB1 = SystemCoreClock / 2 = 42MHz
            .apb2_prescaler = RCC_CFGR_PPRE2_DIV1,    // APB2 = SystemCoreClock = 84MHz
        },
    [HSE_PLL_64MHz] =
        {
            .flash_latency  = FLASH_ACR_LATENCY_1WS,  // 1 flash wait state since the SYSCLK is at a fairly low value
            .vos_scale      = 0b01 << PWR_CR_VOS_Pos, // Voltage scale of 3 since fairly low SYSCLK frequency
            .sysclk_source  = RCC_CFGR_SWS_PLL,       // SYSCLK source is the PLL
            .pll_source     = RCC_PLLCFGR_PLLSRC_HSE, // PLL source is the HSE
            .pllm           = HSE_VALUE_MHz,          // Divides the HSE by its value in MHz to give us 1MHz regardless of its starting value
            .plln           = 128,                    // Multiplies the resultant 1MHz by 128 to give 128MHz
            .pllp           = 0b00,                   // Divides the 128MHz by 2 to provide 64MHz for the SYSCLK. For more details, refer above
            .pllq           = 4,                      // Divides the 128MHz by 4 to provide 32MHz for the USB and SDIO clocks
            .ahb_prescaler  = RCC_CFGR_HPRE_DIV1,     // SystemCoreClock = SYSCLK = 64MHz
            .apb1_prescaler = RCC_CFGR_PPRE1_DIV2,    // APB1 = SystemCoreClock / 2 = 32MHz
            .apb2_prescaler = RCC_CFGR_PPRE2_DIV1,    // APB2 = SystemCoreClock = 64MHz
        },
    [HSE_PLL_48MHz] =
        {
            .flash_latency  = FLASH_ACR_LATENCY_1WS,  // 1 flash wait state since the SYSCLK is at a low value
            .vos_scale      = 0b01 << PWR_CR_VOS_Pos, // Voltage scale of 3 since low SYSCLK frequency
            .sysclk_source  = RCC_CFGR_SWS_PLL,       // SYSCLK source is the PLL
            .pll_source     = RCC_PLLCFGR_PLLSRC_HSE, // PLL source is the HSE
            .pllm           = HSE_VALUE_MHz,          // Divides the HSE by its value in MHz to give us 1MHz regardless of its starting value
            .plln           = 96,                     // Multiplies the resultant 1MHz by 96 to give 96MHz
            .pllp           = 0b00,                   // Divides the 96MHz by 2 to provide 48MHz for the SYSCLK. For more details, refer above
            .pllq           = 2,                      // Divides the 96MHz by 2 to provide 48MHz for the USB and SDIO clocks
            .ahb_prescaler  = RCC_CFGR_HPRE_DIV1,     // SystemCoreClock = SYSCLK = 48MHz
            .apb1_prescaler = RCC_CFGR_PPRE1_DIV1,    // APB1 = SystemCoreClock  = 48MHz
            .apb2_prescaler = RCC_CFGR_PPRE2_DIV1,    // APB2 = SystemCoreClock = 48MHz
        },
    [HSE_PLL_MATCH_HSI] =
        {
            .flash_latency  = FLASH_ACR_LATENCY_0WS,  // 0 flash wait state since the SYSCLK is at a low value
            .vos_scale      = 0b01 << PWR_CR_VOS_Pos, // Voltage scale of 3 since low SYSCLK frequency
            .sysclk_source  = RCC_CFGR_SWS_PLL,       // SYSCLK source is the PLL
            .pll_source     = RCC_PLLCFGR_PLLSRC_HSE, // PLL source is the HSE
            .pllm           = HSE_VALUE_MHz,          // Divides the HSE by its value in MHz to give us 1MHz regardless of its starting value
            .plln           = HSI_VALUE_MHz * 2,      // Multiplies the resultant 1MHz by HSI_VALUE_MHz * 2
            .pllp           = 0b00, // Divides the HSI_VALUE_MHz * 2 by 2 to provide HSI_VALUE_MHz for the SYSCLK. For more details, refer above
            .pllq           = 2,    // Divides the HSI_VALUE_MHz * 2 by 2 to provide HSI_VALUE_MHz for the USB and SDIO clocks
            .ahb_prescaler  = RCC_CFGR_HPRE_DIV1,  // SystemCoreClock = SYSCLK = HSI_VALUE_MHz
            .apb1_prescaler = RCC_CFGR_PPRE1_DIV1, // APB1 = SystemCoreClock  = HSI_VALUE_MHz
            .apb2_prescaler = RCC_CFGR_PPRE2_DIV1, // APB2 = SystemCoreClock = HSI_VALUE_MHz
        },
    [HSE_PLL_DIRECT] =
        {
            .flash_latency  = FLASH_ACR_LATENCY_0WS, // 0 flash wait states since the SYSCLK frequency is very low
            .sysclk_source  = RCC_CFGR_SWS_HSE,      // SYSCLK source is the HSE
            .ahb_prescaler  = RCC_CFGR_HPRE_DIV1,    // SystemCoreClock = SYSCLK = HSE
            .apb1_prescaler = RCC_CFGR_PPRE1_DIV1,   // APB1 = SystemCoreClock = HSE
            .apb2_prescaler = RCC_CFGR_PPRE2_DIV1,   // APB2 = SystemCoreClock = HSE
        },
    [HSI_PLL_100MHz] =
        {
            .flash_latency  = FLASH_ACR_LATENCY_3WS,  // 3 flash wait states since the SYSCLK is at the highest value
            .vos_scale      = 0b11 << PWR_CR_VOS_Pos, // Voltage scale of 1 since high SYSCLK frequency
            .sysclk_source  = RCC_CFGR_SWS_PLL,       // SYSCLK source is the PLL
            .pll_source     = RCC_PLLCFGR_PLLSRC_HSI, // PLL source is the HSI
            .pllm           = HSI_VALUE_MHz,          // Divides the HSI by its value in MHz to give us 1MHz regardless of its starting value
            .plln           = 200,                    // Multiplies the resultant 1MHz by 200 to give 200MHz
            .pllp           = 0b00,                   // Divides the 200MHz by 2 to provide 100MHz for the SYSCLK. For more details, refer above
            .pllq           = 4,                      // Divides the 200MHz by 4 to provide 50MHz for the USB and SDIO clocks
            .ahb_prescaler  = RCC_CFGR_HPRE_DIV1,     // SystemCoreClock = SYSCLK = 100MHz
            .apb1_prescaler = RCC_CFGR_PPRE1_DIV2,    // APB1 = SystemCoreClock / 2 = 50MHz
            .apb2_prescaler = RCC_CFGR_PPRE2_DIV1,    // APB2 = SystemCoreClock = 100MHz
        },
    [HSI_PLL_96MHz] =
        {
            .flash_latency  = FLASH_ACR_LATENCY_3WS,  // 3 flash wait states since the SYSCLK is at a very high value
            .vos_scale      = 0b11 << PWR_CR_VOS_Pos, // Voltage scale of 1 since high SYSCLK frequency
            .sysclk_source  = RCC_CFGR_SWS_PLL,       // SYSCLK source is the PLL
            .pll_source     = RCC_PLLCFGR_PLLSRC_HSI, // PLL source is the HSI
            .pllm           = HSI_VALUE_MHz,          // Divides the HSI by its value in MHz to give us 1MHz regardless of its starting value
            .plln           = 192,                    // Multiplies the resultant 1MHz by 192 to give 192MHz
            .pllp           = 0b00,                   // Divides the 192MHz by 2 to provide 96MHz for the SYSCLK. For more details, refer above
            .pllq           = 4,                      // Divides the 192MHz by 4 to provide 48MHz for the USB and SDIO clocks
            .ahb_prescaler  = RCC_CFGR_HPRE_DIV1,     // SystemCoreClock = SYSCLK = 96MHz
            .apb1_prescaler = RCC_CFGR_PPRE1_DIV2,    // APB1 = SystemCoreClock / 2 = 48MHz
            .apb2_prescaler = RCC_CFGR_PPRE2_DIV1,    // APB2 = SystemCoreClock = 96MHz
        },
    [HSI_PLL_84MHz] =
        {
            .flash_latency  = FLASH_ACR_LATENCY_2WS,  // 2 flash wait states since the SYSCLK is at a fairly high value
            .vos_scale      = 0b10 << PWR_CR_VOS_Pos, // Voltage scale of 2 since fairly high SYSCLK frequency
            .sysclk_source  = RCC_CFGR_SWS_PLL,       // SYSCLK source is the PLL
            .pll_source     = RCC_PLLCFGR_PLLSRC_HSI, // PLL source is the HSI
            .pllm           = HSI_VALUE_MHz,          // Divides the HSI by its value in MHz to give us 1MHz regardless of its starting value
            .plln           = 168,                    // Multiplies the resultant 1MHz by 168 to give 168MHz
            .pllp           = 0b00,                   // Divides the 168MHz by 2 to provide 84MHz for the SYSCLK. For more details, refer above
            .pllq           = 4,                      // Divides the 168MHz by 4 to provide 42MHz for the USB and SDIO clocks
            .ahb_prescaler  = RCC_CFGR_HPRE_DIV1,     // SystemCoreClock = SYSCLK = 84MHz
            .apb1_prescaler = RCC_CFGR_PPRE1_DIV2,    // APB1 = SystemCoreClock / 2 = 42MHz
            .apb2_prescaler = RCC_CFGR_PPRE2_DIV1,    // APB2 = SystemCoreClock = 84MHz
        },
    [HSI_PLL_64MHz] =
        {
            .flash_latency  = FLASH_ACR_LATENCY_1WS,  // 1 flash wait state since the SYSCLK is at a fairly low value
            .vos_scale      = 0b01 << PWR_CR_VOS_Pos, // Voltage scale of 3 since fairly low SYSCLK frequency
            .sysclk_source  = RCC_CFGR_SWS_PLL,       // SYSCLK source is the PLL
            .pll_source     = RCC_PLLCFGR_PLLSRC_HSI, // PLL source is the HSI
            .pllm           = HSI_VALUE_MHz,          // Divides the HSI by its value in MHz to give us 1MHz regardless of its starting value
            .plln           = 128,                    // Multiplies the resultant 1MHz by 128 to give 128MHz
            .pllp           = 0b00,                   // Divides the 128MHz by 2 to provide 64MHz for the SYSCLK. For more details, refer above
            .pllq           = 4,                      // Divides the 128MHz by 4 to provide 32MHz for the USB and SDIO clocks
            .ahb_prescaler  = RCC_CFGR_HPRE_DIV1,     // SystemCoreClock = SYSCLK = 64MHz
            .apb1_prescaler = RCC_CFGR_PPRE1_DIV2,    // APB1 = SystemCoreClock / 2 = 32MHz
            .apb2_prescaler = RCC_CFGR_PPRE2_DIV1,    // APB2 = SystemCoreClock = 64MHz
        },
    [HSI_PLL_48MHz] =
        {
            .flash_latency  = FLASH_ACR_LATENCY_1WS,  // 1 flash wait state since the SYSCLK is at a low value
            .vos_scale      = 0b01 << PWR_CR_VOS_Pos, // Voltage scale of 3 since low SYSCLK frequency
            .sysclk_source  = RCC_CFGR_SWS_PLL,       // SYSCLK source is the PLL
            .pll_source     = RCC_PLLCFGR_PLLSRC_HSI, // PLL source is the HSI
            .pllm           = HSI_VALUE_MHz,          // Divides the HSI by its value in MHz to give us 1MHz regardless of its starting value
            .plln           = 96,                     // Multiplies the resultant 1MHz by 96 to give 96MHz
            .pllp           = 0b00,                   // Divides the 96MHz by 2 to provide 48MHz for the SYSCLK. For more details, refer above
            .pllq           = 2,                      // Divides the 96MHz by 2 to provide 48MHz for the USB and SDIO clocks
            .ahb_prescaler  = RCC_CFGR_HPRE_DIV1,     // SystemCoreClock = SYSCLK = 48MHz
            .apb1_prescaler = RCC_CFGR_PPRE1_DIV1,    // APB1 = SystemCoreClock  = 48MHz
            .apb2_prescaler = RCC_CFGR_PPRE2_DIV1,    // APB2 = SystemCoreClock = 48MHz
        },
    [HSI_PLL_MATCH_HSE] =
        {
            .flash_latency  = FLASH_ACR_LATENCY_0WS,  // 0 flash wait states since the SYSCLK is at a low value
            .vos_scale      = 0b01 << PWR_CR_VOS_Pos, // Voltage scale of 3 since low SYSCLK frequency
            .sysclk_source  = RCC_CFGR_SWS_PLL,       // SYSCLK source is the PLL
            .pll_source     = RCC_PLLCFGR_PLLSRC_HSI, // PLL source is the HSI
            .pllm           = HSI_VALUE_MHz,          // Divides the HSI by its value in MHz to give us 1MHz regardless of its starting value
            .plln           = HSE_VALUE_MHz * 2,      // Multiplies the resultant 1MHz by HSE_VALUE_MHz * 2
            .pllp           = 0b00, // Divides the HSE_VALUE_MHz * 2 by 2 to provide HSE_VALUE_MHz for the SYSCLK. For more details, refer above
            .pllq           = 2,    // Divides the HSE_VALUE_MHz * 2 by 2 to provide HSE_VALUE_MHz for the USB and SDIO clocks
            .ahb_prescaler  = RCC_CFGR_HPRE_DIV1,  // SystemCoreClock = SYSCLK = HSE_VALUE_MHz
            .apb1_prescaler = RCC_CFGR_PPRE1_DIV1, // APB1 = SystemCoreClock  = HSE_VALUE_MHz
            .apb2_prescaler = RCC_CFGR_PPRE2_DIV1, // APB2 = SystemCoreClock = HSE_VALUE_MHz
        },
    [HSI_PLL_DIRECT] =
        {
            .flash_latency  = FLASH_ACR_LATENCY_0WS, // 0 flash wait states since the SYSCLK frequency is very low
            .sysclk_source  = RCC_CFGR_SWS_HSI,      // SYSCLK source is the HSI
            .ahb_prescaler  = RCC_CFGR_HPRE_DIV1,    // SystemCoreClock = SYSCLK = HSI
            .apb1_prescaler = RCC_CFGR_PPRE1_DIV1,   // APB1 = SystemCoreClock = HSI
            .apb2_prescaler = RCC_CFGR_PPRE2_DIV1,   // APB2 = SystemCoreClock = HSI
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

    // Temporarily switch the SYSCLK source to the HSI so we can safely disable the PLLs.
    // We use the HSI explicitly for this purpose because this function could be called
    // when the HSE experiences a fault, so trying to reuse it here will cause more issues.

    // Enable the HSI
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY));

    // Use the HSI as the SYSCLK source
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_HSI;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI);

    // Set the required flash latency
    FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY) | preset->flash_latency;
    __DSB();
    __ISB();

    // Disable the PLLs before configuring any clock
    RCC->CR &= ~(RCC_CR_PLLON | RCC_CR_PLLI2SON);
    while (RCC->CR & (RCC_CR_PLLRDY | RCC_CR_PLLI2SRDY));

    // Configure the voltage regulator. Requires that the PLLs be disabled
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    __DSB();

    // Set the required voltage regulator scale
    PWR->CR = (PWR->CR & ~PWR_CR_VOS) | preset->vos_scale;

    // Set the bus prescalers
    RCC->CFGR &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);
    RCC->CFGR |= (preset->ahb_prescaler | preset->apb1_prescaler | preset->apb2_prescaler);

    switch (preset->sysclk_source) {
        case RCC_CFGR_SWS_HSI:
            // Do nothing. The HSI is already enabled and the SYSCLK source
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
            // Enable the PLL source
            if (preset->pll_source == RCC_PLLCFGR_PLLSRC_HSI) {
                // Do nothing. The HSI is already enabled
            } else if (preset->pll_source == RCC_PLLCFGR_PLLSRC_HSE) {
                RCC->CR |= RCC_CR_HSEON;
                while (!(RCC->CR & RCC_CR_HSERDY));
            } else {
                ASSERT(0);
            }

            // Configure the PLL to provide the SYSCLK source
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

        // Disable the Clock Security System since the HSE is disabled
        RCC->CR &= ~RCC_CR_CSSON;
    } else {
        // Should be unreachable
        ASSERT(0);
    }

    __enable_irq();

    // Update the global variables tracking the system clock and reconfigure
    // the audio PLL to its old state since it was disabled here.
    system_core_clock_update();
    audio_pll_clock_config(AudioPLLCoreClockType);
}

static inline void audio_pll_clock_config_preset(const audio_clock_preset_t* preset) {
    __disable_irq();

    // Disable the audio PLL and clear all state
    RCC->CR &= ~RCC_CR_PLLI2SON;
    RCC->PLLI2SCFGR &= ~(RCC_PLLI2SCFGR_PLLI2SM | RCC_PLLI2SCFGR_PLLI2SN | RCC_PLLI2SCFGR_PLLI2SR);

    if (preset->disable) {
        __enable_irq();
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
            RCC->PLLCFGR = (RCC->PLLCFGR & ~RCC_PLLCFGR_PLLSRC) | RCC_PLLCFGR_PLLSRC_HSI;
            pllm         = HSI_VALUE_MHz;
            break;

        case RCC_CFGR_SWS_HSE:
            RCC->PLLCFGR = (RCC->PLLCFGR & ~RCC_PLLCFGR_PLLSRC) | RCC_PLLCFGR_PLLSRC_HSE;
            pllm         = HSE_VALUE_MHz;
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

    ASSERT(SystemCoreClock <= MAX_SYSTEM_CLOCK_Hz);
    ASSERT(APB1CoreClock <= MAX_APB1_CLOCK_Hz);
    ASSERT(APB2CoreClock <= MAX_APB2_CLOCK_Hz);

    // Reconfigure the SysTick since the system clock got updated
    systick_init();
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
    AudioPLLCoreClock     = (RCC->CR & RCC_CR_PLLI2SRDY) ? (((source / pllm) * plln) / pllr) : 0;
    ASSERT(AudioPLLCoreClock <= MAX_AUDIO_PLL_CLOCK_Hz);
}
