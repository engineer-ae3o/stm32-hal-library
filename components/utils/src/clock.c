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

} system_clock_preset_t;

typedef struct {

} audio_clock_preset_t;

// Tables mapping the different clock frequencies and sources to their respective configuration data
const static system_clock_preset_t s_system_clock_preset_lut[] = {};
const static audio_clock_preset_t  s_audio_clock_preset_lut[]  = {};

// Helpers
static inline void system_core_clock_config_preset(const system_clock_preset_t* preset) {
    __disable_irq();


    // Set the flash latency, enable I and D caches, as well as the instruction prefetch buffer
    FLASH->ACR &= ~FLASH_ACR_LATENCY;
    FLASH->ACR |= (FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_3WS | FLASH_ACR_PRFTEN);
    __DSB();
    __ISB();

    // Disable the PLLs
    RCC->CR &= ~(RCC_CR_PLLON | RCC_CR_PLLI2SON);
    while (RCC->CR & (RCC_CR_PLLRDY | RCC_CR_PLLI2SRDY));

    // Configure the voltage regulator. Requires that the PLLs be disabled
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    __DSB();
    PWR->CR &= ~PWR_CR_VOS;
    PWR->CR |= (PWR_CR_VOS_1 | PWR_CR_VOS_0);

#ifdef USE_HSE
    // Enable the HSE
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY));

    // Configure the PLL to provide a clock of 100MHz, derived from the HSE
    RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLM | RCC_PLLCFGR_PLLN | RCC_PLLCFGR_PLLP | RCC_PLLCFGR_PLLSRC);
    RCC->PLLCFGR |= (HSE_VALUE_MHz << RCC_PLLCFGR_PLLM_Pos) | (200 << RCC_PLLCFGR_PLLN_Pos) | (0 << RCC_PLLCFGR_PLLP_Pos) | (RCC_PLLCFGR_PLLSRC_HSE);
#else
    // Enable the HSI
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY));

    // Configure the PLL to provide a clock of 100MHz, derived from the HSI
    RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLM | RCC_PLLCFGR_PLLN | RCC_PLLCFGR_PLLP | RCC_PLLCFGR_PLLSRC);
    RCC->PLLCFGR |= (HSI_VALUE_MHz << RCC_PLLCFGR_PLLM_Pos) | (200 << RCC_PLLCFGR_PLLN_Pos) | (0 << RCC_PLLCFGR_PLLP_Pos) | (RCC_PLLCFGR_PLLSRC_HSI);
#endif

    // Set the bus prescalers.
    RCC->CFGR &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);
    RCC->CFGR |= (RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PPRE2_DIV1);

    // Enable the PLL
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));

    // Ensure the VOSRDY bit reads 1 after the PLLs have been enabled
    while (!(PWR->CSR & PWR_CSR_VOSRDY));

    // Use the PLL as the SYSCLK source
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

    __DSB();
    __ISB();

    // Enable the CSS to monitor the HSE
    RCC->CR |= RCC_CR_CSSON;

#ifdef USE_HSE
    // Disable the HSI since not in use
    RCC->CR &= ~RCC_CR_HSION;
    while (RCC->CR & RCC_CR_HSIRDY);
#else
    // Disable the HSE since not in use
    RCC->CR &= ~RCC_CR_HSEON;
    while (RCC->CR & RCC_CR_HSERDY);
#endif


    __enable_irq();

    system_core_clock_update();
}

static inline void audio_pll_clock_config_preset(const audio_clock_preset_t* preset) {
    __disable_irq();

    // Disable the audio PLL
    RCC->CR &= ~RCC_CR_PLLI2SON;

    // Clear all state
    RCC->PLLI2SCFGR &= ~(RCC_PLLI2SCFGR_PLLI2SM | RCC_PLLI2SCFGR_PLLI2SN | RCC_PLLI2SCFGR_PLLI2SR);
    RCC->CFGR &= ~RCC_CFGR_I2SSRC;

    if (clock == AUDIO_PLL_DISABLE) {
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
