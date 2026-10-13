#include "stm32f411xe.h"
#include "utils/common.h"
#include "utils/board.h"
#include "utils/clock.h"
#include "utils/tick.h"

#include <stdint.h>


// At startup, the HSI feeds the SYSCLK, and since there are no prescalers or divider
// active at boot, the values of the HCLK, PCLK1 and PCLK2 are equal to the HSI value
// Whereas, the audio PLL is disabled after reset.
static volatile system_clock_t s_system_core_clock_type = HSI_PLL_DIRECT;
static volatile audio_clock_t  s_audio_pll_clock_type   = AUDIO_PLL_DISABLE;

static volatile uint32_t s_system_core_clock = HSI_VALUE_Hz;
static volatile uint32_t s_apb1_core_clock   = HSI_VALUE_Hz;
static volatile uint32_t s_apb2_core_clock   = HSI_VALUE_Hz;
static volatile uint32_t s_audio_pll_clock   = 0;

// Lookup tables for the HCLK and APB prescalers
static const uint8_t s_ahb_presc_lut[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
static const uint8_t s_apb_presc_lut[8]  = {0, 0, 0, 0, 1, 2, 3, 4};


typedef struct {
    // General settings
    uint32_t flash_latency; // CMSIS defined macro. See FLASH_ACR_LATENCY_xxx
    uint32_t
        vos_scale; // CMSIS defined macro. See PWR_CR_VOS_xxx. Ignored if sysclk_source is RCC_CFGR_SWS_HSI or RCC_CFGR_SWS_HSE
    uint32_t sysclk_source; // CMSIS defined macro. See RCC_CFGR_SWS_xxx
    // These only have any meaning if sysclk_source is RCC_CFGR_SWS_PLL
    uint32_t pll_source; // CMSIS defined macro. See RCC_PLLCFGR_PLLSRC_xxx
    uint32_t pllm;       // Regular number. Allowed range is 2 - 63
    uint32_t plln;       // Regular number. Allowed range is 50 - 432
    uint32_t
        pllp; // Regular number. Allowed values are 2, 4, 6 and 8 (encoded as 0b00, 0b01, 0b10 and 0b11 respectively)
    uint32_t pllq; // Regular number. Allowed range is 2 - 15
    // Bus prescalers
    uint32_t ahb_prescaler;  // CMSIS defined macro. See RCC_CFGR_HPRE_DIVx
    uint32_t apb1_prescaler; // CMSIS defined macro. See RCC_CFGR_PPRE1_DIVx
    uint32_t apb2_prescaler; // CMSIS defined macro. See RCC_CFGR_PPRE2_DIVx
} system_clock_preset_t;

typedef struct {
    bool disable;  // Set to power down the audio PLL. The other fields are ignored if this is set
    uint32_t plln; // Regular number. Allowed range is 50 - 432
    uint32_t pllr; // Regular number. Allowed range is 2 - 7
} audio_clock_preset_t;

// Tables mapping the different clock frequencies and sources to their respective configuration data.
//
// The PLL input is always divided down to 1MHz (pllm = source frequency in MHz), so the VCO output
// frequency in MHz is simply plln. RM0383 requires the VCO output to be within 100 - 432MHz, which
// is enforced at compile time below and again at runtime in system_core_clock_config_preset().
// The PLLP encodings are 0b00 = /2, 0b01 = /4, 0b10 = /6 and 0b11 = /8.

#define VOS_SCALE1 (0b11U << PWR_CR_VOS_Pos) // Up to 100MHz
#define VOS_SCALE2 (0b10U << PWR_CR_VOS_Pos) // Up to 84MHz
#define VOS_SCALE3 (0b01U << PWR_CR_VOS_Pos) // Up to 64MHz

#define PLLP_DIV2 (0b00U)
#define PLLP_DIV4 (0b01U)
#define PLLP_DIV6 (0b10U)
#define PLLP_DIV8 (0b11U)

// VCO frequencies (in MHz, since the PLL input is 1MHz) of each frequency preset. Doubles as the PLLN values
#define VCO_100MHz_MHz (200U)                    // /2 = 100MHz SYSCLK, /4 = 50MHz PLLQ
#define VCO_96MHz_MHz (192U)                     // /2 = 96MHz SYSCLK,  /4 = 48MHz PLLQ (USB ok)
#define VCO_84MHz_MHz (336U)                     // /4 = 84MHz SYSCLK,  /7 = 48MHz PLLQ (USB ok)
#define VCO_64MHz_MHz (384U)                     // /6 = 64MHz SYSCLK,  /8 = 48MHz PLLQ (USB ok)
#define VCO_48MHz_MHz (192U)                     // /4 = 48MHz SYSCLK,  /4 = 48MHz PLLQ (USB ok)
#define VCO_MATCH_HSI_MHz ((HSI_VALUE_MHz) * 8U) // /8 = HSI_VALUE_MHz SYSCLK
#define VCO_MATCH_HSE_MHz ((HSE_VALUE_MHz) * 8U) // /8 = HSE_VALUE_MHz SYSCLK

#define PLL_VCO_CHECK(vco_MHz)                                                                     \
    _Static_assert((vco_MHz) >= 100U && (vco_MHz) <= 432U,                                         \
                   "PLL VCO output must be within range 100 - 432MHz")
PLL_VCO_CHECK(VCO_100MHz_MHz);
PLL_VCO_CHECK(VCO_96MHz_MHz);
PLL_VCO_CHECK(VCO_84MHz_MHz);
PLL_VCO_CHECK(VCO_64MHz_MHz);
PLL_VCO_CHECK(VCO_48MHz_MHz);
PLL_VCO_CHECK(VCO_MATCH_HSI_MHz);
PLL_VCO_CHECK(VCO_MATCH_HSE_MHz);

// pllm = source frequency in MHz must be an integer within 2 - 63 to give a 1MHz PLL input
_Static_assert((HSE_VALUE_Hz % 1'000'000U) == 0 && (HSE_VALUE_MHz >= 2U) && (HSE_VALUE_MHz <= 63U),
               "HSE must be a whole number of MHz in 2 - 63MHz");
_Static_assert((HSI_VALUE_Hz % 1'000'000U) == 0 && (HSI_VALUE_MHz >= 2U) && (HSI_VALUE_MHz <= 63U),
               "HSI must be a whole number of MHz in 2 - 63MHz");

// SYSCLK = HCLK (AHB /1) and APB2 /1 for every preset. Only the APB1 prescaler varies
#define PLL_PRESET(SRC, SRC_MHz, FLASH, VOS, N, P, Q, APB1)                                        \
    {                                                                                              \
        .flash_latency  = (FLASH),                                                                 \
        .vos_scale      = (VOS),                                                                   \
        .sysclk_source  = RCC_CFGR_SWS_PLL,                                                        \
        .pll_source     = (SRC),                                                                   \
        .pllm           = (SRC_MHz),                                                               \
        .plln           = (N),                                                                     \
        .pllp           = (P),                                                                     \
        .pllq           = (Q),                                                                     \
        .ahb_prescaler  = RCC_CFGR_HPRE_DIV1,                                                      \
        .apb1_prescaler = (APB1),                                                                  \
        .apb2_prescaler = RCC_CFGR_PPRE2_DIV1,                                                     \
    }

#define HSE_PLL_PRESET(...) PLL_PRESET(RCC_PLLCFGR_PLLSRC_HSE, HSE_VALUE_MHz, __VA_ARGS__)
#define HSI_PLL_PRESET(...) PLL_PRESET(RCC_PLLCFGR_PLLSRC_HSI, HSI_VALUE_MHz, __VA_ARGS__)

#define DIRECT_PRESET(SWS)                                                                         \
    {                                                                                              \
        .flash_latency  = FLASH_ACR_LATENCY_0WS,                                                   \
        .vos_scale      = VOS_SCALE3,                                                              \
        .sysclk_source  = (SWS),                                                                   \
        .ahb_prescaler  = RCC_CFGR_HPRE_DIV1,                                                      \
        .apb1_prescaler = RCC_CFGR_PPRE1_DIV1,                                                     \
        .apb2_prescaler = RCC_CFGR_PPRE2_DIV1,                                                     \
    }

// Arguments: flash latency, VOS scale, plln, pllp, pllq, APB1 prescaler
static const system_clock_preset_t s_system_clock_preset_lut[] = {
    // HSE
    [HSE_PLL_100MHz]    = HSE_PLL_PRESET(FLASH_ACR_LATENCY_3WS,
                                         VOS_SCALE1,
                                         VCO_100MHz_MHz,
                                         PLLP_DIV2,
                                         4,
                                         RCC_CFGR_PPRE1_DIV2), // APB1 = 50MHz
    [HSE_PLL_96MHz]     = HSE_PLL_PRESET(FLASH_ACR_LATENCY_3WS,
                                         VOS_SCALE1,
                                         VCO_96MHz_MHz,
                                         PLLP_DIV2,
                                         4,
                                         RCC_CFGR_PPRE1_DIV2), // APB1 = 48MHz
    [HSE_PLL_84MHz]     = HSE_PLL_PRESET(FLASH_ACR_LATENCY_2WS,
                                         VOS_SCALE2,
                                         VCO_84MHz_MHz,
                                         PLLP_DIV4,
                                         7,
                                         RCC_CFGR_PPRE1_DIV2), // APB1 = 42MHz
    [HSE_PLL_64MHz]     = HSE_PLL_PRESET(FLASH_ACR_LATENCY_1WS,
                                         VOS_SCALE3,
                                         VCO_64MHz_MHz,
                                         PLLP_DIV6,
                                         8,
                                         RCC_CFGR_PPRE1_DIV2), // APB1 = 32MHz
    [HSE_PLL_48MHz]     = HSE_PLL_PRESET(FLASH_ACR_LATENCY_1WS,
                                         VOS_SCALE3,
                                         VCO_48MHz_MHz,
                                         PLLP_DIV4,
                                         4,
                                         RCC_CFGR_PPRE1_DIV1), // APB1 = 48MHz
    [HSE_PLL_MATCH_HSI] = HSE_PLL_PRESET(
        FLASH_ACR_LATENCY_0WS, VOS_SCALE3, VCO_MATCH_HSI_MHz, PLLP_DIV8, 8, RCC_CFGR_PPRE1_DIV1),
    [HSE_PLL_DIRECT] = DIRECT_PRESET(RCC_CFGR_SWS_HSE),
    // HSI
    [HSI_PLL_100MHz]    = HSI_PLL_PRESET(FLASH_ACR_LATENCY_3WS,
                                         VOS_SCALE1,
                                         VCO_100MHz_MHz,
                                         PLLP_DIV2,
                                         4,
                                         RCC_CFGR_PPRE1_DIV2), // APB1 = 50MHz
    [HSI_PLL_96MHz]     = HSI_PLL_PRESET(FLASH_ACR_LATENCY_3WS,
                                         VOS_SCALE1,
                                         VCO_96MHz_MHz,
                                         PLLP_DIV2,
                                         4,
                                         RCC_CFGR_PPRE1_DIV2), // APB1 = 48MHz
    [HSI_PLL_84MHz]     = HSI_PLL_PRESET(FLASH_ACR_LATENCY_2WS,
                                         VOS_SCALE2,
                                         VCO_84MHz_MHz,
                                         PLLP_DIV4,
                                         7,
                                         RCC_CFGR_PPRE1_DIV2), // APB1 = 42MHz
    [HSI_PLL_64MHz]     = HSI_PLL_PRESET(FLASH_ACR_LATENCY_1WS,
                                         VOS_SCALE3,
                                         VCO_64MHz_MHz,
                                         PLLP_DIV6,
                                         8,
                                         RCC_CFGR_PPRE1_DIV2), // APB1 = 32MHz
    [HSI_PLL_48MHz]     = HSI_PLL_PRESET(FLASH_ACR_LATENCY_1WS,
                                         VOS_SCALE3,
                                         VCO_48MHz_MHz,
                                         PLLP_DIV4,
                                         4,
                                         RCC_CFGR_PPRE1_DIV1), // APB1 = 48MHz
    [HSI_PLL_MATCH_HSE] = HSI_PLL_PRESET(
        FLASH_ACR_LATENCY_0WS, VOS_SCALE3, VCO_MATCH_HSE_MHz, PLLP_DIV8, 8, RCC_CFGR_PPRE1_DIV1),
    [HSI_PLL_DIRECT] = DIRECT_PRESET(RCC_CFGR_SWS_HSI),
};

static const audio_clock_preset_t s_audio_clock_preset_lut[] = {
    [AUDIO_PLL_DISABLE]  = {.disable = true, .plln = 0, .pllr = 0},
    [AUDIO_PLL_76_8MHz]  = {.disable = false, .plln = 384, .pllr = 5},
    [AUDIO_PLL_135_5MHz] = {.disable = false, .plln = 271, .pllr = 2},
    [AUDIO_PLL_151MHz]   = {.disable = false, .plln = 302, .pllr = 2},
    [AUDIO_PLL_172MHz]   = {.disable = false, .plln = 344, .pllr = 2},
};

// Helpers
static const char* const TAG = "Clock";

// Poll a register until (*reg & mask) == value, giving up after TIMEOUT iterations
static inline bool wait_reg(volatile uint32_t* reg, uint32_t mask, uint32_t value) {
    for (uint32_t i = 0; i < TIMEOUT; i++) {
        if ((*reg & mask) == value) {
            return true;
        }
    }
    return false;
}

// Start the HSE. On failure (e.g. missing or faulty crystal), the HSE is switched back off
static inline bool hse_start(void) {
    RCC->CR |= RCC_CR_HSEON;
    if (wait_reg(&RCC->CR, RCC_CR_HSERDY, RCC_CR_HSERDY)) {
        return true;
    }
    RCC->CR &= ~RCC_CR_HSEON;
    return false;
}

// Returns false, leaving the SYSCLK running from the HSI, if the HSE or PLL failed to start
static bool system_core_clock_config_preset(const system_clock_preset_t* preset) {
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    // Temporarily switch the SYSCLK source to the HSI so we can safely disable the PLLs.
    // We use the HSI explicitly for this purpose because this function could be called
    // when the HSE experiences a fault, so trying to reuse it here will cause more issues.

    // Enable the HSI
    RCC->CR |= RCC_CR_HSION;
    ASSERT(wait_reg(&RCC->CR, RCC_CR_HSIRDY, RCC_CR_HSIRDY));

    // Use the HSI as the SYSCLK source
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_HSI;
    ASSERT(wait_reg(&RCC->CFGR, RCC_CFGR_SWS, RCC_CFGR_SWS_HSI));

    // Set the required flash latency
    FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY) | preset->flash_latency;
    __DSB();
    __ISB();

    // Disable the PLLs before configuring any clock
    RCC->CR &= ~(RCC_CR_PLLON | RCC_CR_PLLI2SON);
    ASSERT(wait_reg(&RCC->CR, RCC_CR_PLLRDY | RCC_CR_PLLI2SRDY, 0));

    // Configure the voltage regulator. Requires that the PLLs be disabled. The new scale only takes effect
    // once the PLL is on; while it is off the regulator is automatically in scale 3, so the VOS bits of the
    // direct HSI/HSE presets (left at zero) are harmless.
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    __DSB();

    // Set the required voltage regulator scale
    PWR->CR = (PWR->CR & ~PWR_CR_VOS) | preset->vos_scale;

    // Set the bus prescalers
    RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2)) |
                (preset->ahb_prescaler | preset->apb1_prescaler | preset->apb2_prescaler);

    switch (preset->sysclk_source) {
        case RCC_CFGR_SWS_HSI:
            // Do nothing. The HSI is already enabled and the SYSCLK source
            break;

        case RCC_CFGR_SWS_HSE:
            // Enable the HSE
            if (!hse_start()) {
                goto fault;
            }

            // Use the HSE as the SYSCLK source
            RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_HSE;
            ASSERT(wait_reg(&RCC->CFGR, RCC_CFGR_SWS, RCC_CFGR_SWS_HSE));
            break;

        case RCC_CFGR_SWS_PLL: {
            ASSERT(preset->pll_source == RCC_PLLCFGR_PLLSRC_HSI ||
                   preset->pll_source == RCC_PLLCFGR_PLLSRC_HSE);

            // Sanity check the prescaler values
            ASSERT(preset->pllm >= 2 && preset->pllm <= 63);
            ASSERT(preset->plln >= 50 && preset->plln <= 432);
            ASSERT(preset->pllp <= 0b11U);
            ASSERT(preset->pllq >= 2 && preset->pllq <= 15);

            // Sanity check the resulting frequencies: PLL input 1 - 2MHz, VCO output 100 - 432MHz
            const uint32_t pll_source_hz =
                (preset->pll_source == RCC_PLLCFGR_PLLSRC_HSE) ? HSE_VALUE_Hz : HSI_VALUE_Hz;
            const uint32_t pll_input_hz = pll_source_hz / preset->pllm;
            const uint32_t pll_vco_hz   = pll_input_hz * preset->plln;
            ASSERT(pll_input_hz >= 1'000'000U && pll_input_hz <= 2'000'000U);
            ASSERT(pll_vco_hz >= 100'000'000U && pll_vco_hz <= 432'000'000U);
            ASSERT(pll_vco_hz / ((preset->pllp + 1U) * 2U) <= MAX_SYSTEM_CLOCK_Hz);

            // Enable the PLL source
            if (preset->pll_source == RCC_PLLCFGR_PLLSRC_HSE && !hse_start()) {
                goto fault;
            }

            // Configure the PLL to provide the SYSCLK source
            RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLM | RCC_PLLCFGR_PLLN | RCC_PLLCFGR_PLLP |
                              RCC_PLLCFGR_PLLQ | RCC_PLLCFGR_PLLSRC);
            RCC->PLLCFGR |= (preset->pllm << RCC_PLLCFGR_PLLM_Pos) |
                            (preset->plln << RCC_PLLCFGR_PLLN_Pos) |
                            (preset->pllp << RCC_PLLCFGR_PLLP_Pos) |
                            (preset->pllq << RCC_PLLCFGR_PLLQ_Pos) | (preset->pll_source);

            // Enable the PLL
            RCC->CR |= RCC_CR_PLLON;
            if (!wait_reg(&RCC->CR, RCC_CR_PLLRDY, RCC_CR_PLLRDY)) {
                RCC->CR &= ~RCC_CR_PLLON;
                goto fault;
            }

            // Use the PLL as the SYSCLK source
            RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
            ASSERT(wait_reg(&RCC->CFGR, RCC_CFGR_SWS, RCC_CFGR_SWS_PLL));
            break;
        }

        default:
            // Should be unreachable
            __builtin_unreachable();
            ASSERT(0);
            break;
    }

    __DSB();
    __ISB();

    if (preset->sysclk_source == RCC_CFGR_SWS_HSE ||
        (preset->sysclk_source == RCC_CFGR_SWS_PLL &&
         preset->pll_source == RCC_PLLCFGR_PLLSRC_HSE)) {
        // The HSE is used either as the SYSCLK source directly or as the PLL input, so we disable the HSI since not in use to save power
        RCC->CR &= ~RCC_CR_HSION;
        ASSERT(wait_reg(&RCC->CR, RCC_CR_HSIRDY, 0));

        // Enable the Clock Security System to monitor the HSE
        RCC->CR |= RCC_CR_CSSON;
    } else {
        // The HSI is used either as the SYSCLK source directly or as the PLL input, so we disable the HSE since not in use to save power

        // Disable the Clock Security System first, since it monitors the HSE
        RCC->CR &= ~RCC_CR_CSSON;

        RCC->CR &= ~RCC_CR_HSEON;
        ASSERT(wait_reg(&RCC->CR, RCC_CR_HSERDY, 0));
    }

    __set_PRIMASK(primask);

    // Update the global variables tracking the system clock and reconfigure
    // the audio PLL to its old state since it was disabled here.
    system_core_clock_update();
    audio_pll_clock_config(s_audio_pll_clock_type);
    return true;

fault:
    __set_PRIMASK(primask);
    return false;
}

static void audio_pll_clock_config_preset(const audio_clock_preset_t* preset) {
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    // Disable the audio PLL and clear all state
    RCC->CR &= ~RCC_CR_PLLI2SON;
    RCC->PLLI2SCFGR &= ~(RCC_PLLI2SCFGR_PLLI2SM | RCC_PLLI2SCFGR_PLLI2SN | RCC_PLLI2SCFGR_PLLI2SR);

    if (preset->disable) {
        goto done;
    }

    // We use a PLLM value to get us a Vco of 1MHz regardless if the HSI or HSE is used.
    uint32_t pllm = 0;

    // Sanity check the prescaler values
    ASSERT(preset->plln >= 50 && preset->plln <= 432);
    ASSERT(preset->pllr >= 2 && preset->pllr <= 7);

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
            pllm = ((RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) == RCC_PLLCFGR_PLLSRC_HSE) ? HSE_VALUE_MHz
                                                                                   : HSI_VALUE_MHz;
            break;

        default:
            pllm = HSI_VALUE_MHz;
            break;
    }

    RCC->PLLI2SCFGR |= (pllm << RCC_PLLI2SCFGR_PLLI2SM_Pos) |
                       (preset->plln << RCC_PLLI2SCFGR_PLLI2SN_Pos) |
                       (preset->pllr << RCC_PLLI2SCFGR_PLLI2SR_Pos);

    // Enable the audio PLL
    RCC->CR |= RCC_CR_PLLI2SON;
    while (!(RCC->CR & RCC_CR_PLLI2SRDY));

    // Use the just configured I2S PLL for the I2S peripheral(s)
    RCC->CFGR &= ~RCC_CFGR_I2SSRC;

    __DSB();
    __ISB();

done:
    __set_PRIMASK(primask);
    audio_pll_clock_update();
}


// System Clock Configuration
void system_core_clock_config(system_clock_t clock) {
    ASSERT(clock < ARRAY_SIZE(s_system_clock_preset_lut));

    if (!system_core_clock_config_preset(&s_system_clock_preset_lut[clock])) {
        // The HSE (or the PLL) failed to start. The SYSCLK is still on the HSI, so make that official
        LOGE(TAG,
             "Failed to start the clock preset %u. Falling back to the HSI",
             (unsigned int)clock);
        clock = HSI_PLL_DIRECT;
        ASSERT(system_core_clock_config_preset(&s_system_clock_preset_lut[HSI_PLL_DIRECT]));
    }

    s_system_core_clock_type = clock;
}

void system_core_clock_update(void) {
    uint32_t sysclk = 0;

    // Get the SYSCLK clock source
    const uint32_t rcc_cfgr = RCC->CFGR;
    switch (rcc_cfgr & RCC_CFGR_SWS) {
        case RCC_CFGR_SWS_HSI:
            sysclk = HSI_VALUE_Hz;
            break;

        case RCC_CFGR_SWS_HSE:
            sysclk = HSE_VALUE_Hz;
            break;

        case RCC_CFGR_SWS_PLL:
            // Get the PLL clock source
            const uint32_t rcc_pllcfgr = RCC->PLLCFGR;
            if ((rcc_pllcfgr & RCC_PLLCFGR_PLLSRC) == RCC_PLLCFGR_PLLSRC_HSE) {
                // The HSE is the PLL clock source
                const uint32_t pllm = (rcc_pllcfgr & RCC_PLLCFGR_PLLM) >> RCC_PLLCFGR_PLLM_Pos;
                const uint32_t plln = (rcc_pllcfgr & RCC_PLLCFGR_PLLN) >> RCC_PLLCFGR_PLLN_Pos;
                const uint32_t pllp =
                    (((rcc_pllcfgr & RCC_PLLCFGR_PLLP) >> RCC_PLLCFGR_PLLP_Pos) + 1) * 2;
                const uint32_t pllvco = (HSE_VALUE_Hz / pllm) * plln;
                sysclk                = pllvco / pllp;
            } else {
                // The HSI is the PLL clock source
                const uint32_t pllm = (rcc_pllcfgr & RCC_PLLCFGR_PLLM) >> RCC_PLLCFGR_PLLM_Pos;
                const uint32_t plln = (rcc_pllcfgr & RCC_PLLCFGR_PLLN) >> RCC_PLLCFGR_PLLN_Pos;
                const uint32_t pllp =
                    (((rcc_pllcfgr & RCC_PLLCFGR_PLLP) >> RCC_PLLCFGR_PLLP_Pos) + 1) * 2;
                const uint32_t pllvco = (HSI_VALUE_Hz / pllm) * plln;
                sysclk                = pllvco / pllp;
            }
            break;

        default:
            sysclk = HSI_VALUE_Hz;
            break;
    }

    // Compute the HCLK, APB1 and APB2 bus frequencies
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_system_core_clock = sysclk >> s_ahb_presc_lut[rcc_cfgr >> RCC_CFGR_HPRE_Pos];
    s_apb1_core_clock   = s_system_core_clock >> s_apb_presc_lut[rcc_cfgr >> RCC_CFGR_PPRE1_Pos];
    s_apb2_core_clock   = s_system_core_clock >> s_apb_presc_lut[rcc_cfgr >> RCC_CFGR_PPRE2_Pos];
    __set_PRIMASK(primask);

    ASSERT(s_system_core_clock <= MAX_SYSTEM_CLOCK_Hz);
    ASSERT(s_apb1_core_clock <= MAX_APB1_CLOCK_Hz);
    ASSERT(s_apb2_core_clock <= MAX_APB2_CLOCK_Hz);

    // Reconfigure SysTick since the system clock got updated
    systick_init();
}


// Audio PLL Configuration
void audio_pll_clock_config(audio_clock_t clock) {
    ASSERT(clock < ARRAY_SIZE(s_audio_clock_preset_lut));
    audio_pll_clock_config_preset(&s_audio_clock_preset_lut[clock]);
    s_audio_pll_clock_type = clock;
}

void audio_pll_clock_update(void) {
    const uint32_t rcc_cr         = RCC->CR;
    const uint32_t rcc_pllcfgr    = RCC->PLLCFGR;
    const uint32_t rcc_plli2scfgr = RCC->PLLI2SCFGR;

    const uint32_t source =
        (rcc_pllcfgr & RCC_PLLCFGR_PLLSRC) == RCC_PLLCFGR_PLLSRC_HSE ? HSE_VALUE_Hz : HSI_VALUE_Hz;
    const uint32_t pllm = (rcc_plli2scfgr & RCC_PLLI2SCFGR_PLLI2SM) >> RCC_PLLI2SCFGR_PLLI2SM_Pos;
    const uint32_t plln = (rcc_plli2scfgr & RCC_PLLI2SCFGR_PLLI2SN) >> RCC_PLLI2SCFGR_PLLI2SN_Pos;
    const uint32_t pllr = (rcc_plli2scfgr & RCC_PLLI2SCFGR_PLLI2SR) >> RCC_PLLI2SCFGR_PLLI2SR_Pos;
    s_audio_pll_clock   = (rcc_cr & RCC_CR_PLLI2SON) ? (((source / pllm) * plln) / pllr) : 0;
    ASSERT(s_audio_pll_clock <= MAX_AUDIO_PLL_CLOCK_Hz);
}

// Track the system clock, buses' clock and audio PLL frequencies
system_clock_t get_system_core_clock_type(void) {
    return s_system_core_clock_type;
}

audio_clock_t get_audio_pll_clock_type(void) {
    return s_audio_pll_clock_type;
}

uint32_t get_system_core_clock(void) {
    return s_system_core_clock;
}

uint32_t get_audio_pll_clock(void) {
    return s_audio_pll_clock;
}

uint32_t get_apb1_core_clock(void) {
    return s_apb1_core_clock;
}

uint32_t get_apb2_core_clock(void) {
    return s_apb2_core_clock;
}

bool is_sysclk_on_hse(void) {
    switch (RCC->CFGR & RCC_CFGR_SWS) {
        case RCC_CFGR_SWS_HSE:
            return true;
        case RCC_CFGR_SWS_HSI:
            return false;
        default:
            return RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC_HSE;
    }
}
