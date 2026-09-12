#ifndef CLOCK_H_
#define CLOCK_H_


#ifdef __cplusplus
extern "C" {
#endif


#include <stdint.h>


typedef enum : uint8_t {
    // Derived from the HSE
    HSE_PLL_100MHz,    // The HSE feeds the main PLL, which feeds the SYSCLK 100MHz
    HSE_PLL_96MHz,     // The HSE feeds the main PLL, which feeds the SYSCLK 96MHz, suitable for operation with USB
    HSE_PLL_84MHz,     // The HSE feeds the main PLL, which feeds the SYSCLK 84MHz
    HSE_PLL_64MHz,     // The HSE feeds the main PLL, which feeds the SYSCLK 64MHz
    HSE_PLL_48MHz,     // The HSE feeds the main PLL, which feeds the SYSCLK 48MHz, suitable for operation with USB
    HSE_PLL_MATCH_HSI, // The HSE feeds the main PLL to generate a clock frequency matching HSI_VALUE_MHz (defined in board.h)
    HSE_PLL_DIRECT,    // The HSE feeds the SYSCLK directly, and has a value of HSE_VALUE_MHz (defined in board.h)
    // Derived from the HSI
    HSI_PLL_100MHz,    // The HSI feeds the main PLL, which feeds the SYSCLK 100MHz
    HSI_PLL_96MHz,     // The HSI feeds the main PLL, which feeds the SYSCLK 96MHz, suitable for operation with USB
    HSI_PLL_84MHz,     // The HSI feeds the main PLL, which feeds the SYSCLK 84MHz
    HSI_PLL_64MHz,     // The HSI feeds the main PLL, which feeds the SYSCLK 64MHz
    HSI_PLL_48MHz,     // The HSI feeds the main PLL, which feeds the SYSCLK 48MHz, suitable for operation with USB
    HSI_PLL_MATCH_HSE, // The HSI feeds the main PLL to generate a clock frequency matching HSE_VALUE_MHz (defined in board.h)
    HSI_PLL_DIRECT,    // The HSI feeds the SYSCLK directly, and has a value of HSI_VALUE_MHz (defined in board.h)
} system_clock_t;

typedef enum : uint8_t {
    AUDIO_PLL_DISABLE, // The Audio PLL is not active
    // The value of the audio PLL. It feeds the I2S peripheral directly, so the clock value should
    // be chosen with the needs of the target I2S sampling frequency in mind. These values were the
    // frequencies that minimize the audio sampling frequency errors for the given modes.
    AUDIO_PLL_76_8MHz,  // The Audio PLL at 76.8MHz. Suitable for the 48kHz family with MCK output disabled
    AUDIO_PLL_135_5MHz, // The Audio PLL at 135.5MHz. Suitable for the 44.1kHz family with MCK output enabled
    AUDIO_PLL_151MHz,   // The Audio PLL at 151MHz. Suitable for the 44.1kHz family with MCK output disabled
    AUDIO_PLL_172MHz,   // The Audio PLL at 172MHz. Suitable for the 48kHz family with MCK output enabled
    // NOTE: Due to hardware limits, operation with an I2S frequency of 192kHz and MCK output enabled is not
    // supported due to theinaccuracies the clock would produce, as this would render this specific setup not
    // feasible and impractical for any realistic I2S and audio usage.
} audio_clock_t;

// Configure the system clock, audio PLL and update the global variables tracking them
void system_core_clock_config(system_clock_t clock);
void system_core_clock_update(void);

void audio_pll_clock_config(audio_clock_t clock);
void audio_pll_clock_update(void);

// Track the system clock, buses' clock and audio PLL frequencies
system_clock_t get_system_core_clock_type(void);
audio_clock_t  get_audio_pll_clock_type(void);

uint32_t get_system_core_clock(void);
uint32_t get_audio_pll_clock(void);
uint32_t get_apb1_core_clock(void);
uint32_t get_apb2_core_clock(void);
bool     is_sysclk_on_hse(void);


#ifdef __cplusplus
}
#endif


#endif // CLOCK_H_