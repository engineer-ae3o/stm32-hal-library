#ifndef CLOCK_H_
#define CLOCK_H_


#ifdef __cplusplus
extern "C" {
#endif


#include <stdint.h>


typedef enum : uint8_t {
    // Derived from the HSE
    HSE_PLL_100MHz, // The HSE feeds the main PLL, which feeds the SYSCLK 100MHz
    HSE_PLL_96MHz,  // The HSE feeds the main PLL, which feeds the SYSCLK 96MHz, suitable for operation with USB
    HSE_PLL_84MHz,  // The HSE feeds the main PLL, which feeds the SYSCLK 84MHz
    HSE_PLL_64MHz,  // The HSE feeds the main PLL, which feeds the SYSCLK 64MHz
    HSE_PLL_48MHz,  // The HSE feeds the main PLL, which feeds the SYSCLK 48MHz, suitable for operation with USB
    HSE_PLL_DIRECT, // The HSE feeds the SYSCLK directly, and has a value of HSE_VALUE_MHz (defined in common.h)
    // Derived from the HSI
    HSI_PLL_100MHz, // The HSI feeds the main PLL, which feeds the SYSCLK 100MHz
    HSI_PLL_96MHz,  // The HSI feeds the main PLL, which feeds the SYSCLK 96MHz, suitable for operation with USB
    HSI_PLL_84MHz,  // The HSI feeds the main PLL, which feeds the SYSCLK 84MHz
    HSI_PLL_64MHz,  // The HSI feeds the main PLL, which feeds the SYSCLK 64MHz
    HSI_PLL_48MHz,  // The HSI feeds the main PLL, which feeds the SYSCLK 48MHz, suitable for operation with USB
    HSI_PLL_DIRECT, // The HSI feeds the SYSCLK directly, and has a value of HSI_VALUE_MHz (defined in common.h)
} system_clock_t;

typedef enum : uint8_t {
    AUDIO_PLL_DISABLE, // The Audio PLL is not active
    // The value of the audio PLL. It feeds the I2S peripheral directly, so the clock value should
    // be chosen with the needs of the target I2S sampling frequency in mind. These values were the
    // frequencies that minimize the audio sampling frequency errors for the given modes.
    AUDIO_PLL_76_8MHz,  // The Audio PLL at 76.8MHz. Suitable for the 48kHz family with no MCK output
    AUDIO_PLL_135_5MHz, // The Audio PLL at 135.5MHz. Suitable for the 44.1kHz family with MCK output
    AUDIO_PLL_151MHz,   // The Audio PLL at 151MHz. Suitable for the 44.1kHz family with no MCK output
    AUDIO_PLL_195_5MHz, // The Audio PLL at 195.5MHz. Suitable for the 48kHz family with MCK output
} audio_clock_t;

// Track the system clock, buses' clock and audio PLL frequencies
extern volatile system_clock_t SystemCoreClockType;
extern volatile audio_clock_t  AudioPLLCoreClockType;
extern volatile uint32_t       SystemCoreClock;
extern volatile uint32_t       APB1CoreClock;
extern volatile uint32_t       APB2CoreClock;
extern volatile uint32_t       AudioPLLCoreClock;


// Configure the system clock, audio PLL and update the global variables tracking them
void system_core_clock_config(system_clock_t clock);
void system_core_clock_update(void);

void audio_pll_clock_config(audio_clock_t clock);
void audio_pll_clock_update(void);


#ifdef __cplusplus
}
#endif


#endif // CLOCK_H_