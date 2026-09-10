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
    HSE_DIRECT,     // The HSE feeds the SYSCLK directly, and has a value of HSE_VALUE_MHz (defined in common.h)

    // Derived from the HSI
    HSI_PLL_100MHz, // The HSI feeds the main PLL, which feeds the SYSCLK 100MHz
    HSI_PLL_96MHz,  // The HSI feeds the main PLL, which feeds the SYSCLK 96MHz, suitable for operation with USB
    HSI_PLL_84MHz,  // The HSI feeds the main PLL, which feeds the SYSCLK 84MHz
    HSI_PLL_64MHz,  // The HSI feeds the main PLL, which feeds the SYSCLK 64MHz
    HSI_PLL_48MHz,  // The HSI feeds the main PLL, which feeds the SYSCLK 48MHz, suitable for operation with USB
    HSI_DIRECT,     // The HSI feeds the SYSCLK directly, and has a value of HSI_VALUE_MHz (defined in common.h)
} system_clock_t;

typedef enum : uint8_t {
    AUDIO_PLL_DISABLE, // The Audio PLL is not active
    // The val
    AUDIO_PLL_76_8MHz,  // The Audio PLL at 76.8MHz. Suitable for the 48kHz family with no MCK output
    AUDIO_PLL_195_5MHz, // The Audio PLL at 195.5MHz. Suitable for the 48kHz family with MCK output
    AUDIO_PLL_151MHz,   // The Audio PLL at 151MHz. Suitable for the 44.1kHz family with no MCK output
    AUDIO_PLL_135_5MHz, // The Audio PLL at 135.5MHz. Suitable for the 44.1kHz family with no MCK output
} audio_clock_t;


// Global variables to match the SystemCoreClock variable, but for the APB1 and APB2 buses
extern system_clock_t SystemCoreClockType;
extern audio_clock_t  AudioPLLCoreClockType;
extern uint32_t       SystemCoreClock;
extern float          AudioPLLCoreClock;
extern uint32_t       APB1CoreClock;
extern uint32_t       APB2CoreClock;


// Configure the system clock, audio PLL and update the global variables tracking them
void system_core_clock_config(system_clock_t system_clock);
void system_core_clock_update(void);

void audio_pll_clock_config(audio_clock_t audio_clock);
void audio_pll_clock_update(void);


#ifdef __cplusplus
}
#endif


#endif // CLOCK_H_