#ifndef CLOCK_H_
#define CLOCK_H_


#ifdef __cplusplus
extern "C" {
#endif


#include <stdint.h>


// Global variables to match the SystemCoreClock variable, but for the APB1 and APB2 buses
extern uint32_t SystemCoreClock;
extern uint32_t AudioPLLCoreClock;
extern uint32_t APB1CoreClock;
extern uint32_t APB2CoreClock;


// COnfigure the system clock, audio PLL and update the global variables tracking them
void system_core_clock_config();
void audio_pll_clock_config();
void system_core_clock_update();


#ifdef __cplusplus
}
#endif


#endif // CLOCK_H_