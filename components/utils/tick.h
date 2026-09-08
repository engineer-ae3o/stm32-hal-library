#ifndef TICK_H_
#define TICK_H_


#ifdef __cplusplus
extern "C" {
#endif


#include "stm32f411xe.h"

#include <stdint.h>


// The system tick rate
#define TICK_RATE_HZ (1000)

// Converts a time in milliseconds to a time in ticks.
#define MS_TO_TICKS(ms) (((ms) * TICK_RATE_HZ) / 1000)

// Converts a time in ticks to a time in milliseconds.
#define TICKS_TO_MS(ticks) (((ticks) * 1000U) / TICK_RATE_HZ)

// Get number of miliseconds that have passed since bootup
uint32_t ticks_since_boot_ms(void);


// Polling delay functions
#define delay_ms(ms)                                                                                                                                 \
    do {                                                                                                                                             \
        uint32_t start = ticks_since_boot_ms();                                                                                                      \
        while ((ticks_since_boot_ms() - start) < (ms));                                                                                              \
    } while (0)

#define delay_us(us)                                                                                                                                 \
    do {                                                                                                                                             \
        uint32_t cycles = (us) * (SystemCoreClock / 1'000'000U);                                                                                     \
        uint32_t start  = DWT->CYCCNT;                                                                                                               \
        while ((DWT->CYCCNT - start) < cycles);                                                                                                      \
    } while (0)


// Profiling helpers
[[maybe_unused]] static uint32_t s_prof_start = 0;

#define prof_start()                                                                                                                                 \
    do {                                                                                                                                             \
        s_prof_start = DWT->CYCCNT;                                                                                                                  \
    } while (0)

#define prof_end() (DWT->CYCCNT - s_prof_start)

// Convert from cycles to time.
#define cycles_to_us(cycles) ((cycles) * 1'000'000ULL / SystemCoreClock)
#define cycles_to_us_frac(cycles) (((cycles) * 1'000'000'000ULL / SystemCoreClock) % 1'000ULL)

#define cycles_to_ms(cycles) ((cycles) * 1'000ULL / SystemCoreClock)
#define cycles_to_ms_frac(cycles) (((cycles) * 1'000'000ULL / SystemCoreClock) % 1'000ULL)

#define cycles_to_ss(cycles) ((cycles) / SystemCoreClock)
#define cycles_to_ss_frac(cycles) (((cycles) * 1'000ULL / SystemCoreClock) % 1'000ULL)


#ifdef __cplusplus
}
#endif


#endif // TICK_H_