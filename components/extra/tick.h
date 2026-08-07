#ifndef TICK_H_
#define TICK_H_


#ifdef __cplusplus
extern "C" {
#endif


#include "stm32f411xe.h"

#include <stdint.h>


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

// Convert from cycles to time. Could overflow, hence 64 bit unsigned ints
#define cycles_to_ns(cycles) (((uint64_t)(cycles) * 1'000'000'000ULL) / SystemCoreClock)
#define cycles_to_us(cycles) (((uint64_t)(cycles) * 1'000'000ULL) / SystemCoreClock)
#define cycles_to_ms(cycles) (((uint64_t)(cycles) * 1'000ULL) / SystemCoreClock)


#ifdef __cplusplus
}
#endif


#endif // TICK_H_