#ifndef TICK_H_
#define TICK_H_


#ifdef __cplusplus
extern "C" {
#endif


#include "stm32f411xe.h"
#include "utils/common.h"
#include "utils/clock.h"

#include <stdint.h>


// Initialize or reinitialize (after a clock switch) the SysTick timer
void systick_init(void);

// Enable the DWT->CYCCNT counter if supported
void dwt_cnt_init(void);

// Get number of milliseconds since boot
uint32_t ms_since_boot(void);


// Polling delay macros
#define delay_ms(ms)                                                                                                                                 \
    do {                                                                                                                                             \
        uint32_t start = ms_since_boot();                                                                                                            \
        while ((ms_since_boot() - start) < (ms));                                                                                                    \
    } while (0)

#define delay_us(us)                                                                                                                                 \
    do {                                                                                                                                             \
        ASSERT(DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk);                                                                                                  \
        uint32_t cycles = (us) * (SystemCoreClock / 1'000'000U);                                                                                     \
        uint32_t start  = DWT->CYCCNT;                                                                                                               \
        while ((DWT->CYCCNT - start) < cycles);                                                                                                      \
    } while (0)


// Profiling helpers
[[maybe_unused]] static uint32_t s_prof_start = 0;

#define prof_start()                                                                                                                                 \
    do {                                                                                                                                             \
        ASSERT(DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk);                                                                                                  \
        s_prof_start = DWT->CYCCNT;                                                                                                                  \
    } while (0)

#define prof_end() (DWT->CYCCNT - s_prof_start)

// Convert cycles to time.
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