#ifndef COMMON_H_
#define COMMON_H_


#ifdef __cplusplus
extern "C" {
#endif


#include "stm32f411xe.h"
#include "utils/log.h"
#include "utils/err.h"

#include <stdint.h>


// Loop iteration based timeout
#define TIMEOUT (1'000'000U)
#define UNUSED(x) (void)(x)

// NVIC interrupt priorities for the different peripherals
#define SysTick_NVIC_IRQ_PRIORITY (0U)
#define TIMER_NVIC_IRQ_PRIORITY (15U)
#define EXTI_LINE_NVIC_IRQ_PRIORITY (14U)
#define SPI_DMA_NVIC_IRQ_PRIORITY (6U)
#define I2S_DMA_NVIC_IRQ_PRIORITY (12U)
#define UART_DMA_NVIC_IRQ_PRIORITY (10U)
#define ADC_NVIC_IRQ_PRIORITY (8U)
#define ADC_DMA_NVIC_IRQ_PRIORITY ADC_NVIC_IRQ_PRIORITY
#define CRC_DMA_NVIC_IRQ_PRIORITY (11U)
#define M2M_DMA_NVIC_IRQ_PRIORITY (13U)


// "Branch prediction" hints (technically, no branch predictor exists on this hardware, but it
// would help gcc order the branches in a more efficient manner, causing better I-cache locality)
#define gnu_likely(x) __builtin_expect(!!(x), 1)
#define gnu_unlikely(x) __builtin_expect(!!(x), 0)

// The system tick rate
#define TICK_RATE_Hz (1000U)

#define HEAP_SIZE_kB (32U)
#define HEAP_SIZE_BYTES ((HEAP_SIZE_kB) * 1024)
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))


// Ceiling division: smallest integer >= a/b.
// Assumes a + b - 1 does not overflow the type.
[[__gnu__::__always_inline__]] static inline uint32_t ceil_div_u32(uint32_t a, uint32_t b) {
    return (a + (b - 1)) / b;
}

[[__gnu__::__always_inline__]] static inline uint64_t ceil_div_u64(uint64_t a, uint64_t b) {
    return (a + (b - 1)) / b;
}

// Round half up division: nearest integer to a/b.
// Assumes a + b/2 does not overflow the type.
[[__gnu__::__always_inline__]] static inline uint32_t round_div_u32(uint32_t a, uint32_t b) {
    return (a + (b / 2)) / b;
}

[[__gnu__::__always_inline__]] static inline uint64_t round_div_u64(uint64_t a, uint64_t b) {
    return (a + (b / 2)) / b;
}


#define REBOOT()                                                                                                                                     \
    do {                                                                                                                                             \
        reboot(__PRETTY_FUNCTION__, __FILE__, __LINE__);                                                                                             \
    } while (0)

#define PANIC()                                                                                                                                      \
    do {                                                                                                                                             \
        panic(__PRETTY_FUNCTION__, __FILE__, __LINE__);                                                                                              \
    } while (0)

#define ASSERT(cond)                                                                                                                                 \
    do {                                                                                                                                             \
        assert_check((cond), #cond, __PRETTY_FUNCTION__, __FILE__, __LINE__);                                                                        \
    } while (0)


[[__gnu__::__noreturn__, __gnu__::__always_inline__]] static inline void halt(void) {
    __disable_irq();
#if defined(DEBUG)
    __BKPT(0);
#endif
    while (true) {
        __WFI();
    }
}

[[__gnu__::__noreturn__]] void panic(const char* function, const char* file, int line);

void restart(const char* function, const char* file, int line);
void assert_check(bool cond, const char* msg, const char* function, const char* file, int line);


// RTT buffer for logging. Controls the output buffer parameter
#define RTT_BUFFER_INDEX (0)

// Buffer size of format strings
#define FMT_STR_BUF_SIZE (192)


// Macros to help with error propagation
#define TRY(func)                                                                                                                                    \
    do {                                                                                                                                             \
        hal_err_t ret_ = (func);                                                                                                                     \
        if (ret_ != HAL_OK) {                                                                                                                        \
            LOGE("ERROR", "%s (%s:%d) failed: %s", #func, __FILE__, __LINE__, hal_err_to_string(ret_));                                              \
            return ret_;                                                                                                                             \
        }                                                                                                                                            \
    } while (0)

#define TRY_WITH_FUNC(func, err_cb)                                                                                                                  \
    do {                                                                                                                                             \
        hal_err_t ret_ = (func);                                                                                                                     \
        if (ret_ != HAL_OK) {                                                                                                                        \
            LOGE("ERROR", "%s (%s:%d) failed: %s", #func, __FILE__, __LINE__, hal_err_to_string(ret_));                                              \
            (err_cb);                                                                                                                                \
            return ret_;                                                                                                                             \
        }                                                                                                                                            \
    } while (0)

#define TRY_WITH_FUNC_NO_RET(func, err_cb)                                                                                                           \
    do {                                                                                                                                             \
        hal_err_t ret_ = (func);                                                                                                                     \
        if (ret_ != HAL_OK) {                                                                                                                        \
            LOGE("ERROR", "%s (%s:%d) failed: %s", #func, __FILE__, __LINE__, hal_err_to_string(ret_));                                              \
            (err_cb);                                                                                                                                \
        }                                                                                                                                            \
    } while (0)

#define TRY_THEN_LOG(func, msg)                                                                                                                      \
    do {                                                                                                                                             \
        hal_err_t ret_ = (func);                                                                                                                     \
        if (ret_ != HAL_OK) {                                                                                                                        \
            LOGE(TAG, "%s: %s", msg, hal_err_to_string(ret_));                                                                                       \
        }                                                                                                                                            \
    } while (0)


#ifdef __cplusplus
}
#endif


#endif // COMMON_H_