#ifndef COMMON_H_
#define COMMON_H_


#ifdef __cplusplus
extern "C" {
#endif


#include "stm32f411xe.h"
#include "utils/log.h"
#include "utils/err.h"

#include <stdint.h>


// HSI and HSE clocks
#define HSE_VALUE_MHz (25U)
#define HSE_VALUE_Hz (HSE_VALUE_MHz * 1000000U)

#define HSI_VALUE_MHz (16U)
#define HSI_VALUE_Hz (HSI_VALUE_MHz * 1000000U)


#define TIMEOUT_CYCLES (10'000U)
#define UNUSED(x) (void)(x)

// NVIC interrupt priorities for the different peripherals
#define TIM2_NVIC_IRQ_PRIORITY (3U)
#define SPI_DMA_NVIC_IRQ_PRIORITY (6U)
#define I2S_DMA_NVIC_IRQ_PRIORITY (12U)
#define UART_DMA_NVIC_IRQ_PRIORITY (10U)
#define ADC_DMA_NVIC_IRQ_PRIORITY (8U)
#define CRC_DMA_NVIC_IRQ_PRIORITY (11U)


// The system tick rate
#define TICK_RATE_HZ (1000)

// Heap size
#define HEAP_SIZE_BYTES (32 * 1024)

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

// RTT buffer for logging. Controls the output buffer parameter
#define RTT_BUFFER_INDEX (0)

// Use the cycle counter component
#define USE_DWT_CYCCNT 1

#define HALT()                                                                                                                                       \
    do {                                                                                                                                             \
        __disable_irq();                                                                                                                             \
        __BKPT(0);                                                                                                                                   \
        while (true) {                                                                                                                               \
            __WFI();                                                                                                                                 \
        }                                                                                                                                            \
    } while (0)


#define REBOOT()                                                                                                                                     \
    do {                                                                                                                                             \
        LOGI("Restart", "System reboot requested from %s (%s:%d)", __PRETTY_FUNCTION__, __FILE__, __LINE__);                                         \
        NVIC_SystemReset();                                                                                                                          \
    } while (0)

#define PANIC()                                                                                                                                      \
    do {                                                                                                                                             \
        LOGE("Panic", "System ran into a fatal error from %s (%s:%d)", __PRETTY_FUNCTION__, __FILE__, __LINE__);                                     \
        HALT();                                                                                                                                      \
    } while (0)

#define ASSERT(cond)                                                                                                                                 \
    do {                                                                                                                                             \
        if (!(cond)) {                                                                                                                               \
            LOGE("Assert", "Assert (%s) failed", #cond);                                                                                             \
            PANIC();                                                                                                                                 \
        }                                                                                                                                            \
    } while (0)

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