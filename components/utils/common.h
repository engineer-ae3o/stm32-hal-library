#ifndef COMMON_H_
#define COMMON_H_


#ifdef __cplusplus
extern "C" {
#endif


#include "stm32f411xe.h"
#include "utils/log.h"
#include "utils/err.h"

#include <stdint.h>


// Core clock and audio PLL frequencies
#define MAX_CLOCK_SPEED_HZ (100'000'000)
#define AUDIO_PLL_HZ (76'800'000)


// Use the HSE, not the HSI
#define USE_HSE

#if defined(USE_HSI)
#undef USE_HSI
#endif


// HSI and HSE clocks
#define HSE_VALUE_MHZ (25)
#define HSI_VALUE_MHZ (16)


// At 100MHz, this is 100us: suitable for most use cases
#define TIMEOUT_CYCLES (10'000)

// SPI DMA has the highest priority
#define SPI_DMA_NVIC_IRQ_PRIORITY (14)
#define UART_DMA_NVIC_IRQ_PRIORITY (8)
#define I2S_DMA_NVIC_IRQ_PRIORITY (12)
#define ADC_DMA_NVIC_IRQ_PRIORITY (4)


#if defined(STM32F411xE)

// ADC values to use the internal temperature sensor. Gotten from the datasheet
#define NUM_OF_ADC_CONTROLLERS 1
#define MAX_ADC_CLOCK_MHZ 36
#define TEMP_SENSOR_AVERAGE_SLOPE 2.5F
#define TEMP_SENSOR_VSENSE_AT_25C 0.76F
#define TEMP_SENSOR_MIN_SAMPLING_TIME_US 10U
#define TEMP_SENSOR_STARUP_TIME_US 10U
#define VBAT_DIVIDER_RATIO 4U

#else
#error "No V_sense and Avg_Slope defined for the given target. Refer to the datasheet for your specific chip and add these values"
#endif


// Heap size
#define HEAP_SIZE_BYTES (32 * 1024)

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

// RTT buffer for logging. Controls the output buffer parameter
#define RTT_BUFFER_INDEX (0)

// Use the cycle counter component
#define USE_DWT_CYCCNT 1


#define REBOOT() restart(__PRETTY_FUNCTION__, __FILE__, __LINE__)
#define PANIC() panic(__PRETTY_FUNCTION__, __FILE__, __LINE__)
#define ASSERT(cond)                                                                                                                                 \
    do {                                                                                                                                             \
        if (!(cond)) {                                                                                                                               \
            LOGE("Assert", "Assert (%s) failed", #cond);                                                                                             \
            PANIC();                                                                                                                                 \
        }                                                                                                                                            \
    } while (0)

#define LOCK_ACQUIRE() __disable_irq()
#define LOCK_RELEASE() __enable_irq()

// Buffer size of format strings
#define FMT_STR_BUF_SIZE (192)


[[noreturn]] void panic(const char* function, const char* file, uint32_t line);
[[noreturn]] void restart(const char* function, const char* file, uint32_t line);


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