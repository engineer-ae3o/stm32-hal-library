#ifndef _COMMON_H_
#define _COMMON_H_


#ifdef __cplusplus
extern "C" {
#endif


#include <stdint.h>


// Core clock and audio PLL frequencies
constexpr uint32_t CLOCK_SPEED_HZ = 100'000'000;
constexpr uint32_t AUDIO_PLL_HZ   = 76'800'000;

// HSI and HSE clocks
constexpr uint32_t HSE_VALUE_MHZ = 25;
constexpr uint32_t HSI_VALUE_MHZ = 16;

// At 100MHz, this is 100us: suitable for most use cases
constexpr uint32_t TIMEOUT_CYCLES = 10'000;

// SPI DMA has the highest priority
constexpr uint32_t SPI_DMA_NVIC_IRQ_PRIORITY  = 6;
constexpr uint32_t UART_DMA_NVIC_IRQ_PRIORITY = 8;
constexpr uint32_t I2S_DMA_NVIC_IRQ_PRIORITY  = 12;
constexpr uint32_t ADC_DMA_NVIC_IRQ_PRIORITY  = 12;

typedef enum hal_err_t : uint8_t {
    // Success
    HAL_OK = 0,

    // Generic failure
    HAL_FAIL,

    // More specific errors
    HAL_INVALID_ARG,
    HAL_INVALID_STATE,
    HAL_TIMEOUT,

    // Generic Transfer and Reception failures
    HAL_TX_ERROR,
    HAL_RX_ERROR,

    // I2C extensions
    HAL_I2C_DEVICE_NOT_FOUND,
    HAL_I2C_ARBITRATION_LOST,

    // SPI extensions
    HAL_SPI_TXE_FAILED_TO_SET,
    HAL_SPI_BSY_FAILED_TO_CLEAR,

    // UART extension
    HAL_UART_TC_FAILED_TO_SET,

    // DMA extensions
    HAL_DMA_TE,
    HAL_DMA_DME,
    HAL_DMA_ERR_UNKNOWN,
} hal_err_t;

const char* hal_err_to_string(hal_err_t err);

uint32_t get_tick_ms(void);


#ifdef __cplusplus
}
#endif

#endif // _COMMON_H_