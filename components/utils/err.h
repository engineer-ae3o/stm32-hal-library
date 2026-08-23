#ifndef ERR_H_
#define ERR_H_


#ifdef __cplusplus
extern "C" {
#endif


#include <stdint.h>


typedef enum : uint8_t {
    // Success
    HAL_OK = 0,

    // Generic failure
    HAL_FAIL,

    // More specific errors
    HAL_ERR_INVALID_ARG,
    HAL_ERR_INVALID_STATE,
    HAL_ERR_TIMEOUT,
    HAL_ERR_NOT_DONE,
    HAL_ERR_NOT_FOUND,

    // Generic Transfer and Reception failures
    HAL_ERR_TX,
    HAL_ERR_RX,

    // I2C extensions
    HAL_ERR_I2C_DEVICE_NOT_FOUND,
    HAL_ERR_I2C_ARBITRATION_LOST,

    // SPI extensions
    HAL_ERR_SPI_TXE_FAILED_TO_SET,
    HAL_ERR_SPI_BSY_FAILED_TO_CLEAR,

    // UART extension
    HAL_ERR_UART_TC_FAILED_TO_SET,

    // DMA extensions
    HAL_ERR_DMA_TE,
    HAL_ERR_DMA_DME,
    HAL_ERR_DMA_ERR_UNKNOWN,

    HAL_ERR_COUNT, // Sentinel value
} hal_err_t;

const char* hal_err_to_string(hal_err_t err);


#ifdef __cplusplus
}
#endif


#endif // ERR_H_