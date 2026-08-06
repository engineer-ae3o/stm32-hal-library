#ifndef _ERR_H_
#define _ERR_H_


#ifdef __cplusplus
extern "C" {
#endif


#include <stdint.h>


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


#ifdef __cplusplus
}
#endif


#endif // _ERR_H_