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
    HAL_ERR_NOT_SUPPORTED,
    HAL_ERR_TIMEOUT,
    HAL_ERR_NOT_DONE,
    HAL_ERR_NOT_FOUND,

    // Generic Transfer and Reception failures
    HAL_ERR_TX,
    HAL_ERR_RX,

    // I2C extensions
    HAL_ERR_I2C_DEVICE_NOT_FOUND, // The selectd slave with the given address was not found on the bus
    HAL_ERR_I2C_ARBITRATION_LOST, // I2C bus arbitration lost to another master

    // SPI extensions
    HAL_ERR_SPI_TXE_FAILED_TO_SET,   // The TXE bit failed to set after SPI transfers were completed
    HAL_ERR_SPI_BSY_FAILED_TO_CLEAR, // The BSY bit failed to set after SPI transfers were completed

    // UART extension
    HAL_ERR_UART_TC_FAILED_TO_SET, // The TC bit in the ISR failed to set even after the DMA TX done irq had fired

    // DMA extensions
    HAL_ERR_DMA_TE,          // DMA transfer error
    HAL_ERR_DMA_DME,         // Direct mode error
    HAL_ERR_DMA_FE,          // FIFO mode error
    HAL_ERR_DMA_ERR_UNKNOWN, // A DMA error occurred, but its sources are unknown

    // Sentinel value
    HAL_ERR_COUNT,
} hal_err_t;

const char* hal_err_to_string(hal_err_t err);


#ifdef __cplusplus
}
#endif


#endif // ERR_H_