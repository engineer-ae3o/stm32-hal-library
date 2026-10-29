#ifndef ERR_H_
#define ERR_H_


#ifdef __cplusplus
extern "C" {
#endif


#include <stdint.h>


typedef enum : uint8_t {
    HAL_OK = 0, // Success

    HAL_FAIL, // Generic failure

    HAL_ERR_TX, // Generic transfer error
    HAL_ERR_RX, // Generic reception error

    HAL_ERR_TIMEOUT, // Operation or function timed out
    HAL_ERR_NOT_DONE, // Operation not completed when another operation is requested
    HAL_ERR_NOT_FOUND,    // Resource not found
    HAL_ERR_OUT_OF_MEM,   // The heap ran out of memory
    HAL_ERR_INVALID_ARG,  // Invalid argument passed to a function
    HAL_ERR_INVALID_SIZE, // Invalid size passed to a function
    HAL_ERR_INVALID_STATE, // Application or hardware in an invalid state when something is requested
    HAL_ERR_NOT_SUPPORTED, // Feature or operation not supported

    // I2C extensions
    HAL_ERR_I2C_BUS_BUSY, // The bus is in use by a different master
    HAL_ERR_I2C_BUS_ERROR, // An illegal start or stop condition detected on the bus
    HAL_ERR_I2C_DEVICE_NOT_FOUND, // The selected slave with the given address was not found on the bus
    HAL_ERR_I2C_ARBITRATION_LOST, // I2C bus arbitration lost to another master

    // SPI extensions
    HAL_ERR_SPI_TXE_FAILED_TO_SET, // The TXE bit failed to set after SPI transfers were completed
    HAL_ERR_SPI_BSY_FAILED_TO_CLEAR, // The BSY bit failed to clear after SPI transfers were completed

    // UART extension
    HAL_ERR_UART_TC_FAILED_TO_SET, // The TC bit in the ISR failed to set even after the DMA TX done irq had fired

    // DMA extensions
    HAL_ERR_DMA_TE,  // DMA transfer error
    HAL_ERR_DMA_DME, // Direct mode error
    HAL_ERR_DMA_FE,  // FIFO mode error

    // Timer extension
    HAL_ERR_TIM_OVER_CAPTURE, // Overcapture in input capture mode on a timer

    // Sentinel value
    HAL_SENTINEL_COUNT,
} hal_err_t;


const char* hal_err_to_string(hal_err_t err);


#ifdef __cplusplus
}
#endif


#endif // ERR_H_