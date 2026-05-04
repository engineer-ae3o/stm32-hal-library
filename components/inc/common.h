#ifndef _COMMON_H_
#define _COMMON_H_


#ifdef __cplusplus
extern "C" {
#endif


#include <stdint.h>


#define CLOCK_SPEED_HZ      100'000'000UL
#define HSE_VALUE_MHZ       25U
#define HSI_VALUE_MHZ       16U
#define USE_HSE             1U


typedef enum hal_err_t : uint8_t {
    HAL_OK                        = 0x00U,
    HAL_FAIL                      = 0x01U,

    HAL_INVALID_ARG               = 0x02U,
    HAL_INVALID_STATE             = 0x03U,
    HAL_TIMEOUT                   = 0x04U,

    // Generic Transfer and Reception failures
    HAL_TX_ERROR                  = 0x05U,
    HAL_RX_ERROR                  = 0x06U,

    // I2C extensions
    HAL_I2C_DEVICE_NOT_FOUND      = 0x07U,
    HAL_I2C_ARBITRATION_LOST      = 0x08U,

    // SPI extensions
    HAL_SPI_TXE_FAILED_TO_SET     = 0x09U,
    HAL_SPI_BSY_FAILED_TO_CLEAR   = 0x0AU,
    
    // UART extension
    HAL_UART_TC_FAILED_TO_SET     = 0x0BU,
    
    // DMA extensions
    HAL_DMA_TE                    = 0x0CU,
    HAL_DMA_TC                    = 0x0DU,
    HAL_DMA_DME                   = 0x0EU,
    HAL_DMA_HTE                   = 0x0FU,
    HAL_DMA_ERR_UNKNOWN           = 0x10U,
    
} hal_err_t;


static inline const char* hal_err_to_string(hal_err_t err) {
    switch (err) {
        case HAL_OK:                      return "HAL_OK";
        case HAL_FAIL:                    return "HAL_FAIL";
        case HAL_INVALID_ARG:             return "HAL_INVALID_ARG";
        case HAL_INVALID_STATE:           return "HAL_INVALID_STATE";
        case HAL_TIMEOUT:                 return "HAL_TIMEOUT";
        case HAL_TX_ERROR:                return "HAL_TX_ERROR";
        case HAL_RX_ERROR:                return "HAL_RX_ERROR";
        case HAL_I2C_DEVICE_NOT_FOUND:    return "HAL_I2C_DEVICE_NOT_FOUND";
        case HAL_I2C_ARBITRATION_LOST:    return "HAL_I2C_ARBITRATION_LOST";
        case HAL_SPI_TXE_FAILED_TO_SET:   return "HAL_SPI_TXE_FAILED_TO_SET";
        case HAL_SPI_BSY_FAILED_TO_CLEAR: return "HAL_SPI_BSY_FAILED_TO_CLEAR";
        case HAL_UART_TC_FAILED_TO_SET:   return "HAL_UART_TC_FAILED_TO_SET";
        case HAL_DMA_TC:                  return "HAL_DMA_TC";
        case HAL_DMA_TE:                  return "HAL_DMA_TE";
        case HAL_DMA_DME:                 return "HAL_DMA_DME";
        case HAL_DMA_HTE:                 return "HAL_DMA_HTE";
        case HAL_DMA_ERR_UNKNOWN:         return "HAL_DMA_ERR_UNKNOWN";
        default:                          return "";
    }
}


#ifdef __cplusplus
}
#endif


#endif // _COMMON_H_