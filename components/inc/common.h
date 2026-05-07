#ifndef _COMMON_H_
#define _COMMON_H_


#ifdef __cplusplus
extern "C" {
#endif


#include <stdint.h>


#define CLOCK_SPEED_HZ               100'000'000UL
#define AUDIO_PLL_HZ                 76'800'000UL

#define HSE_VALUE_MHZ                25U
#define HSI_VALUE_MHZ                16U
#define USE_HSE                      1U

// At 100MHz, this is 100us: suitable for most cases
#define TIMEOUT_CYCLES               10'000UL

// SPI DMA has the highest priority
#define SPI_DMA_NVIC_IRQ_PRIORITY    6U
#define UART_DMA_NVIC_IRQ_PRIORITY   8U
#define I2S_DMA_NVIC_IRQ_PRIORITY    12U


typedef enum hal_err_t : uint8_t {
    // Success
    HAL_OK                        = 0x00U,

    // Generic failure
    HAL_FAIL                      = 0x01U,

    // More specific errors
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
    HAL_DMA_DME                   = 0x0DU,
    HAL_DMA_HTE                   = 0x0EU,
    HAL_DMA_ERR_UNKNOWN           = 0x0FU,
    
} hal_err_t;


const char* hal_err_to_string(hal_err_t err);


#ifdef __cplusplus
}
#endif


#endif // _COMMON_H_