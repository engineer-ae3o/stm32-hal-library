#include "utils/common.h"
#include "utils/err.h"


// Convert error codes to string
const char* err_code_lut[] = {
    [HAL_OK]                      = "HAL_OK",
    [HAL_FAIL]                    = "HAL_FAIL",
    [HAL_INVALID_ARG]             = "HAL_INVALID_ARG",
    [HAL_INVALID_STATE]           = "HAL_INVALID_STATE",
    [HAL_TIMEOUT]                 = "HAL_TIMEOUT",
    [HAL_TX_ERROR]                = "HAL_TX_ERROR",
    [HAL_RX_ERROR]                = "HAL_RX_ERROR",
    [HAL_I2C_DEVICE_NOT_FOUND]    = "HAL_I2C_DEVICE_NOT_FOUND",
    [HAL_I2C_ARBITRATION_LOST]    = "HAL_I2C_ARBITRATION_LOST",
    [HAL_SPI_TXE_FAILED_TO_SET]   = "HAL_SPI_TXE_FAILED_TO_SET",
    [HAL_SPI_BSY_FAILED_TO_CLEAR] = "HAL_SPI_BSY_FAILED_TO_CLEAR",
    [HAL_UART_TC_FAILED_TO_SET]   = "HAL_UART_TC_FAILED_TO_SET",
    [HAL_DMA_TE]                  = "HAL_DMA_TE",
    [HAL_DMA_DME]                 = "HAL_DMA_DME",
    [HAL_DMA_ERR_UNKNOWN]         = "HAL_DMA_ERR_UNKNOWN",
};

const char* hal_err_to_string(hal_err_t err) {
    ASSERT(err < (sizeof(err_code_lut) / sizeof(err_code_lut[0])));
    return err_code_lut[err];
}
