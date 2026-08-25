#include "utils/common.h"
#include "utils/err.h"


// Convert error codes to string
static const char* s_err_code_lut[] = {
    [HAL_OK]                          = "HAL_OK",
    [HAL_FAIL]                        = "HAL_FAIL",
    [HAL_ERR_INVALID_ARG]             = "HAL_ERR_INVALID_ARG",
    [HAL_ERR_INVALID_STATE]           = "HAL_ERR_INVALID_STATE",
    [HAL_ERR_TIMEOUT]                 = "HAL_ERR_TIMEOUT",
    [HAL_ERR_NOT_DONE]                = "HAL_ERR_NOT_DONE",
    [HAL_ERR_NOT_FOUND]               = "HAL_ERR_NOT_FOUND",
    [HAL_ERR_TX]                      = "HAL_ERR_TX",
    [HAL_ERR_RX]                      = "HAL_ERR_RX",
    [HAL_ERR_I2C_DEVICE_NOT_FOUND]    = "HAL_ERR_I2C_DEVICE_NOT_FOUND",
    [HAL_ERR_I2C_ARBITRATION_LOST]    = "HAL_ERR_I2C_ARBITRATION_LOST",
    [HAL_ERR_SPI_TXE_FAILED_TO_SET]   = "HAL_ERR_SPI_TXE_FAILED_TO_SET",
    [HAL_ERR_SPI_BSY_FAILED_TO_CLEAR] = "HAL_ERR_SPI_BSY_FAILED_TO_CLEAR",
    [HAL_ERR_UART_TC_FAILED_TO_SET]   = "HAL_ERR_UART_TC_FAILED_TO_SET",
    [HAL_ERR_DMA_TE]                  = "HAL_ERR_DMA_TE",
    [HAL_ERR_DMA_DME]                 = "HAL_ERR_DMA_DME",
    [HAL_ERR_DMA_ERR_UNKNOWN]         = "HAL_ERR_DMA_ERR_UNKNOWN",
};

_Static_assert(HAL_ERR_COUNT == ARRAY_SIZE(s_err_code_lut));

const char* hal_err_to_string(hal_err_t err) {
    ASSERT(err < ARRAY_SIZE(s_err_code_lut));
    return s_err_code_lut[err];
}
