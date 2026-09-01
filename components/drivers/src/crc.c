#include "stm32f411xe.h"
#include "utils/common.h"
#include "drivers/crc.h"
#include "drivers/dma.h"


// Public API
void crc_clk_enable(bool enable) {
    if (enable) {
        RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN;
    } else {
        RCC->AHB1ENR &= ~RCC_AHB1ENR_CRCEN;
    }
}

hal_err_t crc_get(const uint32_t* data, size_t size, uint32_t* crc32) {
    if (data == NULL || size == 0 || crc32 == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    CRC->CR |= CRC_CR_RESET;

    for (size_t i = 0; i < size; i++) {
        CRC->DR = data[i];
    }

    *crc32 = CRC->DR;
    return HAL_OK;
}
