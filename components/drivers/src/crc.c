#include "stm32f411xe.h"
#include "utils/common.h"
#include "drivers/crc.h"
#include "drivers/dma.h"
#include "utils/err.h"


// The DMA stream being used
const dma_map_t s_crc_dma_map = {
    .stream  = DMA2_Stream5,
    .channel = 0,
};

// User data
static crc_dma_done_cb_t s_user_callback = NULL;
static void*             s_user_data     = NULL;


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

    // Reset the CRC peripheral
    CRC->CR |= CRC_CR_RESET;

    // Feed all the elements into the data register
    for (size_t i = 0; i < size; i++) {
        CRC->DR = data[i];
    }

    // Get the final CRC32 checksum from the same data register
    *crc32 = CRC->DR;
    return HAL_OK;
}

hal_err_t crc_get_dma(const uint32_t* data, uint16_t size, dma_priority_t priority, crc_dma_done_cb_t cb, void* arg) {
    if (data == NULL || size == 0 || cb == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    // If s_user_callback points to a valid address, a DMA transaction is still ongoing
    if (s_user_callback) {
        return HAL_ERR_INVALID_STATE;
    }

    // Configure the DMA stream
    const dma_stream_config_t stream_config = {
        .deconfigure   = false,
        .enable_stream = false,

        .per_inc        = true,
        .mem_inc        = false,
        .tc_irq_enable  = true,
        .ht_irq_enable  = false,
        .te_irq_enable  = true,
        .dme_irq_enable = false,

        .mode            = DMA_MODE_FIFO,
        .priority        = priority,
        .direction       = DMA_DIR_M_M,
        .per_data_size   = DMA_SIZE_WORD,
        .mem_data_size   = DMA_SIZE_WORD,
        .circular_mode   = DMA_MODE_NO_CIRCULAR,
        .flow_controller = DMA_FLOW_CONTROLLER_DMA,

        .buffer_size       = size,
        .channel           = s_crc_dma_map.channel,
        .nvic_irq_priority = CRC_DMA_NVIC_IRQ_PRIORITY,

        .per_addr  = data,
        .mem_buf_0 = &CRC->DR,
        .mem_buf_1 = NULL,
    };
    TRY(dma_configure_stream(s_crc_dma_map.stream, &stream_config));

    // Reset the CRC peripheral
    CRC->CR |= CRC_CR_RESET;

    s_user_callback = cb;
    s_user_data     = arg;

    return dma_enable_stream(s_crc_dma_map.stream);
}

dma_map_t crc_get_dma_stream_info() {
    return s_crc_dma_map;
}

// DMA interrupt handler
void DMA2_Stream5_IRQHandler(void) {
    hal_err_t ret = dma_isr_helper(s_crc_dma_map.stream);
    if (s_user_callback) {
        if (ret == HAL_OK) {
            s_user_callback(s_user_data, HAL_OK, CRC->DR);
        } else {
            s_user_callback(s_user_data, ret, 0);
        }
        s_user_callback = NULL;
        s_user_data     = NULL;
    }
}
