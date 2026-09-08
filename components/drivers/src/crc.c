#include "stm32f411xe.h"
#include "utils/common.h"
#include "drivers/crc.h"
#include "drivers/dma.h"
#include "utils/err.h"


// The DMA stream being used
static const dma_map_t s_crc_dma_map = {
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
    __DSB();

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

        .per_addr_incement = true,
        .mem_addr_incement = false,

        .tc_irq_enable  = true,
        .ht_irq_enable  = false,
        .te_irq_enable  = true,
        .dme_irq_enable = false,
        .fe_irq_enable  = true,

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
    __DSB();

    __disable_irq();
    s_user_callback = cb;
    s_user_data     = arg;
    __enable_irq();

    return dma_enable_stream(s_crc_dma_map.stream);
}

dma_map_t crc_get_dma_stream_info() {
    return s_crc_dma_map;
}

// DMA interrupt handler
void DMA2_Stream5_IRQHandler(void) {
    hal_err_t ret = dma_isr_helper(s_crc_dma_map.stream);

    __disable_irq();
    const crc_dma_done_cb_t local_cb  = s_user_callback;
    void* const             local_arg = s_user_data;

    s_user_callback = NULL;
    s_user_data     = NULL;
    __enable_irq();

    if (local_cb) {
        if (ret == HAL_OK) {
            local_cb(local_arg, HAL_OK, CRC->DR);
        } else {
            local_cb(local_arg, ret, 0);
        }
    }

    // Deinitialize the stream. This clears all DMA flags as well
    dma_stream_config_t stream_config = {};
    stream_config.deconfigure         = true;
    ASSERT(dma_configure_stream(s_crc_dma_map.stream, &stream_config) == HAL_OK);
}
