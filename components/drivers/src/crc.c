#include "stm32f411xe.h"
#include "utils/common.h"
#include "drivers/crc.h"
#include "drivers/dma.h"
#include "utils/err.h"


// The DMA stream being used
#define CRC_DMA_CONTROLLER DMA2
#define CRC_DMA_STREAM DMA2_Stream5
#define CRC_DMA_STREAM_NO 0
#define CRC_DMA_CHANNEL 0
#define CRC_DMA_IRQ_TYPE DMA2_Stream5_IRQn

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

hal_err_t crc_get_dma(const uint32_t* data, uint16_t size, crc_dma_done_cb_t cb, void* arg) {
    if (data == NULL || size == 0 || cb == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    // If s_user_callback points to a valid address, a DMA transaction is still ongoing
    if (s_user_callback) {
        return HAL_ERR_INVALID_STATE;
    }

    // Configuration
    TRY(dmax_clk_enable(CRC_DMA_CONTROLLER, true));
    TRY(dma_clear_flags(CRC_DMA_CONTROLLER, CRC_DMA_STREAM_NO));
    TRY(dma_disable_stream(CRC_DMA_STREAM));
    dma_set_channel(CRC_DMA_STREAM, CRC_DMA_CHANNEL);
    dma_set_stream_priority(CRC_DMA_STREAM, DMA_PRIORITY_MEDIUM);
    dma_set_direction(CRC_DMA_STREAM, DMA_DIR_M_M);
    dma_enable_irqs(CRC_DMA_STREAM, true, true, false, false);
    dma_set_increment(CRC_DMA_STREAM, true, false);
    dma_set_per_mem_size(CRC_DMA_STREAM, DMA_SIZE_WORD, DMA_SIZE_WORD);
    dma_enable_circm_dbm(CRC_DMA_STREAM, false, false);
    dma_set_flow_controller(CRC_DMA_STREAM, true);
    dma_set_direct_mode(CRC_DMA_STREAM, false);
    dma_set_addresses(CRC_DMA_STREAM, data, &CRC->DR, NULL);
    dma_set_trans_length(CRC_DMA_STREAM, size);

    // Enable the DMA stream interrupts
    NVIC_SetPriority(CRC_DMA_IRQ_TYPE, CRC_DMA_NVIC_IRQ_PRIORITY);
    NVIC_EnableIRQ(CRC_DMA_IRQ_TYPE);

    // Reset the CRC peripheral
    CRC->CR |= CRC_CR_RESET;

    s_user_callback = cb;
    s_user_data     = arg;

    return dma_enable_stream(CRC_DMA_STREAM);
}

dma_map_t crc_get_dma_stream_info() {
    const dma_map_t info = {
        .controller = CRC_DMA_CONTROLLER,
        .stream     = CRC_DMA_STREAM,
        .irq_type   = CRC_DMA_IRQ_TYPE,
        .stream_no  = CRC_DMA_STREAM_NO,
        .channel    = CRC_DMA_CHANNEL,
    };
    return info;
}

// DMA interrupt handler
void DMA2_Stream5_IRQHandler(void) {
    hal_err_t ret = dma_isr_helper(DMA2_Stream5, &DMA2->HIFCR, &DMA2->HISR, DMA_HISR_TCIF5, DMA_HISR_TEIF5, DMA_HISR_DMEIF5, DMA_HISR_HTIF5);
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
