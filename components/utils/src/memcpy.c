#include "stm32f411xe.h"
#include "drivers/dma_types.h"
#include "utils/memcpy.h"
#include "drivers/dma.h"
#include "utils/err.h"


// The DMA stream being used
static const dma_map_t s_memcpy_dma_map = {
    .stream  = DMA2_Stream3,
    .channel = 0,
};

// User data
static dma_done_cb_t s_user_callback = NULL;
static void*         s_user_data     = NULL;

static volatile memcpy_state_t* s_dma_done_flag = NULL;


// Public API
hal_err_t dma_memcpy(void* dest, const void* src, uint16_t len, volatile memcpy_state_t* dma_done_flag) {
    if (dest == NULL || src == NULL || len == 0 || dma_done_flag == NULL || dest == src) {
        return HAL_ERR_INVALID_ARG;
    }

    if (s_user_callback || s_dma_done_flag) {
        // A DMA transaction is still ongoing
        return HAL_ERR_INVALID_STATE;
    }

    // Configure the DMA stream
    const dma_stream_config_t stream_config = {
        .deconfigure   = false,
        .enable_stream = false,

        .per_addr_incement = true,
        .mem_addr_incement = true,

        .tc_irq_enable  = true,
        .ht_irq_enable  = false,
        .te_irq_enable  = true,
        .dme_irq_enable = false,
        .fe_irq_enable  = true,

        .mode            = DMA_MODE_FIFO,
        .priority        = DMA_PRIORITY_LOW,
        .direction       = DMA_DIR_M2M,
        .per_data_size   = DMA_SIZE_BYTE,
        .mem_data_size   = DMA_SIZE_BYTE,
        .circular_mode   = DMA_MODE_NO_CIRCULAR,
        .flow_controller = DMA_FLOW_CONTROLLER_DMA,

        .buffer_size       = len,
        .channel           = s_memcpy_dma_map.channel,
        .nvic_irq_priority = M2M_DMA_NVIC_IRQ_PRIORITY,

        .per_addr  = src,
        .mem_buf_0 = dest,
        .mem_buf_1 = NULL,
    };
    TRY(dma_configure_stream(s_memcpy_dma_map.stream, &stream_config));

    s_dma_done_flag  = dma_done_flag;
    *s_dma_done_flag = DMA_MEMCPY_NOT_DONE;

    return dma_enable_stream(s_memcpy_dma_map.stream);
}

hal_err_t dma_memcpy_cb(void* dest, const void* src, uint16_t len, dma_done_cb_t cb, void* arg) {
    if (dest == NULL || src == NULL || len == 0 || cb == NULL || dest == src) {
        return HAL_ERR_INVALID_ARG;
    }

    if (s_user_callback || s_dma_done_flag) {
        // A DMA transaction is still ongoing
        return HAL_ERR_INVALID_STATE;
    }

    // Configure the DMA stream
    const dma_stream_config_t stream_config = {
        .deconfigure   = false,
        .enable_stream = false,

        .per_addr_incement = true,
        .mem_addr_incement = true,

        .tc_irq_enable  = true,
        .ht_irq_enable  = false,
        .te_irq_enable  = true,
        .dme_irq_enable = false,
        .fe_irq_enable  = true,

        .mode            = DMA_MODE_FIFO,
        .priority        = DMA_PRIORITY_LOW,
        .direction       = DMA_DIR_M2M,
        .per_data_size   = DMA_SIZE_BYTE,
        .mem_data_size   = DMA_SIZE_BYTE,
        .circular_mode   = DMA_MODE_NO_CIRCULAR,
        .flow_controller = DMA_FLOW_CONTROLLER_DMA,

        .buffer_size       = len,
        .channel           = s_memcpy_dma_map.channel,
        .nvic_irq_priority = M2M_DMA_NVIC_IRQ_PRIORITY,

        .per_addr  = src,
        .mem_buf_0 = dest,
        .mem_buf_1 = NULL,
    };
    TRY(dma_configure_stream(s_memcpy_dma_map.stream, &stream_config));

    s_user_callback = cb;
    s_user_data     = arg;

    return dma_enable_stream(s_memcpy_dma_map.stream);
}

memcpy_state_t dma_memcpy_wait_for_flag(volatile memcpy_state_t* dma_done_flag, uint32_t timeout) {
    while ((*dma_done_flag == DMA_MEMCPY_NOT_DONE) && --timeout);
    if (timeout == 0) {
        return DMA_MEMCPY_TIMEOUT;
    }
    return *dma_done_flag;
}

dma_map_t dma_memcpy_get_stream_info(void) {
    return s_memcpy_dma_map;
}

void DMA2_Stream3_IRQHandler(void) {
    hal_err_t ret = dma_isr_helper(s_memcpy_dma_map.stream);

    const dma_done_cb_t local_cb  = s_user_callback;
    void* const         local_arg = s_user_data;

    volatile memcpy_state_t* const dma_done_flag = s_dma_done_flag;

    s_user_callback = NULL;
    s_user_data     = NULL;
    s_dma_done_flag = NULL;

    if (local_cb) {
        local_cb(local_arg, ret);
    }

    if (dma_done_flag) {
        if (ret == HAL_OK) {
            *dma_done_flag = DMA_MEMCPY_DONE;
        } else {
            *dma_done_flag = DMA_MEMCPY_ERROR;
        }
    }

    dma_stream_config_t stream_config;
    stream_config.deconfigure = true;
    ASSERT(dma_configure_stream(s_memcpy_dma_map.stream, &stream_config) == HAL_OK);
}
