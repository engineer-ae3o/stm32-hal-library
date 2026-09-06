#ifndef DMA_H_
#define DMA_H_


#ifdef __cplusplus
extern "C" {
#endif


#include "stm32f411xe.h"
#include "drivers/dma_types.h"
#include "utils/common.h"
#include "utils/err.h"

#include <stdint.h>
#include <stddef.h>


hal_err_t dmax_clk_enable(DMA_TypeDef* controller, bool enable);

hal_err_t dma_enable_stream(DMA_Stream_TypeDef* stream);
hal_err_t dma_disable_stream(DMA_Stream_TypeDef* stream);
hal_err_t dma_get_stream_flags(DMA_Stream_TypeDef* stream, dma_stream_flags_t* flags);
hal_err_t dma_get_stream_info(DMA_Stream_TypeDef* stream, dma_stream_info_t* stream_info);
hal_err_t dma_configure_stream(DMA_Stream_TypeDef* stream, const dma_stream_config_t* config);

void dma_set_channel(DMA_Stream_TypeDef* stream, uint32_t channel);
void dma_set_direct_mode(DMA_Stream_TypeDef* stream, bool direct_mode);
void dma_set_trans_length(DMA_Stream_TypeDef* stream, uint16_t length);
void dma_set_direction(DMA_Stream_TypeDef* stream, dma_direction_t dir);
void dma_set_increment(DMA_Stream_TypeDef* stream, bool per_inc, bool mem_inc);
void dma_set_flow_controller(DMA_Stream_TypeDef* stream, bool dma_is_flow_ctrler);
void dma_set_stream_priority(DMA_Stream_TypeDef* stream, dma_priority_t priority);
void dma_enable_circm_dbm(DMA_Stream_TypeDef* stream, bool ena_circm, bool ena_dbm);
void dma_set_per_mem_size(DMA_Stream_TypeDef* stream, dma_data_size_t per, dma_data_size_t mem);
void dma_enable_irqs(DMA_Stream_TypeDef* stream, bool tc_mask, bool te_mask, bool ht_mask, bool dme_mask);
void dma_set_addresses(DMA_Stream_TypeDef* stream, const volatile void* per, const volatile void* mem_0, const volatile void* mem_1);

// Helper to abstract checking and clearing of DMA irq flags
[[__gnu__::__always_inline__]] static inline hal_err_t dma_isr_helper(DMA_Stream_TypeDef* stream) {
    if (stream == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    // Get the DMA status flags for this stream and the corresponding status and irq clear register
    dma_stream_flags_t flags;
    TRY(dma_get_stream_flags(stream, &flags));

    const uint32_t status         = *flags.irq_status_register;
    uint32_t       flags_to_clear = 0;

    hal_err_t error = HAL_OK;

    // Transfer complete
    if (status & flags.tc_mask) {
        flags_to_clear |= flags.tc_mask;
    }

    // Transfer error
    if (status & flags.te_mask) {
        flags_to_clear |= flags.te_mask;
        error = HAL_ERR_DMA_TE;
    }

    // Direct mode error
    if (status & flags.dme_mask) {
        flags_to_clear |= flags.dme_mask;
        error = HAL_ERR_DMA_DME;
    }

    // Half transfer complete
    if (status & flags.ht_mask) {
        flags_to_clear |= flags.ht_mask;
    }

    // FIFO mode error
    if (status & flags.fe_mask) {
        flags_to_clear |= flags.fe_mask;
        error = HAL_ERR_DMA_FE;
    }

    // Clear all the set flags
    *flags.irq_clear_register = flags_to_clear;

    // Return if the stream is in circular mode or half transfer,
    // so as not to disable the DMA strean. Everything else should
    // disable the stream since not being used till the next transfer
    if (stream->CR & (DMA_SxCR_CIRC | DMA_SxCR_HTIE | DMA_SxCR_DBM)) {
        return error;
    }

    TRY(dma_disable_stream(stream));
    return error;
}


#ifdef __cplusplus
}
#endif


#endif // DMA_H_