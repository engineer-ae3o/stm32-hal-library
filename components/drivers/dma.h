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
hal_err_t dma_clear_stream_flags(DMA_Stream_TypeDef* stream);
hal_err_t get_stream_info(DMA_Stream_TypeDef* stream, stream_info_t* stream_info);
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
[[__gnu__::__always_inline__]] inline hal_err_t dma_isr_helper(DMA_Stream_TypeDef* stream) {
    if (stream == NULL || irq_clear_register == NULL || irq_status_register == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    hal_err_t error = HAL_OK;

    // irq_status_register is the DMA interrupt status register.
    // When a bit gets set, the corresponding DMA interrupt is fired.

    // irq_clear_register is the DMA interrupt clear register. It is used to clear the
    // interrupt bit set in its corresponding status register. It is write 1 to clear a bit.

    const uint32_t status_register = *irq_status_register;
    uint32_t       clear_register  = 0;

    // Transfer complete
    if (status_register & tc_mask) {
        // Clear DMA TC interrupt bit
        clear_register |= tc_mask;
    }

    // Transfer error
    if (status_register & te_mask) {
        // Clear DMA TE interrupt bit
        clear_register |= te_mask;
        error = HAL_ERR_DMA_TE;
    }

    // Direct mode error
    if (status_register & dme_mask) {
        // Clear DMA DME interrupt bit
        clear_register |= dme_mask;
        error = HAL_ERR_DMA_DME;
    }

    // Half transfer complete
    if (status_register & ht_mask) {
        // Clear DMA HT interrupt bit
        clear_register |= ht_mask;
    }

    *irq_clear_register = clear_register;

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