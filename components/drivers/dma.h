#ifndef DMA_H_
#define DMA_H_


#ifdef __cplusplus
extern "C" {
#endif


#include "stm32f411xe.h"
#include "utils/common.h"
#include "utils/err.h"

#include <stdint.h>
#include <stddef.h>


typedef enum : uint8_t {
    DMA_DIR_P_M = 0b00, // Peripheral to memory
    DMA_DIR_M_P = 0b01, // Memory to peripheral
    DMA_DIR_M_M = 0b10, // Memory to memory
} dma_stream_dir_t;

typedef enum : uint8_t {
    DMA_PRIORITY_LOW       = 0b00,
    DMA_PRIORITY_MEDIUM    = 0b01,
    DMA_PRIORITY_HIGH      = 0b10,
    DMA_PRIORITY_VERY_HIGH = 0b11,
} dma_priority_t;

typedef enum : uint8_t {
    DMA_SIZE_BYTE  = 0b00, // Data size is a byte
    DMA_SIZE_HWORD = 0b01, // Data size is a half word (2 bytes)
    DMA_SIZE_WORD  = 0b10, // Data size is a word (4 bytes)
} dma_data_size_t;

hal_err_t dmax_clk_enable(DMA_TypeDef* controller, bool enable);
hal_err_t dma_clear_flags(DMA_TypeDef* controller, uint8_t stream);

hal_err_t dma_enable_stream(DMA_Stream_TypeDef* stream);
hal_err_t dma_disable_stream(DMA_Stream_TypeDef* stream);

void dma_set_channel(DMA_Stream_TypeDef* stream, uint8_t channel);
void dma_set_direct_mode(DMA_Stream_TypeDef* stream, bool direct_mode);
void dma_set_trans_length(DMA_Stream_TypeDef* stream, uint16_t length);
void dma_set_direction(DMA_Stream_TypeDef* stream, dma_stream_dir_t dir);
void dma_set_increment(DMA_Stream_TypeDef* stream, bool per_inc, bool mem_inc);
void dma_set_flow_controller(DMA_Stream_TypeDef* stream, bool dma_is_flow_ctrler);
void dma_set_stream_priority(DMA_Stream_TypeDef* stream, dma_priority_t priority);
void dma_enable_circm_dbm(DMA_Stream_TypeDef* stream, bool ena_circm, bool ena_dbm);
void dma_set_per_mem_size(DMA_Stream_TypeDef* stream, dma_data_size_t per, dma_data_size_t mem);
void dma_enable_irqs(DMA_Stream_TypeDef* stream, bool tc_mask, bool te_mask, bool ht_mask, bool dme_mask);
void dma_set_addresses(DMA_Stream_TypeDef* stream, const volatile void* per, const volatile void* mem_0, const volatile void* mem_1);

// Utilities for mapping the peripherals instances to DMA streams
typedef struct {
    DMA_TypeDef*        controller;
    DMA_Stream_TypeDef* stream;
    IRQn_Type           irq_type;
    uint8_t             stream_no;
    uint8_t             channel;
} dma_map_t;

typedef struct {
    dma_map_t tx;
    dma_map_t rx;
} dma_stream_map_t;

// Callback for DMA transmission and reception completion
typedef void (*dma_done_cb_t)(void* arg, hal_err_t error);

// Helper to abstract checking and clearing of DMA irq flags
[[__gnu__::__always_inline__]] inline hal_err_t dma_isr_helper(DMA_Stream_TypeDef* stream,
                                                               volatile uint32_t*  irq_clear_register,
                                                               volatile uint32_t*  irq_status_register,
                                                               uint32_t            tc_mask,
                                                               uint32_t            te_mask,
                                                               uint32_t            dme_mask,
                                                               uint32_t            ht_mask) {
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