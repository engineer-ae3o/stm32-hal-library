#ifndef _DMA_H_
#define _DMA_H_


#ifdef __cplusplus
extern "C" {
#endif


#include "stm32f411xe.h"
#include "common.h"

#include <stdint.h>
#include <stdbool.h>


typedef enum : uint8_t {
    DMA_DIR_P_M = 0b00U,
    DMA_DIR_M_P = 0b01U,
    DMA_DIR_M_M = 0b10U
} dma_stream_dir_t;

typedef enum : uint8_t {
    DMA_PRIORITY_LOW       = 0b00U,
    DMA_PRIORITY_MEDIUM    = 0b01U,
    DMA_PRIORITY_HIGH      = 0b10U,
    DMA_PRIORITY_VERY_HIGH = 0b11U
} dma_priority_t;

typedef enum : uint8_t {
    DMA_SIZE_BYTE  = 0b00U,
    DMA_SIZE_HWORD = 0b01U,
    DMA_SIZE_WORD  = 0b10U
} dma_data_size_t;

hal_err_t dmax_clk_enable(DMA_TypeDef* controller, bool enable);
hal_err_t dma_clear_flags(DMA_TypeDef* controller, uint8_t stream);

hal_err_t dma_enable_stream(DMA_Stream_TypeDef* stream);
hal_err_t dma_disable_stream(DMA_Stream_TypeDef* stream);

void dma_set_channel(DMA_Stream_TypeDef* stream, uint8_t channel);
void dma_set_direct_mode(DMA_Stream_TypeDef* stream, bool direct_mode);
void dma_set_trans_length(DMA_Stream_TypeDef* stream, uint16_t length);
void dma_set_direction(DMA_Stream_TypeDef* stream, dma_stream_dir_t dir);
void dma_set_flow_controller(DMA_Stream_TypeDef* stream, bool dma_is_flow_ctrler);
void dma_set_stream_priority(DMA_Stream_TypeDef* stream, dma_priority_t priority);
void dma_enable_circm_dbm(DMA_Stream_TypeDef* stream, bool ena_circ, bool ena_dbm);
void dma_set_increment(DMA_Stream_TypeDef* stream, bool per_inc, bool mem_inc);
void dma_enable_irqs(DMA_Stream_TypeDef* stream, bool tc, bool te, bool hte, bool dme);
void dma_set_per_mem_size(DMA_Stream_TypeDef* stream, dma_data_size_t per, dma_data_size_t mem);
void dma_set_addresses(DMA_Stream_TypeDef* stream, const volatile void* p, const volatile void* m1, const volatile void* m2);


// Utilities for mapping the peripherals instances to DMA streams
typedef struct {
    DMA_TypeDef* controller;
    DMA_Stream_TypeDef* stream;
    IRQn_Type irq_type;
    uint8_t stream_no;
    uint8_t channel;
} dma_map_t;

typedef struct {
    const dma_map_t tx;
    const dma_map_t rx;
} dma_stream_map_t;

// Callback for DMA transmission and reception completion
typedef void (*dma_trans_done_cb_t)(void* arg, hal_err_t error);

// Callback for DMA transmission and reception completion when double buffering
typedef void (*dma_dbm_done_cb_t)(void* arg, hal_err_t error, bool is_buf_a_free);

static inline hal_err_t dma_isr_helper(DMA_Stream_TypeDef* stream, volatile uint32_t* irq_clr_rg,
                                       volatile uint32_t* irq_sta_rg, uint32_t tc, uint32_t te, uint32_t dme, uint32_t ht) {

    hal_err_t error = HAL_OK;

    // Transfer complete
    if (*irq_sta_rg & tc) {
        // Clear DMA TC interrupt bit
        *irq_clr_rg = tc;
    }
    // Transfer error
    else if (*irq_sta_rg & te) {
        // Clear DMA TE interrupt bit
        *irq_clr_rg = te;
        error = HAL_DMA_TE;
    }
    // Direct mode error
    else if (*irq_sta_rg & dme) {
        // Clear DMA DME interrupt bit
        *irq_clr_rg = dme;
        error = HAL_DMA_DME;
    }
    // Half transfer complete
    else if (*irq_sta_rg & ht) {
        // Clear DMA HT interrupt bit
        *irq_clr_rg = ht;
    }
    // Should be unreachable, but default catch-all
    else {
        // Clear DMA TC, DME, TE and HTE interrupt bits
        *irq_clr_rg = (tc | te | dme | ht);
        error = HAL_DMA_ERR_UNKNOWN;
    }
    
    // Only disable the stream if not in circular mode
    if (!(stream->CR & DMA_SxCR_CIRC)) {
        hal_err_t ret = dma_disable_stream(stream);
        if (ret != HAL_OK) error = ret;
    }

    return error;
}


#ifdef __cplusplus
}
#endif


#endif // _DMA_H_