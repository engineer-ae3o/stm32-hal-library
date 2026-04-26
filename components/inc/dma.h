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

hal_err_t dmax_clk_enable(DMA_TypeDef* handle);
void dma_clear_flags(DMA_TypeDef* handle);

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


#ifdef __cplusplus
}
#endif


#endif // _DMA_H_