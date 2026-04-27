#include "stm32f411xe.h"
#include "dma.h"


hal_err_t dmax_clk_enable(DMA_TypeDef* controller) {
    if (controller == DMA1) {
        RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
    } else if (controller == DMA2) {
        RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
    } else {
        return HAL_INVALID_ARG;
    }
    __DSB();

    return HAL_OK;
}

void dma_clear_flags(DMA_TypeDef* controller) {
    controller->LIFCR = 0xFFFFFFFFUL;
    controller->HIFCR = 0xFFFFFFFFUL;
}

void dma_clear_flags_bitmask(DMA_TypeDef* handle, uint32_t flags, bool hifcr) {
    if (hifcr) handle->HIFCR |= flags;
    else       handle->LIFCR |= flags;
}

hal_err_t dma_enable_stream(DMA_Stream_TypeDef* stream) {
    stream->CR |= DMA_SxCR_EN;
    uint32_t timeout = 1'000U;
    while (!(stream->CR & DMA_SxCR_EN) && (--timeout));
    if (timeout == 0) return HAL_TIMEOUT;
    return HAL_OK;
}

hal_err_t dma_disable_stream(DMA_Stream_TypeDef* stream) {
    stream->CR &= ~DMA_SxCR_EN;
    uint32_t timeout = 1'000U;
    while ((stream->CR & DMA_SxCR_EN) && (--timeout));
    if (timeout == 0) return HAL_TIMEOUT;
    return HAL_OK;
}

void dma_set_channel(DMA_Stream_TypeDef* stream, uint8_t channel) {
    stream->CR &= ~DMA_SxCR_CHSEL;
    stream->CR |= ((channel & 0b111U) << DMA_SxCR_CHSEL_Pos);
}

void dma_set_direct_mode(DMA_Stream_TypeDef* stream, bool direct_mode) {
    if (direct_mode) stream->FCR &= ~DMA_SxFCR_DMDIS;
    else             stream->FCR |= DMA_SxFCR_DMDIS;
}

void dma_set_trans_length(DMA_Stream_TypeDef* stream, uint16_t length) {
    stream->NDTR = length;
}

void dma_set_direction(DMA_Stream_TypeDef* stream, dma_stream_dir_t dir) {
    stream->CR &= ~DMA_SxCR_DIR;
    stream->CR |= ((uint32_t)dir << DMA_SxCR_DIR_Pos);
}

void dma_set_increment(DMA_Stream_TypeDef* stream, bool per_inc, bool mem_inc) {
    stream->CR &= ~(DMA_SxCR_PINC | DMA_SxCR_MINC);
    if (per_inc) stream->CR |= DMA_SxCR_PINC;
    if (mem_inc) stream->CR |= DMA_SxCR_MINC;
}

void dma_set_stream_priority(DMA_Stream_TypeDef* stream, dma_priority_t priority) {
    stream->CR &= ~DMA_SxCR_PL;
    stream->CR |= (((uint32_t)priority) << DMA_SxCR_PL_Pos);
}

void dma_set_flow_controller(DMA_Stream_TypeDef* stream, bool dma_is_flow_ctrler) {
    if (dma_is_flow_ctrler) {
        stream->CR &= ~DMA_SxCR_PFCTRL;
        return;
    }
    stream->CR |= DMA_SxCR_PFCTRL;
}

void dma_enable_circm_dbm(DMA_Stream_TypeDef* stream, bool ena_circ, bool ena_dbm) {
    stream->CR &= ~(DMA_SxCR_CIRC | DMA_SxCR_DBM);
    if (ena_circ) stream->CR |= DMA_SxCR_CIRC;
    if (ena_dbm)  stream->CR |= DMA_SxCR_DBM;
}

void dma_enable_irqs(DMA_Stream_TypeDef* stream, bool tc, bool te, bool hte, bool dme) {
    stream->CR &= ~(DMA_SxCR_TCIE | DMA_SxCR_TEIE | DMA_SxCR_HTIE | DMA_SxCR_DMEIE);
    if (tc)  stream->CR |= DMA_SxCR_TCIE;
    if (te)  stream->CR |= DMA_SxCR_TEIE;
    if (hte) stream->CR |= DMA_SxCR_HTIE;
    if (dme) stream->CR |= DMA_SxCR_DMEIE;
}

void dma_set_per_mem_size(DMA_Stream_TypeDef* stream, dma_data_size_t per, dma_data_size_t mem) {
    stream->CR &= ~(DMA_SxCR_PSIZE | DMA_SxCR_MSIZE);
    stream->CR |= ((uint32_t)per << DMA_SxCR_PSIZE_Pos);
    stream->CR |= ((uint32_t)mem << DMA_SxCR_MSIZE_Pos);
}

void dma_set_addresses(DMA_Stream_TypeDef* stream, const volatile void* p, const volatile void* m1, const volatile void* m2) {
    if (p)  stream->PAR  = (uint32_t)p;
    if (m1) stream->M0AR = (uint32_t)m1;
    if (m2) stream->M1AR = (uint32_t)m2;
}
