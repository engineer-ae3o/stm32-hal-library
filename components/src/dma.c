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
    stream->CR |= ((channel & 0b111U) << DMA_SxCR_CHSEL_Pos);
}

void dma_enable_circular_mode(DMA_Stream_TypeDef* stream, bool enable) {
    if (enable) stream->CR |= DMA_SxCR_CIRC;
    else        stream->CR &= ~DMA_SxCR_CIRC;
}

void dma_set_direct_mode(DMA_Stream_TypeDef* stream, bool direct_mode) {
    if (direct_mode) stream->FCR &= ~DMA_SxFCR_DMDIS;
    else             stream->FCR |= DMA_SxFCR_DMDIS;
}

void dma_set_trans_length(DMA_Stream_TypeDef* stream, uint16_t length) {
    stream->NDTR = length;
}

void dma_set_direction(DMA_Stream_TypeDef* stream, dma_stream_dir_t dir) {

}

void dma_set_increment(DMA_Stream_TypeDef* stream, bool pinc_fixed, bool minc_fixed) {
    if (pinc_fixed) stream ->CR &= ~DMA_SxCR_PINC;
    else            stream->CR |= DMA_SxCR_PINC;
    if (minc_fixed) stream->CR &= ~DMA_SxCR_MINC;
    else            stream->CR |= DMA_SxCR_MINC;
}

void dma_set_stream_priority(DMA_Stream_TypeDef* stream, uint8_t priority) {
    stream->CR |= ((priority & 0b11U) << DMA_SxCR_PL_Pos);
}

void dma_set_flow_controller(DMA_Stream_TypeDef* stream, bool dma_is_flow_ctrler) {
    if (dma_is_flow_ctrler) {
        stream->CR &= ~DMA_SxCR_PFCTRL;
        return;
    }
    stream->CR |= DMA_SxCR_PFCTRL;
}

void dma_enable_irqs(DMA_Stream_TypeDef* stream, bool tc, bool te, bool hte, bool dme) {
    stream->CR &= ~(DMA_SxCR_TCIE | DMA_SxCR_TEIE | DMA_SxCR_HTIE | DMA_SxCR_DMEIE);
    if (tc)  stream->CR |= DMA_SxCR_TCIE;
    if (te)  stream->CR |= DMA_SxCR_TEIE;
    if (hte) stream->CR |= DMA_SxCR_HTIE;
    if (dme) stream->CR |= DMA_SxCR_DMEIE;
}

void dma_set_addresses(DMA_Stream_TypeDef* stream, uint8_t* p, uint8_t* m1, uint8_t* m2) {
    if (p)  stream->PAR  = (uint32_t)p;
    if (m1) stream->M0AR = (uint32_t)m1;
    if (m2) {
        // Enable Double buffering
        stream->CR |= DMA_SxCR_DBM;
        stream->M1AR = (uint32_t)m2;
    }
}

void dma_set_per_mem_size(DMA_Stream_TypeDef* stream, dma_data_size_t per, dma_data_size_t mem) {

}
