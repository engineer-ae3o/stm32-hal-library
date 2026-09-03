#include "stm32f411xe.h"
#include "utils/common.h"
#include "drivers/dma.h"
#include "utils/err.h"


hal_err_t dmax_clk_enable(DMA_TypeDef* controller, bool enable) {
    if (enable) {
        if (controller == DMA1) {
            RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
        } else if (controller == DMA2) {
            RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
        } else {
            return HAL_ERR_INVALID_ARG;
        }

    } else {
        if (controller == DMA1) {
            RCC->AHB1ENR &= ~RCC_AHB1ENR_DMA1EN;
        } else if (controller == DMA2) {
            RCC->AHB1ENR &= ~RCC_AHB1ENR_DMA2EN;
        } else {
            return HAL_ERR_INVALID_ARG;
        }
    }

    __DSB();
    return HAL_OK;
}

hal_err_t dma_clear_flags(DMA_TypeDef* controller, uint8_t stream) {
    if (controller == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    uint32_t flags = 0;
    switch (stream) {
        case 0:
            flags = (DMA_LISR_TCIF0 | DMA_LISR_HTIF0 | DMA_LISR_TEIF0 | DMA_LISR_DMEIF0 | DMA_LISR_FEIF0);
            break;
        case 1:
            flags = (DMA_LISR_TCIF1 | DMA_LISR_HTIF1 | DMA_LISR_TEIF1 | DMA_LISR_DMEIF1 | DMA_LISR_FEIF1);
            break;
        case 2:
            flags = (DMA_LISR_TCIF2 | DMA_LISR_HTIF2 | DMA_LISR_TEIF2 | DMA_LISR_DMEIF2 | DMA_LISR_FEIF2);
            break;
        case 3:
            flags = (DMA_LISR_TCIF3 | DMA_LISR_HTIF3 | DMA_LISR_TEIF3 | DMA_LISR_DMEIF3 | DMA_LISR_FEIF3);
            break;
        case 4:
            flags = (DMA_HISR_TCIF4 | DMA_HISR_HTIF4 | DMA_HISR_TEIF4 | DMA_HISR_DMEIF4 | DMA_HISR_FEIF4);
            break;
        case 5:
            flags = (DMA_HISR_TCIF5 | DMA_HISR_HTIF5 | DMA_HISR_TEIF5 | DMA_HISR_DMEIF5 | DMA_HISR_FEIF5);
            break;
        case 6:
            flags = (DMA_HISR_TCIF6 | DMA_HISR_HTIF6 | DMA_HISR_TEIF6 | DMA_HISR_DMEIF6 | DMA_HISR_FEIF6);
            break;
        case 7:
            flags = (DMA_HISR_TCIF7 | DMA_HISR_HTIF7 | DMA_HISR_TEIF7 | DMA_HISR_DMEIF7 | DMA_HISR_FEIF7);
            break;
        default:
            return HAL_ERR_INVALID_ARG;
    }

    if (stream <= 3) {
        controller->LIFCR = flags;
    } else if (stream <= 7) {
        controller->HIFCR = flags;
    }

    return HAL_OK;
}

hal_err_t dma_enable_stream(DMA_Stream_TypeDef* stream) {
    if (stream == NULL) {
        return HAL_ERR_INVALID_ARG;
    }
    stream->CR |= DMA_SxCR_EN;
    uint32_t timeout = TIMEOUT_CYCLES;
    while (!(stream->CR & DMA_SxCR_EN) && (--timeout));
    if (timeout == 0) {
        return HAL_ERR_TIMEOUT;
    }
    return HAL_OK;
}

hal_err_t dma_disable_stream(DMA_Stream_TypeDef* stream) {
    if (stream == NULL) {
        return HAL_ERR_INVALID_ARG;
    }
    stream->CR &= ~DMA_SxCR_EN;
    uint32_t timeout = TIMEOUT_CYCLES;
    while ((stream->CR & DMA_SxCR_EN) && (--timeout));
    if (timeout == 0) {
        return HAL_ERR_TIMEOUT;
    }
    return HAL_OK;
}

void dma_set_channel(DMA_Stream_TypeDef* stream, uint8_t channel) {
    if (stream) {
        stream->CR = (stream->CR & ~DMA_SxCR_CHSEL) | ((channel & 0b111U) << DMA_SxCR_CHSEL_Pos);
    }
}

void dma_set_direct_mode(DMA_Stream_TypeDef* stream, bool direct_mode) {
    if (stream) {
        if (direct_mode) {
            stream->FCR &= ~DMA_SxFCR_DMDIS;
        } else {
            stream->FCR |= DMA_SxFCR_DMDIS;
        }
    }
}

void dma_set_trans_length(DMA_Stream_TypeDef* stream, uint16_t length) {
    if (stream) {
        stream->NDTR = length;
    }
}

void dma_set_direction(DMA_Stream_TypeDef* stream, dma_stream_dir_t dir) {
    if (stream) {
        stream->CR = (stream->CR & ~DMA_SxCR_DIR) | ((uint32_t)dir << DMA_SxCR_DIR_Pos);
    }
}

void dma_set_increment(DMA_Stream_TypeDef* stream, bool per_inc, bool mem_inc) {
    if (stream) {
        uint32_t mask = stream->CR & ~(DMA_SxCR_PINC | DMA_SxCR_MINC);
        mask |= per_inc ? DMA_SxCR_PINC : 0;
        mask |= mem_inc ? DMA_SxCR_MINC : 0;
        stream->CR = mask;
    }
}

void dma_set_stream_priority(DMA_Stream_TypeDef* stream, dma_priority_t priority) {
    if (stream) {
        stream->CR = (stream->CR & ~DMA_SxCR_PL) | (((uint32_t)priority) << DMA_SxCR_PL_Pos);
    }
}

void dma_set_flow_controller(DMA_Stream_TypeDef* stream, bool dma_is_flow_ctrler) {
    if (stream) {
        if (dma_is_flow_ctrler) {
            stream->CR &= ~DMA_SxCR_PFCTRL;
        } else {
            stream->CR |= DMA_SxCR_PFCTRL;
        }
    }
}

void dma_enable_circm_dbm(DMA_Stream_TypeDef* stream, bool ena_circm, bool ena_dbm) {
    if (stream) {
        uint32_t mask = stream->CR & ~(DMA_SxCR_CIRC | DMA_SxCR_DBM);
        mask |= ena_circm ? DMA_SxCR_CIRC : 0;
        mask |= ena_dbm ? DMA_SxCR_DBM : 0;
        stream->CR = mask;
    }
}

void dma_enable_irqs(DMA_Stream_TypeDef* stream, bool tc, bool te, bool ht, bool dme) {
    if (stream) {
        uint32_t mask = stream->CR & ~(DMA_SxCR_TCIE | DMA_SxCR_TEIE | DMA_SxCR_HTIE | DMA_SxCR_DMEIE);
        mask |= tc ? DMA_SxCR_TCIE : 0;
        mask |= te ? DMA_SxCR_TEIE : 0;
        mask |= ht ? DMA_SxCR_HTIE : 0;
        mask |= dme ? DMA_SxCR_DMEIE : 0;
        stream->CR = mask;
    }
}

void dma_set_per_mem_size(DMA_Stream_TypeDef* stream, dma_data_size_t per, dma_data_size_t mem) {
    if (stream) {
        stream->CR =
            (stream->CR & ~(DMA_SxCR_PSIZE | DMA_SxCR_MSIZE)) | ((uint32_t)per << DMA_SxCR_PSIZE_Pos) | ((uint32_t)mem << DMA_SxCR_MSIZE_Pos);
    }
}

void dma_set_addresses(DMA_Stream_TypeDef* stream, const volatile void* per, const volatile void* mem_0, const volatile void* mem_1) {
    if (stream) {
        if (per) {
            stream->PAR = (uint32_t)per;
        }
        if (mem_0) {
            stream->M0AR = (uint32_t)mem_0;
        }
        if (mem_1) {
            stream->M1AR = (uint32_t)mem_1;
        }
    }
}
