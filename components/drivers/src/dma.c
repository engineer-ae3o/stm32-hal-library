#include "stm32f411xe.h"
#include "utils/common.h"
#include "drivers/dma.h"
#include "utils/err.h"


// Helper utilities
typedef struct {
    DMA_Stream_TypeDef* stream;
    DMA_TypeDef*        controller;
    uint32_t            stream_number;
    IRQn_Type           nvic_irq_type;
} stream_info_t;

static inline hal_err_t get_stream_info(DMA_Stream_TypeDef* stream, stream_info_t* stream_info) {
    if (stream_info == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    static const stream_info_t stream_info_lut[] = {
        {.stream = DMA1_Stream0, .controller = DMA1, .stream_number = 0, .nvic_irq_type = DMA1_Stream0_IRQn},
        {.stream = DMA1_Stream1, .controller = DMA1, .stream_number = 1, .nvic_irq_type = DMA1_Stream1_IRQn},
        {.stream = DMA1_Stream2, .controller = DMA1, .stream_number = 2, .nvic_irq_type = DMA1_Stream2_IRQn},
        {.stream = DMA1_Stream3, .controller = DMA1, .stream_number = 3, .nvic_irq_type = DMA1_Stream3_IRQn},
        {.stream = DMA1_Stream4, .controller = DMA1, .stream_number = 4, .nvic_irq_type = DMA1_Stream4_IRQn},
        {.stream = DMA1_Stream5, .controller = DMA1, .stream_number = 5, .nvic_irq_type = DMA1_Stream5_IRQn},
        {.stream = DMA1_Stream6, .controller = DMA1, .stream_number = 6, .nvic_irq_type = DMA1_Stream6_IRQn},
        {.stream = DMA1_Stream7, .controller = DMA1, .stream_number = 7, .nvic_irq_type = DMA1_Stream7_IRQn},
        {.stream = DMA2_Stream0, .controller = DMA2, .stream_number = 0, .nvic_irq_type = DMA2_Stream0_IRQn},
        {.stream = DMA2_Stream1, .controller = DMA2, .stream_number = 1, .nvic_irq_type = DMA2_Stream1_IRQn},
        {.stream = DMA2_Stream2, .controller = DMA2, .stream_number = 2, .nvic_irq_type = DMA2_Stream2_IRQn},
        {.stream = DMA2_Stream3, .controller = DMA2, .stream_number = 3, .nvic_irq_type = DMA2_Stream3_IRQn},
        {.stream = DMA2_Stream4, .controller = DMA2, .stream_number = 4, .nvic_irq_type = DMA2_Stream4_IRQn},
        {.stream = DMA2_Stream5, .controller = DMA2, .stream_number = 5, .nvic_irq_type = DMA2_Stream5_IRQn},
        {.stream = DMA2_Stream6, .controller = DMA2, .stream_number = 6, .nvic_irq_type = DMA2_Stream6_IRQn},
        {.stream = DMA2_Stream7, .controller = DMA2, .stream_number = 7, .nvic_irq_type = DMA2_Stream7_IRQn},
    };

    for (size_t i = 0; i < ARRAY_SIZE(stream_info_lut); i++) {
        if (stream == stream_info_lut[i].stream) {
            *stream_info = stream_info_lut[i];
            return HAL_OK;
        }
    }

    return HAL_ERR_INVALID_ARG;
}


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

hal_err_t dma_clear_stream_flags(DMA_Stream_TypeDef* stream) {
    if (stream == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    stream_info_t stream_info;
    TRY(get_stream_info(stream, &stream_info));

    uint32_t flags = 0;
    switch (stream_info.stream_number) {
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

    if (stream_info.stream_number <= 3) {
        stream_info.controller->LIFCR = flags;
    } else if (stream_info.stream_number <= 7) {
        stream_info.controller->HIFCR = flags;
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

hal_err_t dma_configure_stream(DMA_Stream_TypeDef* stream, const dma_stream_config_t* config) {
    if (stream == NULL || config == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    stream_info_t stream_info;
    TRY(get_stream_info(stream, &stream_info));

    TRY(dmax_clk_enable(stream_info.controller, true));
    TRY(dma_clear_stream_flags(stream));
    TRY(dma_disable_stream(stream));

    stream->CR &= ~(DMA_SxCR_CHSEL | DMA_SxCR_MBURST | DMA_SxCR_PBURST | DMA_SxCR_CT | DMA_SxCR_DBM | DMA_SxCR_PL | DMA_SxCR_PINCOS | DMA_SxCR_MSIZE |
                    DMA_SxCR_PSIZE | DMA_SxCR_MINC | DMA_SxCR_PINC | DMA_SxCR_CIRC | DMA_SxCR_DIR | DMA_SxCR_PFCTRL | DMA_SxCR_TCIE | DMA_SxCR_HTIE |
                    DMA_SxCR_TEIE | DMA_SxCR_DMEIE);
    stream->FCR &= ~(DMA_SxFCR_FEIE | DMA_SxFCR_DMDIS | DMA_SxFCR_FTH);

    if (config->deconfigure) {
        stream->PAR  = 0;
        stream->M0AR = 0;
        stream->M1AR = 0;
        stream->NDTR = 0;
        NVIC_DisableIRQ(stream_info.nvic_irq_type);
        return HAL_OK;
    }

    uint32_t cr_mask = stream->CR;
    cr_mask |= config->per_inc ? DMA_SxCR_PINC : 0;
    cr_mask |= config->mem_inc ? DMA_SxCR_MINC : 0;
    cr_mask |= config->tc_irq_enable ? DMA_SxCR_TCIE : 0;
    cr_mask |= config->te_irq_enable ? DMA_SxCR_TEIE : 0;
    cr_mask |= config->ht_irq_enable ? DMA_SxCR_HTIE : 0;
    cr_mask |= config->dme_irq_enable ? DMA_SxCR_DMEIE : 0;
    cr_mask |= (uint32_t)(config->priority << DMA_SxCR_PL_Pos) & DMA_SxCR_PL;
    cr_mask |= (uint32_t)(config->direction << DMA_SxCR_DIR_Pos) & DMA_SxCR_DIR;
    cr_mask |= (uint32_t)(config->channel << DMA_SxCR_CHSEL_Pos) & DMA_SxCR_CHSEL;
    cr_mask |= (uint32_t)(config->per_data_size << DMA_SxCR_PSIZE_Pos) & DMA_SxCR_PSIZE;
    cr_mask |= (uint32_t)(config->mem_data_size << DMA_SxCR_MSIZE_Pos) & DMA_SxCR_MSIZE;
    cr_mask |= config->flow_controller == DMA_FLOW_CONTROLLER_DMA ? 0 : DMA_SxCR_PFCTRL;
    if (config->circular_mode == DMA_MODE_DOUBLE_BUFFER) {
        cr_mask |= (DMA_SxCR_DBM | DMA_SxCR_CIRC);
    } else if (config->circular_mode == DMA_MODE_CIRCULAR) {
        cr_mask |= DMA_SxCR_CIRC;
    }

    stream->CR = cr_mask;
    stream->FCR |= config->mode == DMA_MODE_DIRECT ? 0 : DMA_SxFCR_DMDIS;
    stream->NDTR = config->buffer_size;

    if (config->per_addr) {
        stream->PAR = (uint32_t)config->per_addr;
    }
    if (config->mem_buf_0) {
        stream->M0AR = (uint32_t)config->mem_buf_0;
    }
    if (config->mem_buf_1 && config->circular_mode == DMA_MODE_DOUBLE_BUFFER) {
        stream->M1AR = (uint32_t)config->mem_buf_1;
    }

    NVIC_ClearPendingIRQ(stream_info.nvic_irq_type);
    NVIC_SetPriority(stream_info.nvic_irq_type, config->nvic_irq_priority);
    NVIC_EnableIRQ(stream_info.nvic_irq_type);

    if (config->enable_stream) {
        TRY(dma_enable_stream(stream));
    }

    return HAL_OK;
}

void dma_set_channel(DMA_Stream_TypeDef* stream, uint32_t channel) {
    if (stream) {
        stream->CR = (stream->CR & ~DMA_SxCR_CHSEL) | ((uint32_t)(channel << DMA_SxCR_CHSEL_Pos) & DMA_SxCR_CHSEL);
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

void dma_set_direction(DMA_Stream_TypeDef* stream, dma_direction_t dir) {
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
