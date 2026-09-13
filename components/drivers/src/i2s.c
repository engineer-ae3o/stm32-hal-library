#include "stm32f411xe.h"
#include "drivers/gpio.h"
#include "utils/common.h"
#include "utils/clock.h"
#include "drivers/spi.h"
#include "drivers/i2s.h"
#include "utils/err.h"


// Clock prescaler table
typedef struct {
    uint8_t i2sdiv;
    uint8_t odd;
} prescaler_t;

// Prescaler tables for the different supported audio PLL clock frequencies
// TODO: Compute the prescaler table
static const prescaler_t s_prescaler_lut[][I2S_FREQ_COUNT] = {
    // The Audio PLL at 76.8MHz. Suitable for the 48kHz family with MCK output disabled.
    // The other unsupported frequencies are left default initialized at 0.
    [AUDIO_PLL_76_8MHz] =
        {
            [I2S_FREQ_8kHz]   = {.i2sdiv = 0, .odd = 0},
            [I2S_FREQ_16kHz]  = {.i2sdiv = 0, .odd = 0},
            [I2S_FREQ_32kHz]  = {.i2sdiv = 0, .odd = 0},
            [I2S_FREQ_48kHz]  = {.i2sdiv = 0, .odd = 0},
            [I2S_FREQ_96kHz]  = {.i2sdiv = 0, .odd = 0},
            [I2S_FREQ_192kHz] = {.i2sdiv = 0, .odd = 0},
        },
    // The Audio PLL at 135.5MHz. Suitable for the 44.1kHz family with MCK output enabled.
    // The other unsupported frequencies are left default initialized at 0.
    [AUDIO_PLL_135_5MHz] =
        {
            [I2S_FREQ_22kHz] = {.i2sdiv = 0, .odd = 0},
            [I2S_FREQ_44kHz] = {.i2sdiv = 0, .odd = 0},
        },
    // The Audio PLL at 151MHz. Suitable for the 44.1kHz family with MCK output disabled.
    // The other unsupported frequencies are left default initialized at 0.
    [AUDIO_PLL_151MHz] =
        {
            [I2S_FREQ_22kHz] = {.i2sdiv = 0, .odd = 0},
            [I2S_FREQ_44kHz] = {.i2sdiv = 0, .odd = 0},
        },
    // The Audio PLL at 172MHz. Suitable for the 48kHz family with MCK output enabled.
    // The other unsupported frequencies are left default initialized at 0.
    [AUDIO_PLL_172MHz] =
        {
            [I2S_FREQ_8kHz]   = {.i2sdiv = 0, .odd = 0},
            [I2S_FREQ_16kHz]  = {.i2sdiv = 0, .odd = 0},
            [I2S_FREQ_32kHz]  = {.i2sdiv = 0, .odd = 0},
            [I2S_FREQ_48kHz]  = {.i2sdiv = 0, .odd = 0},
            [I2S_FREQ_96kHz]  = {.i2sdiv = 0, .odd = 0},
            [I2S_FREQ_192kHz] = {.i2sdiv = 0, .odd = 0},
        },
};


// Helper
[[__gnu__::__always_inline__]] static inline uint8_t get_index(const I2S_TypeDef* handle) {
    if (handle == I2S1) {
        return 0U;
    } else if (handle == I2S2) {
        return 1U;
    } else if (handle == I2S3) {
        return 2U;
    } else if (handle == I2S4) {
        return 3U;
    } else if (handle == I2S5) {
        return 4U;
    } else {
        return 0xFFU;
    }
}

#define ENABLE_I2S()                                                                                                                                 \
    do {                                                                                                                                             \
        handle->I2SCFGR |= SPI_I2SCFGR_I2SE;                                                                                                         \
        __DSB();                                                                                                                                     \
    } while (0)

#define DISABLE_I2S()                                                                                                                                \
    do {                                                                                                                                             \
        handle->I2SCFGR &= ~SPI_I2SCFGR_I2SE;                                                                                                        \
        __DSB();                                                                                                                                     \
    } while (0)

// Defined in the SPI driver. Used to post DMA events or get info from the SPI driver since the I2S peripheral
// shares the same hardware block as the SPI peripheral, and consequently, share the same DMA streams.
// All the NVIC interrupt handlers are managed by the SPI driver.
extern hal_err_t spi_master_get_dma_stream_map(dma_stream_map_t* map, uint32_t idx);
extern hal_err_t spi_master_register_callback(dma_done_cb_t callback, void* arg, uint8_t idx, bool is_tx);


// General API
hal_err_t i2sx_clk_enable(I2S_TypeDef* handle, bool enable) {
    if (enable) {
        if (handle == I2S1) {
            RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
        } else if (handle == I2S2) {
            RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
        } else if (handle == I2S3) {
            RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;
        } else if (handle == I2S4) {
            RCC->APB2ENR |= RCC_APB2ENR_SPI4EN;
        } else if (handle == I2S5) {
            RCC->APB2ENR |= RCC_APB2ENR_SPI5EN;
        } else {
            return HAL_ERR_INVALID_ARG;
        }

    } else {
        if (handle == I2S1) {
            RCC->APB2ENR &= ~RCC_APB2ENR_SPI1EN;
        } else if (handle == I2S2) {
            RCC->APB1ENR &= ~RCC_APB1ENR_SPI2EN;
        } else if (handle == I2S3) {
            RCC->APB1ENR &= ~RCC_APB1ENR_SPI3EN;
        } else if (handle == I2S4) {
            RCC->APB2ENR &= ~RCC_APB2ENR_SPI4EN;
        } else if (handle == I2S5) {
            RCC->APB2ENR &= ~RCC_APB2ENR_SPI5EN;
        } else {
            return HAL_ERR_INVALID_ARG;
        }
    }

    __DSB();
    return HAL_OK;
}

hal_err_t i2s_master_init(I2S_TypeDef* handle, const i2s_master_config_t* config) {
    if (handle == NULL || config == NULL || config->frequency == I2S_FREQ_COUNT) {
        return HAL_ERR_INVALID_ARG;
    }

    if (config->audio_pll_type == AUDIO_PLL_DISABLE) {
        return HAL_ERR_INVALID_STATE;
    }

    // Get the audio PLL clock prescaler
    const uint8_t i2sdiv = s_prescaler_lut[config->audio_pll_type][config->frequency].i2sdiv;
    const uint8_t odd    = s_prescaler_lut[config->audio_pll_type][config->frequency].odd;

    // A sampling rate of 192kHz is not supported when MCK output is not needed.
    // For more details, refer to clock.h as to why this setup is impractical.
    if ((config->frequency == I2S_FREQ_192kHz && config->use_mck) || i2sdiv == 0) {
        return HAL_ERR_NOT_SUPPORTED;
    }

    // Disable the SPI and I2S peripheral before modifying its registers
    handle->CR1 &= ~SPI_CR1_SPE;
    DISABLE_I2S();

    // I2S mode
    handle->I2SCFGR |= SPI_I2SCFGR_I2SMOD;

    // Apply the prescalers
    handle->I2SPR &= ~(SPI_I2SPR_I2SDIV | SPI_I2SPR_ODD | SPI_I2SPR_MCKOE);
    handle->I2SPR |= (uint32_t)(i2sdiv << SPI_I2SPR_I2SDIV_Pos) | (uint32_t)(odd << SPI_I2SPR_ODD_Pos) | ((config->use_mck) ? SPI_I2SPR_MCKOE : 0);

    // Get frame size: It can only be 16 bits when the data is 16 bits
    const uint32_t frame_size_mask = (config->frame == I2S_DATA_16_BITS_FRAME_16_BITS) ? 0 : SPI_I2SCFGR_CHLEN;
    const uint32_t cpol_mask       = (config->cpol) ? SPI_I2SCFGR_CKPOL : 0;

    handle->I2SCFGR &= ~(SPI_I2SCFGR_I2SCFG | SPI_I2SCFGR_CKPOL | SPI_I2SCFGR_CHLEN | SPI_I2SCFGR_I2SSTD | SPI_I2SCFGR_DATLEN);

    // Apply user settings
    handle->I2SCFGR |= (((uint32_t)config->direction << SPI_I2SCFGR_I2SCFG_Pos) | // Direction: TX or RX in master mode
                        ((uint32_t)config->mode << SPI_I2SCFGR_I2SSTD_Pos) |      // I2S mode: Philips, left or right justified
                        ((uint32_t)config->frame << SPI_I2SCFGR_DATLEN_Pos) |     // Data length: 16, 24 or 32 bits
                        frame_size_mask |                                         // Frame size: 16 or 32 bits
                        cpol_mask);                                               // Clock polarity

    // Configure the GPIO pins
    TRY(gpiox_clk_enable(config->gpio_port, true));

    // Alternate function value selection for the GPIO pins
    uint8_t alt_val = 0;
    if ((handle == I2S1) || (handle == I2S2)) {
        alt_val = 5;
    } else if (handle == I2S3) {
        alt_val = (config->gpio_port == GPIOD) ? 5 : 6;
    } else if (handle == I2S4) {
        alt_val = (config->gpio_port == GPIOE) ? 5 : 6;
    } else if (handle == I2S5) {
        alt_val = 6;
    } else {
        return HAL_ERR_INVALID_ARG;
    }

    // MCK pin if used
    if (config->use_mck) {
        TRY(gpio_set_alternate_function(config->gpio_port, config->mck_pin, alt_val));
        gpio_enable_pullup(config->gpio_port, config->mck_pin, true);
        gpio_set_speed_mode(config->gpio_port, config->mck_pin, GPIO_MEDIUM_SPEED);
        gpio_set_output_type(config->gpio_port, config->mck_pin, GPIO_PUSH_PULL);
    }

    // SD pin: Can be input or output
    TRY(gpio_set_alternate_function(config->gpio_port, config->sd_pin, alt_val));
    gpio_enable_pullup(config->gpio_port, config->sd_pin, true);
    gpio_set_speed_mode(config->gpio_port, config->sd_pin, GPIO_MEDIUM_SPEED);
    // Only set output type as push pull when we are driving, that is, in TX mode
    if (config->direction == I2S_DIR_HALF_DUPLEX_TX) {
        gpio_set_output_type(config->gpio_port, config->sd_pin, GPIO_PUSH_PULL);
    }

    // WS pin
    TRY(gpio_set_alternate_function(config->gpio_port, config->ws_pin, alt_val));
    gpio_enable_pullup(config->gpio_port, config->ws_pin, true);
    gpio_set_speed_mode(config->gpio_port, config->ws_pin, GPIO_MEDIUM_SPEED);
    gpio_set_output_type(config->gpio_port, config->ws_pin, GPIO_PUSH_PULL);

    // SCK pin
    TRY(gpio_set_alternate_function(config->gpio_port, config->sck_pin, alt_val));
    gpio_enable_pullup(config->gpio_port, config->sck_pin, true);
    gpio_set_speed_mode(config->gpio_port, config->sck_pin, GPIO_MEDIUM_SPEED);
    gpio_set_output_type(config->gpio_port, config->sck_pin, GPIO_PUSH_PULL);

    return HAL_OK;
}

hal_err_t i2s_master_deinit(I2S_TypeDef* handle) {
    if (handle == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    DISABLE_I2S();

    handle->I2SCFGR &= ~(SPI_I2SCFGR_I2SCFG | SPI_I2SCFGR_CKPOL | SPI_I2SCFGR_CHLEN | SPI_I2SCFGR_I2SSTD | SPI_I2SCFGR_DATLEN | SPI_I2SCFGR_PCMSYNC);
    handle->I2SPR &= ~(SPI_I2SPR_I2SDIV | SPI_I2SPR_ODD | SPI_I2SPR_MCKOE);

    return HAL_OK;
}

hal_err_t i2s_master_dma_init(I2S_TypeDef* handle, dma_priority_t priority) {
    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU) {
        return HAL_ERR_INVALID_ARG;
    }

    // Get the DMA stream mapped to the corresponding I2S handle
    dma_stream_map_t dma_map;
    TRY(spi_master_get_dma_stream_map(&dma_map, idx));

    DMA_Stream_TypeDef* tx_stream = dma_map.tx.stream;
    DMA_Stream_TypeDef* rx_stream = dma_map.rx.stream;

    if (tx_stream == NULL || rx_stream == NULL) {
        return HAL_ERR_NOT_SUPPORTED;
    }

    // DMA TX stream configuration
    const dma_stream_config_t tx_stream_config = {
        .deconfigure   = false,
        .enable_stream = false,

        .per_addr_incement = false,
        .mem_addr_incement = true,

        .tc_irq_enable  = true,
        .ht_irq_enable  = false,
        .te_irq_enable  = true,
        .dme_irq_enable = true,
        .fe_irq_enable  = false,

        .mode            = DMA_MODE_DIRECT,
        .priority        = priority,
        .direction       = DMA_DIR_M_P,
        .per_data_size   = DMA_SIZE_HWORD,
        .mem_data_size   = DMA_SIZE_HWORD,
        .circular_mode   = DMA_MODE_NO_CIRCULAR,
        .flow_controller = DMA_FLOW_CONTROLLER_DMA,

        .buffer_size       = 0,
        .channel           = dma_map.tx.channel,
        .nvic_irq_priority = I2S_DMA_NVIC_IRQ_PRIORITY,

        .per_addr  = NULL,
        .mem_buf_0 = NULL,
        .mem_buf_1 = NULL,
    };

    // DMA RX stream configuration
    const dma_stream_config_t rx_stream_config = {
        .deconfigure   = false,
        .enable_stream = false,

        .per_addr_incement = false,
        .mem_addr_incement = true,

        .tc_irq_enable  = true,
        .ht_irq_enable  = false,
        .te_irq_enable  = true,
        .dme_irq_enable = true,
        .fe_irq_enable  = false,

        .mode            = DMA_MODE_DIRECT,
        .priority        = priority,
        .direction       = DMA_DIR_P_M,
        .per_data_size   = DMA_SIZE_HWORD,
        .mem_data_size   = DMA_SIZE_HWORD,
        .circular_mode   = DMA_MODE_NO_CIRCULAR,
        .flow_controller = DMA_FLOW_CONTROLLER_DMA,

        .buffer_size       = 0,
        .channel           = dma_map.rx.channel,
        .nvic_irq_priority = I2S_DMA_NVIC_IRQ_PRIORITY,

        .per_addr  = NULL,
        .mem_buf_0 = NULL,
        .mem_buf_1 = NULL,
    };

    TRY(dma_configure_stream(tx_stream, &tx_stream_config));
    TRY(dma_configure_stream(rx_stream, &rx_stream_config));

    // Enable I2S requests to the DMA controller
    handle->CR2 |= (SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);

    return HAL_OK;
}

hal_err_t i2s_master_dma_deinit(I2S_TypeDef* handle) {
    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU) {
        return HAL_ERR_INVALID_ARG;
    }

    // Get the DMA stream mapped to the corresponding I2S handle
    dma_stream_map_t dma_map;
    TRY(spi_master_get_dma_stream_map(&dma_map, idx));

    DMA_Stream_TypeDef* tx_stream = dma_map.tx.stream;
    DMA_Stream_TypeDef* rx_stream = dma_map.rx.stream;

    if (tx_stream == NULL || rx_stream == NULL) {
        return HAL_ERR_NOT_SUPPORTED;
    }

    // Disable I2S requests to the DMA controller
    handle->CR2 &= ~(SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);

    dma_stream_config_t tx_stream_config = {};
    tx_stream_config.deconfigure         = true;

    dma_stream_config_t rx_stream_config = {};
    rx_stream_config.deconfigure         = true;

    TRY(dma_configure_stream(tx_stream, &tx_stream_config));
    TRY(dma_configure_stream(rx_stream, &rx_stream_config));

    return HAL_OK;
}


// DMA transfers API
hal_err_t i2s_master_transmit(I2S_TypeDef* handle, const void* buf, uint16_t size, dma_done_cb_t callback, void* arg) {
    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU || buf == NULL || size == 0) {
        return HAL_ERR_INVALID_ARG;
    }

    // Get the DMA stream mapped to the corresponding I2S handle
    dma_stream_map_t dma_map;
    TRY(spi_master_get_dma_stream_map(&dma_map, idx));

    DMA_Stream_TypeDef* stream = dma_map.tx.stream;
    if (stream == NULL) {
        return HAL_ERR_NOT_SUPPORTED;
    }

    // If the data is 24 or 32 bits, we need two DMA transfers
    // Note that if the DATLEN bits are 0b00, that means 16 bit
    // data; 0b01 for 24 and 0b10 for 32. So a non zero value
    // from the DATLEN bits implies a transfer greater than 16 bits
    size = (handle->I2SCFGR & SPI_I2SCFGR_DATLEN) ? (size * 2) : size;

    // Set the memory addresses and length
    dma_set_addresses(stream, &handle->DR, buf, NULL);
    dma_set_trans_length(stream, size);

    // Save the user passed callback
    if (callback) {
        TRY(spi_master_register_callback(callback, arg, idx, true));
    }

    // Enable the DMA TX stream
    TRY(dma_enable_stream(stream));
    ENABLE_I2S();

    return HAL_OK;
}

hal_err_t i2s_master_receive(I2S_TypeDef* handle, void* buf, uint16_t size, dma_done_cb_t callback, void* arg) {
    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU || buf == NULL || size == 0) {
        return HAL_ERR_INVALID_ARG;
    }

    // Get the DMA stream mapped to the corresponding I2S handle
    dma_stream_map_t dma_map;
    TRY(spi_master_get_dma_stream_map(&dma_map, idx));

    DMA_Stream_TypeDef* stream = dma_map.rx.stream;
    if (stream == NULL) {
        return HAL_ERR_NOT_SUPPORTED;
    }

    // Reason for this is above in i2s_master_transmit(...)
    size = (handle->I2SCFGR & SPI_I2SCFGR_DATLEN) ? (size * 2) : size;

    // Set the memory addresses and length
    dma_set_addresses(stream, &handle->DR, buf, NULL);
    dma_set_trans_length(stream, size);

    // Save the user passed callback
    if (callback) {
        TRY(spi_master_register_callback(callback, arg, idx, false));
    }

    // Enable the DMA RX stream
    TRY(dma_enable_stream(stream));
    ENABLE_I2S();

    return HAL_OK;
}


// Double buffering API
hal_err_t i2s_master_dbm_init(I2S_TypeDef* handle, void* buf_0, void* buf_1, uint16_t size, dma_done_cb_t callback, void* arg) {
    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU || buf_0 == NULL || buf_1 == NULL || size == 0) {
        return HAL_ERR_INVALID_ARG;
    }

    // Get the DMA stream mapped to the corresponding I2S handle
    dma_stream_map_t dma_map;
    TRY(spi_master_get_dma_stream_map(&dma_map, idx));

    DMA_Stream_TypeDef* stream = dma_map.rx.stream;
    if (stream == NULL) {
        return HAL_ERR_NOT_SUPPORTED;
    }

    // Get the transfer size from the DATLEN bit
    size = (handle->I2SCFGR & SPI_I2SCFGR_DATLEN) ? (size * 2) : size;

    TRY(dma_disable_stream(stream));
    dma_enable_circm_dbm(stream, true, true);
    dma_set_addresses(stream, &handle->DR, buf_0, buf_1);
    dma_set_trans_length(stream, size);

    // Save the user passed callback
    if (callback) {
        TRY(spi_master_register_callback(callback, arg, idx, false));
    }

    return HAL_OK;
}

hal_err_t i2s_master_dbm_deinit(I2S_TypeDef* handle) {
    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU) {
        return HAL_ERR_INVALID_ARG;
    }

    // Get the DMA stream mapped to the corresponding I2S handle
    dma_stream_map_t dma_map;
    TRY(spi_master_get_dma_stream_map(&dma_map, idx));

    DMA_Stream_TypeDef* stream = dma_map.rx.stream;
    if (stream == NULL) {
        return HAL_ERR_NOT_SUPPORTED;
    }

    TRY(dma_disable_stream(stream));
    DISABLE_I2S();

    return HAL_OK;
}

hal_err_t i2s_master_dbm_start(I2S_TypeDef* handle) {
    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU) {
        return HAL_ERR_INVALID_ARG;
    }

    dma_stream_map_t dma_map;
    TRY(spi_master_get_dma_stream_map(&dma_map, idx));

    DMA_Stream_TypeDef* stream = dma_map.rx.stream;
    if (stream == NULL) {
        return HAL_ERR_NOT_SUPPORTED;
    }

    TRY(dma_enable_stream(stream));
    ENABLE_I2S();

    return HAL_OK;
}

hal_err_t i2s_master_dbm_stop(I2S_TypeDef* handle) {
    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU) {
        return HAL_ERR_INVALID_ARG;
    }

    dma_stream_map_t dma_map;
    TRY(spi_master_get_dma_stream_map(&dma_map, idx));

    DMA_Stream_TypeDef* stream = dma_map.rx.stream;
    if (stream == NULL) {
        return HAL_ERR_NOT_SUPPORTED;
    }

    TRY(dma_disable_stream(stream));
    DISABLE_I2S();

    return HAL_OK;
}

hal_err_t i2s_master_dbm_get_filled_buffer(I2S_TypeDef* handle, uint8_t* buffer_idx) {
    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU) {
        return HAL_ERR_INVALID_ARG;
    }

    // Get the DMA stream mapped to the corresponding I2S handle
    dma_stream_map_t dma_map;
    if (spi_master_get_dma_stream_map(&dma_map, idx) != HAL_OK) {
        return HAL_ERR_INVALID_ARG;
    }

    DMA_Stream_TypeDef* stream = dma_map.rx.stream;
    if (stream == NULL) {
        return HAL_ERR_NOT_SUPPORTED;
    }

    // CT represents the active buffer. That is, the buffer currently in use by the dma controller.
    // 0 represents the first buffer, and 1 is the second buffer. If the bit is 0, that means the first
    // buffer is currently being used by the DMA controller and the second buffer is filled and free.
    *buffer_idx = (stream->CR & DMA_SxCR_CT) ? 0 : 1;

    return HAL_OK;
}
