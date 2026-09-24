#include "stm32f411xe.h"
#include "drivers/spi_internals.h"
#include "drivers/gpio.h"
#include "utils/common.h"
#include "utils/clock.h"
#include "drivers/dma.h"
#include "drivers/i2s.h"
#include "utils/err.h"

#include <stdint.h>


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
    if (handle == NULL || config == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    uint32_t audio_clock = 0;
    switch (config->audio_clock) {
        case AUDIO_PLL_76_8MHz:
            audio_clock = 76'800'000U;
            break;
        case AUDIO_PLL_135_5MHz:
            audio_clock = 135'500'000U;
            break;
        case AUDIO_PLL_151MHz:
            audio_clock = 151'000'000U;
            break;
        case AUDIO_PLL_172MHz:
            audio_clock = 172'000'000U;
            break;
        default:
            return HAL_ERR_INVALID_ARG;
    }

    // Get the audio PLL clock prescalers
    uint32_t multiplier = 0;
    if (config->use_mck) {
        multiplier = 256;
    } else {
        multiplier = (config->frame == I2S_DATA_16_BITS_FRAME_16_BITS) ? 32 : 64;
    }

    const uint32_t denominator = config->frequency * multiplier;
    const uint32_t divisor     = (audio_clock + (denominator / 2)) / denominator;

    const uint32_t i2sdiv = divisor >> 1;
    const uint32_t odd    = divisor & 1;
    if (i2sdiv < 2 || i2sdiv > UINT8_MAX) {
        return HAL_ERR_NOT_SUPPORTED;
    }

    // Disable the I2S (and SPI) peripheral before modifying its registers
    DISABLE_SPI();
    DISABLE_I2S();

    // I2S mode
    handle->I2SCFGR |= SPI_I2SCFGR_I2SMOD;

    // Apply the prescalers
    handle->I2SPR &= ~(SPI_I2SPR_I2SDIV | SPI_I2SPR_ODD | SPI_I2SPR_MCKOE);
    handle->I2SPR |= (uint32_t)(i2sdiv << SPI_I2SPR_I2SDIV_Pos) | (uint32_t)(odd << SPI_I2SPR_ODD_Pos) | ((config->use_mck) ? SPI_I2SPR_MCKOE : 0);

    handle->I2SCFGR &= ~(SPI_I2SCFGR_I2SCFG | SPI_I2SCFGR_CKPOL | SPI_I2SCFGR_CHLEN | SPI_I2SCFGR_I2SSTD | SPI_I2SCFGR_DATLEN | SPI_I2SCFGR_PCMSYNC);
    handle->I2SCFGR |= (((uint32_t)config->direction << SPI_I2SCFGR_I2SCFG_Pos) | // Direction: TX or RX in master mode
                        ((uint32_t)config->mode << SPI_I2SCFGR_I2SSTD_Pos) |      // I2S mode: Philips, left or right justified
                        (uint32_t)config->frame |                                 // Already encodes data frame size and channel length
                        ((uint32_t)config->cpol ? SPI_I2SCFGR_CKPOL : 0));        // Clock polarity

    // Disable SPI DMA requests by default
    DISABLE_SPI_DMA();

    // Configure the GPIO pins
    if (config->use_mck) {
        TRY(gpiox_clk_enable(config->mclk_pin.port, true));
        gpio_set_alternate_function(config->mclk_pin.port, config->mclk_pin.pin, config->mclk_pin.af);
        gpio_enable_pullups(config->mclk_pin.port, config->mclk_pin.pin, true);
        gpio_set_speed_mode(config->mclk_pin.port, config->mclk_pin.pin, GPIO_FULL_SPEED);
        gpio_set_output_type(config->mclk_pin.port, config->mclk_pin.pin, GPIO_PUSH_PULL);
    }

    TRY(gpiox_clk_enable(config->sclk_pin.port, true));
    gpio_set_alternate_function(config->sclk_pin.port, config->sclk_pin.pin, config->sclk_pin.af);
    gpio_enable_pullups(config->sclk_pin.port, config->sclk_pin.pin, true);
    gpio_set_speed_mode(config->sclk_pin.port, config->sclk_pin.pin, GPIO_FULL_SPEED);
    gpio_set_output_type(config->sclk_pin.port, config->sclk_pin.pin, GPIO_PUSH_PULL);

    TRY(gpiox_clk_enable(config->ws_pin.port, true));
    gpio_set_alternate_function(config->ws_pin.port, config->ws_pin.pin, config->ws_pin.af);
    gpio_enable_pullups(config->ws_pin.port, config->ws_pin.pin, true);
    gpio_set_speed_mode(config->ws_pin.port, config->ws_pin.pin, GPIO_FULL_SPEED);
    gpio_set_output_type(config->ws_pin.port, config->ws_pin.pin, GPIO_PUSH_PULL);

    TRY(gpiox_clk_enable(config->sd_pin.port, true));
    gpio_set_alternate_function(config->sd_pin.port, config->sd_pin.pin, config->sd_pin.af);
    gpio_enable_pullups(config->sd_pin.port, config->sd_pin.pin, true);
    gpio_set_speed_mode(config->sd_pin.port, config->sd_pin.pin, GPIO_FULL_SPEED);
    gpio_set_output_type(config->sd_pin.port, config->sd_pin.pin, GPIO_PUSH_PULL);

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

    // By default, i2s_master_dma_init(...) only supports oneshot transfers.
    // To get double buffering, call i2s_master_dbm_init(...) after calling
    // this function. Call i2s_master_dbm_deinit(...) when done with double
    // buffering and want to go back to oneshot transfers.

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

        .per_addr_increment = false,
        .mem_addr_increment = true,

        .tc_irq_enable  = true,
        .ht_irq_enable  = false,
        .te_irq_enable  = true,
        .dme_irq_enable = true,
        .fe_irq_enable  = false,

        .mode            = DMA_MODE_DIRECT,
        .priority        = priority,
        .direction       = DMA_DIR_M2P,
        .per_data_size   = DMA_SIZE_HWORD,
        .mem_data_size   = DMA_SIZE_HWORD,
        .circular_mode   = DMA_MODE_ONESHOT,
        .flow_controller = DMA_FLOW_CONTROLLER_DMA,

        .buffer_size       = 0,
        .channel           = dma_map.tx.channel,
        .nvic_irq_priority = I2S_DMA_NVIC_IRQ_PRIORITY,

        .per_addr  = &handle->DR,
        .mem_buf_0 = NULL,
        .mem_buf_1 = NULL,
    };

    // DMA RX stream configuration
    const dma_stream_config_t rx_stream_config = {
        .deconfigure   = false,
        .enable_stream = false,

        .per_addr_increment = false,
        .mem_addr_increment = true,

        .tc_irq_enable  = true,
        .ht_irq_enable  = false,
        .te_irq_enable  = true,
        .dme_irq_enable = true,
        .fe_irq_enable  = false,

        .mode            = DMA_MODE_DIRECT,
        .priority        = priority,
        .direction       = DMA_DIR_P2M,
        .per_data_size   = DMA_SIZE_HWORD,
        .mem_data_size   = DMA_SIZE_HWORD,
        .circular_mode   = DMA_MODE_ONESHOT,
        .flow_controller = DMA_FLOW_CONTROLLER_DMA,

        .buffer_size       = 0,
        .channel           = dma_map.rx.channel,
        .nvic_irq_priority = I2S_DMA_NVIC_IRQ_PRIORITY,

        .per_addr  = &handle->DR,
        .mem_buf_0 = NULL,
        .mem_buf_1 = NULL,
    };

    TRY(dma_configure_stream(tx_stream, &tx_stream_config));
    TRY(dma_configure_stream(rx_stream, &rx_stream_config));

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

    dma_stream_config_t tx_stream_config;
    tx_stream_config.deconfigure = true;

    dma_stream_config_t rx_stream_config;
    rx_stream_config.deconfigure = true;

    TRY(dma_configure_stream(tx_stream, &tx_stream_config));
    TRY(dma_configure_stream(rx_stream, &rx_stream_config));

    return HAL_OK;
}


// DMA oneshot transfers API
hal_err_t i2s_master_transmit_oneshot(I2S_TypeDef* handle, const void* data, uint16_t size, dma_done_cb_t callback, void* arg) {
    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU || data == NULL || size == 0) {
        return HAL_ERR_INVALID_ARG;
    }

    // Get the DMA stream mapped to the corresponding I2S handle
    dma_stream_map_t dma_map;
    TRY(spi_master_get_dma_stream_map(&dma_map, idx));

    DMA_Stream_TypeDef* stream = dma_map.tx.stream;
    if (stream == NULL) {
        return HAL_ERR_NOT_SUPPORTED;
    }

    // If for some reason, the CIRC and/or DBM bits are set, this means that i2s_master_dbm_init(...)
    // was called without a call to i2s_master_dbm_deinit(...) which would clean up its state. Also
    // check to see if the DMA is currently enabled and ongoing in a transaction or SPI mode is selected.
    if (stream->CR & (DMA_SxCR_CIRC | DMA_SxCR_DBM | DMA_SxCR_EN) || !(handle->I2SCFGR & SPI_I2SCFGR_I2SMOD)) {
        return HAL_ERR_INVALID_STATE;
    }

    // If the data is 24 or 32 bits, we need two DMA transfers
    // Note that if the DATLEN bits are 0b00, that means 16 bit
    // data; 0b01 for 24 and 0b10 for 32. So a non zero value
    // from the DATLEN bits implies a transfer greater than 16 bits
    const uint32_t actual_size = (handle->I2SCFGR & SPI_I2SCFGR_DATLEN) ? (size * 2) : size;
    if (actual_size > UINT16_MAX) {
        return HAL_ERR_INVALID_SIZE;
    }

    // Set the memory addresses and length
    dma_set_addresses(stream, &handle->DR, data, NULL);
    dma_set_trans_length(stream, (uint16_t)actual_size);

    TRY(spi_master_register_callback(callback, arg, idx, true));

    // Enable the DMA stream, SPI requests to the DMA controller,
    // and finally the I2S peripheral. The order is important.
    TRY(dma_enable_stream(stream));
    ENABLE_SPI_TX_DMA();
    ENABLE_I2S();

    return HAL_OK;
}

hal_err_t i2s_master_receive_oneshot(I2S_TypeDef* handle, void* data, uint16_t size, dma_done_cb_t callback, void* arg) {
    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU || data == NULL || size == 0) {
        return HAL_ERR_INVALID_ARG;
    }

    // Get the DMA stream mapped to the corresponding I2S handle
    dma_stream_map_t dma_map;
    TRY(spi_master_get_dma_stream_map(&dma_map, idx));

    DMA_Stream_TypeDef* stream = dma_map.rx.stream;
    if (stream == NULL) {
        return HAL_ERR_NOT_SUPPORTED;
    }

    // If for some reason, the CIRC and/or DBM bits are set, this means that i2s_master_dbm_init(...)
    // was called without a call to i2s_master_dbm_deinit(...) which would clean up its state. Also
    // check to see if the DMA is currently enabled and ongoing in a transaction or SPI mode is selected.
    if (stream->CR & (DMA_SxCR_CIRC | DMA_SxCR_DBM | DMA_SxCR_EN) || !(handle->I2SCFGR & SPI_I2SCFGR_I2SMOD)) {
        return HAL_ERR_INVALID_STATE;
    }

    // Reason for this is above in i2s_master_transmit(...)
    const uint32_t actual_size = (handle->I2SCFGR & SPI_I2SCFGR_DATLEN) ? (size * 2) : size;
    if (actual_size > UINT16_MAX) {
        return HAL_ERR_INVALID_SIZE;
    }

    // Set the memory addresses and length
    dma_set_addresses(stream, &handle->DR, data, NULL);
    dma_set_trans_length(stream, (uint16_t)actual_size);

    TRY(spi_master_register_callback(callback, arg, idx, false));

    // Enable the DMA stream, SPI requests to the DMA controller,
    // and finally the I2S peripheral. The order is important.
    TRY(dma_enable_stream(stream));
    ENABLE_SPI_RX_DMA();
    ENABLE_I2S();

    return HAL_OK;
}


// Double buffering API
hal_err_t i2s_master_dbm_init(I2S_TypeDef* handle, void* buf_0, void* buf_1, uint16_t size, dma_done_cb_t callback, void* arg) {
    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU || buf_0 == NULL || buf_1 == NULL || size == 0 || callback == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    // Get the DMA stream mapped to the corresponding I2S handle
    dma_stream_map_t dma_map;
    TRY(spi_master_get_dma_stream_map(&dma_map, idx));

    DMA_Stream_TypeDef* stream = dma_map.rx.stream;
    if (stream == NULL) {
        return HAL_ERR_NOT_SUPPORTED;
    }

    if (stream->CR & (DMA_SxCR_CIRC | DMA_SxCR_DBM | DMA_SxCR_EN) || !(handle->I2SCFGR & SPI_I2SCFGR_I2SMOD)) {
        return HAL_ERR_INVALID_STATE;
    }

    // Get the transfer size from the DATLEN bit
    const uint32_t actual_size = (handle->I2SCFGR & SPI_I2SCFGR_DATLEN) ? (size * 2) : size;
    if (actual_size > UINT16_MAX) {
        return HAL_ERR_INVALID_SIZE;
    }

    TRY(dma_disable_stream(stream));
    dma_set_circular_mode(stream, DMA_MODE_DOUBLE_BUFFERS);
    dma_set_addresses(stream, &handle->DR, buf_0, buf_1);
    dma_set_trans_length(stream, (uint16_t)actual_size);

    TRY(spi_master_register_callback(callback, arg, idx, false));

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

    // Stop the transaction and disable the stream before deinitiaizing the stream
    DISABLE_I2S();
    DISABLE_SPI_RX_DMA();
    TRY(dma_disable_stream(stream));

    // Reset the mode to oneshot and clear the interrupt handler
    dma_set_circular_mode(stream, DMA_MODE_ONESHOT);
    TRY(spi_master_register_callback(NULL, NULL, idx, false));

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
    ENABLE_SPI_RX_DMA();
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

    DISABLE_I2S();
    DISABLE_SPI_RX_DMA();
    TRY(dma_disable_stream(stream));

    return HAL_OK;
}

hal_err_t i2s_master_dbm_get_filled_buffer(I2S_TypeDef* handle, uint8_t* buffer_idx) {
    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU || buffer_idx == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    // Get the DMA stream mapped to the corresponding I2S handle
    dma_stream_map_t dma_map;
    TRY(spi_master_get_dma_stream_map(&dma_map, idx));

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
