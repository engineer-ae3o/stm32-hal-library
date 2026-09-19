#include "stm32f411xe.h"
#include "drivers/spi_internals.h"
#include "drivers/gpio.h"
#include "utils/common.h"
#include "drivers/spi.h"
#include "drivers/dma.h"
#include "utils/err.h"

#include <string.h>
#include <stddef.h>


// Mapping for the DMA streams to the SPI peripheral instances
static const dma_stream_map_t s_spi_i2s_dma_map[] = {
    // SPI1: DMA not supported: Not enough streams to go round other peripherals
    {
        .tx = {.stream = NULL, .channel = 0},
        .rx = {.stream = NULL, .channel = 0},
    },
    // SPI2
    {
        .tx = {.stream = DMA1_Stream4, .channel = 0},
        .rx = {.stream = DMA1_Stream3, .channel = 0},
    },
    // SPI3
    {
        .tx = {.stream = DMA1_Stream7, .channel = 0},
        .rx = {.stream = DMA1_Stream2, .channel = 0},
    },
    // SPI4
    {
        .tx = {.stream = DMA2_Stream1, .channel = 4},
        .rx = {.stream = DMA2_Stream4, .channel = 4},
    },
    // SPI5: DMA not supported: Not enough streams to go round other peripherals
    {
        .tx = {.stream = NULL, .channel = 0},
        .rx = {.stream = NULL, .channel = 0},
    },
};

// The SPI instances: The ISRs invoked when a DMA event occurred
static dma_stream_ctx_t s_dma_stream_ctx[ARRAY_SIZE(s_spi_i2s_dma_map)] = {};

// Helpers
[[__gnu__::__always_inline__]] static inline uint8_t get_index(const SPI_TypeDef* handle) {
    if (handle == SPI1) {
        return 0U;
    } else if (handle == SPI2) {
        return 1U;
    } else if (handle == SPI3) {
        return 2U;
    } else if (handle == SPI4) {
        return 3U;
    } else if (handle == SPI5) {
        return 4U;
    } else {
        return 0xFFU;
    }
}

[[__gnu__::__always_inline__]] static inline void isr_tx_helper(SPI_TypeDef* handle) {
    const uint8_t idx = get_index(handle);
    ASSERT(idx != 0xFFU);

    // Clear any flags that were set and get the error status
    hal_err_t ret = dma_isr_helper(s_spi_i2s_dma_map[idx].tx.stream);

    // Transfers require us to poll on the TXE and BSY flags even after data has
    // been shifted out. If they're not in the required state, an error occurred

    // Poll till TXE has been set
    // Skip polling if an error has occurred
    if (ret == HAL_OK) {
        uint32_t timeout = TIMEOUT;
        while (!(handle->SR & SPI_SR_TXE) && (--timeout));
        if (timeout == 0) {
            ret = HAL_ERR_SPI_TXE_FAILED_TO_SET;
        }
    }

    // Poll till BSY has been cleared
    // Skip polling BSY if TXE failed to set or another error occurred
    if (ret == HAL_OK) {
        uint32_t timeout = TIMEOUT;
        while ((handle->SR & SPI_SR_BSY) && (--timeout));
        if (timeout == 0) {
            ret = HAL_ERR_SPI_BSY_FAILED_TO_CLEAR;
        }
    }

    __disable_irq();

    // Save the user callback so we can clear it's global array position
    const dma_done_cb_t local_cb  = s_dma_stream_ctx[idx].tx.callback;
    void* const         local_arg = s_dma_stream_ctx[idx].tx.arg;

    // Only clear the user callback and disable the peripheral if not in circular mode
    if (!(s_spi_i2s_dma_map[idx].tx.stream->CR & DMA_SxCR_CIRC)) {
        s_dma_stream_ctx[idx].tx.callback = NULL;
        s_dma_stream_ctx[idx].tx.arg      = NULL;

        // Disable I2S as well since the interrupt could have been triggered by it
        DISABLE_SPI_DMA();
        DISABLE_SPI();
        DISABLE_I2S();
    }

    __enable_irq();

    if (local_cb) {
        // Finally, invoke the user callback
        local_cb(local_arg, ret);
    }
}

[[__gnu__::__always_inline__]] static inline void isr_rx_helper(SPI_TypeDef* handle) {
    const uint8_t idx = get_index(handle);
    ASSERT(idx != 0xFFU);

    // Clear any flags that were set and get the error status
    hal_err_t ret = dma_isr_helper(s_spi_i2s_dma_map[idx].rx.stream);

    __disable_irq();

    // Save the user callback so we can clear it's global array position
    const dma_done_cb_t local_cb  = s_dma_stream_ctx[idx].rx.callback;
    void* const         local_arg = s_dma_stream_ctx[idx].rx.arg;

    // Only clear the user callback and disable the peripheral if not in circular mode
    if (!(s_spi_i2s_dma_map[idx].rx.stream->CR & DMA_SxCR_CIRC)) {
        s_dma_stream_ctx[idx].rx.callback = NULL;
        s_dma_stream_ctx[idx].rx.arg      = NULL;

        // Disable I2S as well since the interrupt could have been triggered by it
        DISABLE_SPI_DMA();
        DISABLE_SPI();
        DISABLE_I2S();
    }

    __enable_irq();

    if (local_cb) {
        // Finally, invoke the user callback
        local_cb(local_arg, ret);
    }
}

[[__gnu__::__always_inline__]] static inline hal_err_t poll_transfer_helper(SPI_TypeDef* handle, const void* tx_data, void* rx_data, size_t size) {
    // The DFF bit being set means a 16 bit transfer
    if (handle->CR1 & SPI_CR1_DFF) {
        // Cast to appropriate type
        const uint16_t* tx_buf = (const uint16_t*)tx_data;
        uint16_t*       rx_buf = (uint16_t*)rx_data;

        for (size_t i = 0; i < size; i++) {
            // Write the data, or a dummy word if this is a receive-only transfer
            handle->DR = tx_buf ? tx_buf[i] : 0;

            // Poll till the data has been transferred out
            uint32_t timeout = TIMEOUT;
            while (!(handle->SR & SPI_SR_TXE) && (--timeout));
            if (timeout == 0) {
                return HAL_ERR_TIMEOUT;
            }

            // Poll till the data has been received
            timeout = TIMEOUT;
            while (!(handle->SR & SPI_SR_RXNE) && (--timeout));
            if (timeout == 0) {
                return HAL_ERR_TIMEOUT;
            }

            // Read the data register: keep it if this is a receive, discard otherwise
            const uint16_t rx_word = (uint16_t)handle->DR;
            if (rx_buf) {
                rx_buf[i] = rx_word;
            }
        }
    } else {
        // Cast to appropriate type
        const uint8_t* tx_buf = (const uint8_t*)tx_data;
        uint8_t*       rx_buf = (uint8_t*)rx_data;

        for (size_t i = 0; i < size; i++) {
            // Write the data, or a dummy byte if this is a receive-only transfer
            handle->DR = tx_buf ? tx_buf[i] : 0;

            // Poll till data has been transferred out
            uint32_t timeout = TIMEOUT;
            while (!(handle->SR & SPI_SR_TXE) && (--timeout));
            if (timeout == 0) {
                return HAL_ERR_TIMEOUT;
            }

            // Wait till data has been received
            timeout = TIMEOUT;
            while (!(handle->SR & SPI_SR_RXNE) && (--timeout));
            if (timeout == 0) {
                return HAL_ERR_TIMEOUT;
            }

            // Read the data register: keep it if this is a receive, discard otherwise
            const uint8_t rx_byte = (uint8_t)handle->DR;
            if (rx_buf) {
                rx_buf[i] = rx_byte;
            }
        }
    }

    // Wait for the TXE and BSY bits to set and clear respectively
    // TXE bit
    uint32_t timeout = TIMEOUT;
    while (!(handle->SR & SPI_SR_TXE) && (--timeout));
    if (timeout == 0) {
        return HAL_ERR_TIMEOUT;
    }

    // BSY bit
    timeout = TIMEOUT;
    while ((handle->SR & SPI_SR_BSY) && (--timeout));
    if (timeout == 0) {
        return HAL_ERR_TIMEOUT;
    }

    return HAL_OK;
}


// General API
hal_err_t spix_clk_enable(SPI_TypeDef* handle, bool enable) {
    if (enable) {
        if (handle == SPI1) {
            RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
        } else if (handle == SPI2) {
            RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
        } else if (handle == SPI3) {
            RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;
        } else if (handle == SPI4) {
            RCC->APB2ENR |= RCC_APB2ENR_SPI4EN;
        } else if (handle == SPI5) {
            RCC->APB2ENR |= RCC_APB2ENR_SPI5EN;
        } else {
            return HAL_ERR_INVALID_ARG;
        }

    } else {
        if (handle == SPI1) {
            RCC->APB2ENR &= ~RCC_APB2ENR_SPI1EN;
        } else if (handle == SPI2) {
            RCC->APB1ENR &= ~RCC_APB1ENR_SPI2EN;
        } else if (handle == SPI3) {
            RCC->APB1ENR &= ~RCC_APB1ENR_SPI3EN;
        } else if (handle == SPI4) {
            RCC->APB2ENR &= ~RCC_APB2ENR_SPI4EN;
        } else if (handle == SPI5) {
            RCC->APB2ENR &= ~RCC_APB2ENR_SPI5EN;
        } else {
            return HAL_ERR_INVALID_ARG;
        }
    }

    __DSB();
    return HAL_OK;
}

hal_err_t spi_master_init(SPI_TypeDef* handle, const spi_master_config_t* config) {
    if (handle == NULL || config == NULL || (!config->use_miso && !config->use_mosi)) {
        return HAL_ERR_INVALID_ARG;
    }

    // Disable the SPI (and I2S) peripheral before modifying it's internal state
    DISABLE_SPI();
    DISABLE_I2S();

    // Deselect I2S since in SPI mode
    handle->I2SCFGR &= ~SPI_I2SCFGR_I2SMOD;

    uint32_t cr1_mask = handle->CR1;
    cr1_mask &= ~(SPI_CR1_LSBFIRST | // MSB first
                  SPI_CR1_RXONLY |   // TX and RX
                  SPI_CR1_CRCEN |    // Hardware CRC disable
                  SPI_CR1_CRCNEXT |  // No CRC phase; data phase always
                  SPI_CR1_BIDIMODE | // MOSI and MISO used from the SPI peripheral's perspective
                  SPI_CR1_BIDIOE |   // Dual communication
                  // Clear remaining state
                  SPI_CR1_CPOL | SPI_CR1_CPHA | SPI_CR1_DFF | SPI_CR1_BR);

    cr1_mask |= (((uint32_t)config->prescaler << SPI_CR1_BR_Pos) | // Clock prescaler
                 SPI_CR1_SSM |                                     // Software slave select
                 SPI_CR1_SSI |                                     // Set internal slave to high
                 SPI_CR1_MSTR);                                    // SPI in master mode
    cr1_mask |= (config->cpol) ? SPI_CR1_CPOL : 0;
    cr1_mask |= (config->cpha) ? SPI_CR1_CPHA : 0;
    cr1_mask |= (config->data_size == SPI_DATA_16_BITS) ? SPI_CR1_DFF : 0;

    handle->CR1 = cr1_mask;

    // Set motorolla mode, slave select output disable and disable SPI DMA requests by default
    handle->CR2 &= ~(SPI_CR2_FRF | SPI_CR2_SSOE | SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);

    // Configure the GPIO pins
    TRY(gpiox_clk_enable(config->sclk_pin.port, true));
    TRY(gpio_set_alternate_function(config->sclk_pin.port, config->sclk_pin.pin, config->sclk_pin.af));
    gpio_enable_pullups(config->sclk_pin.port, config->sclk_pin.pin, true);
    gpio_set_speed_mode(config->sclk_pin.port, config->sclk_pin.pin, GPIO_FULL_SPEED);
    gpio_set_output_type(config->sclk_pin.port, config->sclk_pin.pin, GPIO_PUSH_PULL);

    if (config->use_mosi) {
        TRY(gpiox_clk_enable(config->mosi_pin.port, true));
        TRY(gpio_set_alternate_function(config->mosi_pin.port, config->mosi_pin.pin, config->mosi_pin.af));
        gpio_enable_pullups(config->mosi_pin.port, config->mosi_pin.pin, true);
        gpio_set_speed_mode(config->mosi_pin.port, config->mosi_pin.pin, GPIO_FULL_SPEED);
        gpio_set_output_type(config->mosi_pin.port, config->mosi_pin.pin, GPIO_PUSH_PULL);
    }

    if (config->use_miso) {
        TRY(gpiox_clk_enable(config->miso_pin.port, true));
        TRY(gpio_set_alternate_function(config->miso_pin.port, config->miso_pin.pin, config->miso_pin.af));
        gpio_enable_pullups(config->miso_pin.port, config->miso_pin.pin, true);
        gpio_set_speed_mode(config->miso_pin.port, config->miso_pin.pin, GPIO_FULL_SPEED);
        gpio_set_output_type(config->miso_pin.port, config->miso_pin.pin, GPIO_PUSH_PULL);
    }

    return HAL_OK;
}

hal_err_t spi_master_deinit(SPI_TypeDef* handle) {
    if (handle == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    DISABLE_SPI();

    handle->CR1 &= ~(SPI_CR1_CPHA | SPI_CR1_CPOL | SPI_CR1_MSTR | SPI_CR1_BR | SPI_CR1_LSBFIRST | SPI_CR1_SSI | SPI_CR1_SSM | SPI_CR1_RXONLY |
                     SPI_CR1_DFF | SPI_CR1_CRCNEXT | SPI_CR1_CRCEN | SPI_CR1_BIDIOE | SPI_CR1_BIDIMODE);
    handle->CR2 &= ~(SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN | SPI_CR2_FRF | SPI_CR2_ERRIE | SPI_CR2_RXNEIE | SPI_CR2_TXEIE);

    return HAL_OK;
}

hal_err_t spi_master_dma_init(SPI_TypeDef* handle, dma_priority_t priority) {
    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU) {
        return HAL_ERR_INVALID_ARG;
    }

    DMA_Stream_TypeDef* tx_stream = s_spi_i2s_dma_map[idx].tx.stream;
    DMA_Stream_TypeDef* rx_stream = s_spi_i2s_dma_map[idx].rx.stream;

    if (tx_stream == NULL || rx_stream == NULL) {
        return HAL_ERR_NOT_SUPPORTED;
    }

    // The DFF bit being set means a 16 bit transfer
    const dma_data_size_t dma_data_size = (handle->CR1 & SPI_CR1_DFF) ? DMA_SIZE_HWORD : DMA_SIZE_BYTE;

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
        .direction       = DMA_DIR_M2P,
        .per_data_size   = dma_data_size,
        .mem_data_size   = dma_data_size,
        .circular_mode   = DMA_MODE_ONESHOT,
        .flow_controller = DMA_FLOW_CONTROLLER_DMA,

        .buffer_size       = 0,
        .channel           = s_spi_i2s_dma_map[idx].tx.channel,
        .nvic_irq_priority = SPI_DMA_NVIC_IRQ_PRIORITY,

        .per_addr  = &handle->DR,
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
        .direction       = DMA_DIR_P2M,
        .per_data_size   = dma_data_size,
        .mem_data_size   = dma_data_size,
        .circular_mode   = DMA_MODE_ONESHOT,
        .flow_controller = DMA_FLOW_CONTROLLER_DMA,

        .buffer_size       = 0,
        .channel           = s_spi_i2s_dma_map[idx].rx.channel,
        .nvic_irq_priority = SPI_DMA_NVIC_IRQ_PRIORITY,

        .per_addr  = &handle->DR,
        .mem_buf_0 = NULL,
        .mem_buf_1 = NULL,
    };

    TRY(dma_configure_stream(tx_stream, &tx_stream_config));
    TRY(dma_configure_stream(rx_stream, &rx_stream_config));

    return HAL_OK;
}

hal_err_t spi_master_dma_deinit(SPI_TypeDef* handle) {
    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU) {
        return HAL_ERR_INVALID_ARG;
    }

    DMA_Stream_TypeDef* tx_stream = s_spi_i2s_dma_map[idx].tx.stream;
    DMA_Stream_TypeDef* rx_stream = s_spi_i2s_dma_map[idx].rx.stream;

    if (tx_stream == NULL || rx_stream == NULL) {
        return HAL_ERR_NOT_SUPPORTED;
    }

    // Set the deconfigure flags so dma_configure_stream(...) deinitializes the streams
    dma_stream_config_t tx_stream_config;
    tx_stream_config.deconfigure = true;

    dma_stream_config_t rx_stream_config;
    rx_stream_config.deconfigure = true;

    TRY(dma_configure_stream(tx_stream, &tx_stream_config));
    TRY(dma_configure_stream(rx_stream, &rx_stream_config));

    // Zero out all stored callbacks
    __disable_irq();
    memset(&s_dma_stream_ctx[idx], 0, sizeof(s_dma_stream_ctx[idx]));
    __enable_irq();

    return HAL_OK;
}


// Polling API
hal_err_t spi_master_transmit_poll(SPI_TypeDef* handle, const void* data, size_t size) {
    if (handle == NULL || data == NULL || size == 0) {
        return HAL_ERR_INVALID_ARG;
    }

    // Guard against all scenarios possible here: SPI being enabled or I2S mode selected
    if ((handle->CR1 & SPI_CR1_SPE) || (handle->I2SCFGR & SPI_I2SCFGR_I2SMOD)) {
        return HAL_ERR_INVALID_STATE;
    }

    ENABLE_SPI();
    const hal_err_t error = poll_transfer_helper(handle, data, NULL, size);
    DISABLE_SPI();

    return error;
}

hal_err_t spi_master_receive_poll(SPI_TypeDef* handle, void* data, size_t size) {
    if (handle == NULL || data == NULL || size == 0) {
        return HAL_ERR_INVALID_ARG;
    }

    // Guard against all scenarios possible here: SPI being enabled or I2S mode selected
    if ((handle->CR1 & SPI_CR1_SPE) || (handle->I2SCFGR & SPI_I2SCFGR_I2SMOD)) {
        return HAL_ERR_INVALID_STATE;
    }

    ENABLE_SPI();
    const hal_err_t error = poll_transfer_helper(handle, NULL, data, size);
    DISABLE_SPI();

    return error;
}

hal_err_t spi_master_transceive_poll(SPI_TypeDef* handle, const void* tx_data, void* rx_data, size_t size) {
    if (handle == NULL || tx_data == NULL || rx_data == NULL || size == 0) {
        return HAL_ERR_INVALID_ARG;
    }

    // Guard against all scenarios possible here: SPI being enabled or I2S mode selected
    if ((handle->CR1 & SPI_CR1_SPE) || (handle->I2SCFGR & SPI_I2SCFGR_I2SMOD)) {
        return HAL_ERR_INVALID_STATE;
    }

    ENABLE_SPI();
    const hal_err_t error = poll_transfer_helper(handle, tx_data, rx_data, size);
    DISABLE_SPI();

    return error;
}


// DMA transfers API
hal_err_t spi_master_transmit_dma(SPI_TypeDef* handle, const void* data, uint16_t size, dma_done_cb_t cb, void* arg) {
    // Get index for DMA stream mapping
    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU || data == NULL || size == 0) {
        return HAL_ERR_INVALID_ARG;
    }

    // TX and RX mapping. The RX stream is used purely to drain the bytes SPI
    // shifts in during transmission: with RXDMAEN left enabled by
    // spi_master_dma_init(), an unserviced SPI RX side would leave the RXNE bit
    // set after the first byte and cause an overrun error.
    DMA_Stream_TypeDef* tx_stream = s_spi_i2s_dma_map[idx].tx.stream;
    DMA_Stream_TypeDef* rx_stream = s_spi_i2s_dma_map[idx].rx.stream;

    if (tx_stream == NULL || rx_stream == NULL) {
        return HAL_ERR_NOT_SUPPORTED;
    }

    // Guard against all scenarios possible here: SPI being enabled, the DMA streams being enabled or I2S mode selected
    if ((handle->CR1 & SPI_CR1_SPE) || (handle->I2SCFGR & SPI_I2SCFGR_I2SMOD) || (tx_stream->CR & DMA_SxCR_EN) || (rx_stream->CR & DMA_SxCR_EN)) {
        return HAL_ERR_INVALID_STATE;
    }

    // Discard sink for whatever comes in on MISO. Never incremented, so
    // every incoming word overwrites the same throwaway location.
    static uint16_t s_dummy_rx;

    // Set the memory address and transaction length for the real TX data
    // Peripheral data side doesn't increment but memory side increments.
    dma_set_addresses(tx_stream, &handle->DR, data, NULL);
    dma_set_trans_length(tx_stream, size);
    dma_set_increment(tx_stream, false, true);

    // RX stream drains into the same throwaway word every beat:
    // Peripheral data side and memory side both do not get incremented.
    dma_set_addresses(rx_stream, &handle->DR, &s_dummy_rx, NULL);
    dma_set_trans_length(rx_stream, size);
    dma_set_increment(rx_stream, false, false);

    if (cb) {
        // Save the callback to the TX DMA irq only.
        // The isr_tx_helper(...) polls the BSY and TXE bit, which tells us
        // that the last bit of the data has been transmitted. Its only then
        // we can safely disable the SPI peripheral. isr_rx_helper(...) has
        // no such checks, so isr_tx_helper(...) is better for our usage here.
        __disable_irq();
        s_dma_stream_ctx[idx].tx.callback = cb;
        s_dma_stream_ctx[idx].tx.arg      = arg;
        __enable_irq();
    }

    TRY(dma_enable_stream(rx_stream));
    TRY_WITH_FUNC(dma_enable_stream(tx_stream), dma_disable_stream(rx_stream));

    ENABLE_SPI_DMA();
    ENABLE_SPI();

    return HAL_OK;
}

hal_err_t spi_master_receive_dma(SPI_TypeDef* handle, void* data, uint16_t size, dma_done_cb_t cb, void* arg) {
    // Get index for DMA stream mapping
    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU || data == NULL || size == 0) {
        return HAL_ERR_INVALID_ARG;
    }

    // TX and RX mapping. In master mode, SCLK is only generated while data is
    // shifted out of DR, so the TX stream is used to keep feeding a dummy
    // word in for the duration of the transfer.
    DMA_Stream_TypeDef* tx_stream = s_spi_i2s_dma_map[idx].tx.stream;
    DMA_Stream_TypeDef* rx_stream = s_spi_i2s_dma_map[idx].rx.stream;

    if (tx_stream == NULL || rx_stream == NULL) {
        return HAL_ERR_NOT_SUPPORTED;
    }

    // Guard against all scenarios possible here: SPI being enabled, the DMA streams being enabled or I2S mode selected
    if ((handle->CR1 & SPI_CR1_SPE) || (handle->I2SCFGR & SPI_I2SCFGR_I2SMOD) || (tx_stream->CR & DMA_SxCR_EN) || (rx_stream->CR & DMA_SxCR_EN)) {
        return HAL_ERR_INVALID_STATE;
    }

    // Dummy source fed to DR for the duration of the transfer, purely to generate
    // the clock. Never incremented, so the same zero word is read every beat.
    static const uint16_t s_dummy_tx = 0;

    // TX stream feeds the same dummy word every beat: peripheral and
    // memory side both fixed, nothing to increment
    dma_set_addresses(tx_stream, &handle->DR, &s_dummy_tx, NULL);
    dma_set_trans_length(tx_stream, size);
    dma_set_increment(tx_stream, false, false);

    // Set the memory address and transaction length for the real RX data
    dma_set_addresses(rx_stream, &handle->DR, data, NULL);
    dma_set_trans_length(rx_stream, size);
    dma_set_increment(rx_stream, false, true);

    if (cb) {
        // Save the callback to the TX DMA irq only.
        // The isr_tx_helper(...) polls the BSY and TXE bit, which tells us
        // that the last bit of the data has been transmitted. Its only then
        // we can safely disable the SPI peripheral. isr_rx_helper(...) has
        // no such checks, so isr_tx_helper(...) is better for our usage here.
        __disable_irq();
        s_dma_stream_ctx[idx].tx.callback = cb;
        s_dma_stream_ctx[idx].tx.arg      = arg;
        __enable_irq();
    }

    TRY(dma_enable_stream(rx_stream));
    TRY_WITH_FUNC(dma_enable_stream(tx_stream), dma_disable_stream(rx_stream));

    ENABLE_SPI_DMA();
    ENABLE_SPI();

    return HAL_OK;
}

hal_err_t spi_master_transceive_dma(SPI_TypeDef* handle, const void* tx_data, void* rx_data, uint16_t size, dma_done_cb_t cb, void* arg) {
    // Get index for DMA stream mapping
    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU || tx_data == NULL || rx_data == NULL || size == 0) {
        return HAL_ERR_INVALID_ARG;
    }

    // TX and RX DMA streams mapping
    DMA_Stream_TypeDef* tx_stream = s_spi_i2s_dma_map[idx].tx.stream;
    DMA_Stream_TypeDef* rx_stream = s_spi_i2s_dma_map[idx].rx.stream;

    if (tx_stream == NULL || rx_stream == NULL) {
        return HAL_ERR_NOT_SUPPORTED;
    }

    // Guard against all scenarios possible here: SPI being enabled, the DMA streams being enabled or I2S mode selected
    if ((handle->CR1 & SPI_CR1_SPE) || (handle->I2SCFGR & SPI_I2SCFGR_I2SMOD) || (tx_stream->CR & DMA_SxCR_EN) || (rx_stream->CR & DMA_SxCR_EN)) {
        return HAL_ERR_INVALID_STATE;
    }

    // Set the memory address and transaction length.
    // Reenable memory increment for both streams.
    dma_set_addresses(tx_stream, &handle->DR, tx_data, NULL);
    dma_set_trans_length(tx_stream, size);
    dma_set_increment(tx_stream, false, true);

    dma_set_addresses(rx_stream, &handle->DR, rx_data, NULL);
    dma_set_trans_length(rx_stream, size);
    dma_set_increment(rx_stream, false, true);

    if (cb) {
        // Save the callback to the TX DMA irq only.
        // The isr_tx_helper(...) polls the BSY and TXE bit, which tells us
        // that the last bit of the data has been transmitted. Its only then
        // we can safely disable the SPI peripheral. isr_rx_helper(...) has
        // no such checks, so isr_tx_helper(...) is better for our usage here.
        __disable_irq();
        s_dma_stream_ctx[idx].tx.callback = cb;
        s_dma_stream_ctx[idx].tx.arg      = arg;
        __enable_irq();
    }

    TRY(dma_enable_stream(rx_stream));
    TRY_WITH_FUNC(dma_enable_stream(tx_stream), dma_disable_stream(rx_stream));

    ENABLE_SPI_DMA();
    ENABLE_SPI();

    return HAL_OK;
}


// To be used only by i2s.c to register interrupt handlers and get its DMA stream(s)
hal_err_t spi_master_register_callback(dma_done_cb_t callback, void* arg, uint8_t idx, bool is_tx) {
    if (idx >= ARRAY_SIZE(s_spi_i2s_dma_map)) {
        return HAL_ERR_INVALID_ARG;
    }
    __disable_irq();
    if (is_tx) {
        s_dma_stream_ctx[idx].tx.callback = callback;
        s_dma_stream_ctx[idx].tx.arg      = arg;
    } else {
        s_dma_stream_ctx[idx].rx.callback = callback;
        s_dma_stream_ctx[idx].rx.arg      = arg;
    }
    __enable_irq();
    return HAL_OK;
}

hal_err_t spi_master_get_dma_stream_map(dma_stream_map_t* map, uint8_t idx) {
    if (map == NULL || idx >= ARRAY_SIZE(s_spi_i2s_dma_map)) {
        return HAL_ERR_INVALID_ARG;
    }
    *map = s_spi_i2s_dma_map[idx];
    return HAL_OK;
}


// DMA interrupts
// SPI2: TX
void DMA1_Stream4_IRQHandler(void) {
    isr_tx_helper(SPI2);
}

// SPI2: RX
void DMA1_Stream3_IRQHandler(void) {
    isr_rx_helper(SPI2);
}

// SPI3: TX
void DMA1_Stream7_IRQHandler(void) {
    isr_tx_helper(SPI3);
}

// SPI3: RX
void DMA1_Stream2_IRQHandler(void) {
    isr_rx_helper(SPI3);
}

// SPI4: TX
void DMA2_Stream1_IRQHandler(void) {
    isr_tx_helper(SPI4);
}

// SPI4: RX
void DMA2_Stream4_IRQHandler(void) {
    isr_rx_helper(SPI4);
}
