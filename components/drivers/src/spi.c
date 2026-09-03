#include "stm32f411xe.h"
#include "drivers/gpio.h"
#include "utils/common.h"
#include "drivers/spi.h"
#include "drivers/dma.h"


// The 5 SPI instances: ISRs called when the DMA is done
static dma_done_cb_t s_dma_tx_done_cbs[5] = {};
static dma_done_cb_t s_dma_rx_done_cbs[5] = {};

static void* s_tx_args[5] = {};
static void* s_rx_args[5] = {};


// Mapping for the DMA channels for the 5 SPI channels
static const dma_stream_map_t s_spi_dma_map[5] = {
    // SPI1: DMA not supported: Not enough streams to go round other peripherals
    {
        .tx = {.controller = NULL, .stream = NULL, .stream_no = 0, .irq_type = 0, .channel = 0},
        .rx = {.controller = NULL, .stream = NULL, .stream_no = 0, .irq_type = 0, .channel = 0},
    },
    // SPI2
    {
        .tx = {.controller = DMA1, .stream = DMA1_Stream4, .stream_no = 4, .irq_type = DMA1_Stream4_IRQn, .channel = 0},
        .rx = {.controller = DMA1, .stream = DMA1_Stream3, .stream_no = 3, .irq_type = DMA1_Stream3_IRQn, .channel = 0},
    },
    // SPI3
    {
        .tx = {.controller = DMA1, .stream = DMA1_Stream7, .stream_no = 7, .irq_type = DMA1_Stream7_IRQn, .channel = 0},
        .rx = {.controller = DMA1, .stream = DMA1_Stream2, .stream_no = 2, .irq_type = DMA1_Stream2_IRQn, .channel = 0},
    },
    // SPI4
    {
        .tx = {.controller = DMA2, .stream = DMA2_Stream1, .stream_no = 1, .irq_type = DMA2_Stream1_IRQn, .channel = 4},
        .rx = {.controller = DMA2, .stream = DMA2_Stream4, .stream_no = 4, .irq_type = DMA2_Stream4_IRQn, .channel = 4},
    },
    // SPI5: DMA not supported: Not enough streams to go round other peripherals
    {
        .tx = {.controller = NULL, .stream = NULL, .stream_no = 0, .irq_type = 0, .channel = 0},
        .rx = {.controller = NULL, .stream = NULL, .stream_no = 0, .irq_type = 0, .channel = 0},
    },
};

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

[[__gnu__::__always_inline__]] static inline void isr_tx_helper(SPI_TypeDef* handle, hal_err_t ret, uint8_t idx) {
    if (!s_dma_tx_done_cbs[idx]) {
        return;
    }

    // Transfers require us to poll on the TXE and BSY flags even after data has
    // been shifted out. If they're not in the required state, an error occurred

    // Poll till TXE has been set
    // Skip polling if an error has occurred
    if (ret == HAL_OK) {
        uint32_t timeout = TIMEOUT_CYCLES;
        while (!(handle->SR & SPI_SR_TXE) && (--timeout));
        if (timeout == 0) {
            ret = HAL_ERR_SPI_TXE_FAILED_TO_SET;
        }
    }

    // Poll till BSY has been cleared
    // Skip polling BSY if TXE failed to set
    if (ret == HAL_OK) {
        uint32_t timeout = TIMEOUT_CYCLES;
        while ((handle->SR & SPI_SR_BSY) && (--timeout));
        if (timeout == 0) {
            ret = HAL_ERR_SPI_BSY_FAILED_TO_CLEAR;
        }
    }

    // Save the user callback so we can clear it's global array position
    const dma_done_cb_t local_cb  = s_dma_tx_done_cbs[idx];
    void* const         local_arg = s_tx_args[idx];

    // Only clear the user callback if not in circular mode
    if (!(s_spi_dma_map[idx].tx.stream->CR & DMA_SxCR_CIRC)) {
        s_dma_tx_done_cbs[idx] = NULL;
        s_tx_args[idx]         = NULL;
    }

    // Finally, invoke the user callback
    local_cb(local_arg, ret);
}

[[__gnu__::__always_inline__]] static inline void isr_rx_helper(hal_err_t ret, uint8_t idx) {
    if (!s_dma_rx_done_cbs[idx]) {
        return;
    }

    // Save the user callback so we can clear it's global array position
    const dma_done_cb_t local_cb  = s_dma_rx_done_cbs[idx];
    void* const         local_arg = s_rx_args[idx];

    // Only clear the user callback if not in circular mode
    if (!(s_spi_dma_map[idx].rx.stream->CR & DMA_SxCR_CIRC)) {
        s_dma_rx_done_cbs[idx] = NULL;
        s_rx_args[idx]         = NULL;
    }

    // Finally, invoke the user callback
    local_cb(local_arg, ret);
}


// Public API
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
    if (handle == NULL || config == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    // Configure the GPIO pins
    TRY(gpiox_clk_enable(config->gpio_port, true));

    // Alternate function value selection for the GPIOs
    uint8_t alt_val = 0;
    if ((handle == SPI1) || (handle == SPI2)) {
        alt_val = 5;
    } else if (handle == SPI3) {
        alt_val = (config->gpio_port == GPIOD) ? 5U : 6;
    } else if (handle == SPI4) {
        alt_val = (config->gpio_port == GPIOE) ? 5U : 6;
    } else if (handle == SPI5) {
        alt_val = 6;
    } else {
        return HAL_ERR_INVALID_ARG;
    }

    if (config->use_miso) {
        TRY(gpio_set_alternate_function(config->gpio_port, config->miso_pin, alt_val));
        gpio_enable_pullup(config->gpio_port, config->miso_pin, true);
        gpio_set_speed_mode(config->gpio_port, config->miso_pin, GPIO_HIGH_SPEED);
        // MISO cannot be configured as push pull or open drain
    }

    if (config->use_mosi) {
        TRY(gpio_set_alternate_function(config->gpio_port, config->mosi_pin, alt_val));
        gpio_enable_pullup(config->gpio_port, config->mosi_pin, true);
        gpio_set_speed_mode(config->gpio_port, config->mosi_pin, GPIO_HIGH_SPEED);
        gpio_set_output_type(config->gpio_port, config->mosi_pin, GPIO_PUSH_PULL);
    }

    TRY(gpio_set_alternate_function(config->gpio_port, config->sclk_pin, alt_val));
    gpio_enable_pullup(config->gpio_port, config->sclk_pin, true);
    gpio_set_speed_mode(config->gpio_port, config->sclk_pin, GPIO_HIGH_SPEED);
    gpio_set_output_type(config->gpio_port, config->sclk_pin, GPIO_PUSH_PULL);

    // Disable the SPI peripheral
    handle->CR1 &= ~SPI_CR1_SPE;

    // SPI mode
    handle->I2SCFGR &= ~SPI_I2SCFGR_I2SMOD;

    handle->CR1 |= (((config->clk_divider & 0b111U) << SPI_CR1_BR_Pos) | // Clock prescaler
                    SPI_CR1_SSM |                                        // Software slave select control
                    SPI_CR1_SSI |                                        // Set internal slave to high
                    SPI_CR1_MSTR);                                       // SPI in master mode

    handle->CR1 &= ~(SPI_CR1_LSBFIRST | // MSB first
                     SPI_CR1_RXONLY |   // TX and RX
                     SPI_CR1_CRCEN |    // Hardware CRC disable
                     SPI_CR1_CRCNEXT |  // No CRC phase; data phase always
                     SPI_CR1_BIDIMODE | // MOSI and MISO used
                     SPI_CR1_BIDIOE);

    // CPOL and CPHA setting
    if (config->cpol) {
        handle->CR1 |= SPI_CR1_CPOL;
    } else {
        handle->CR1 &= ~SPI_CR1_CPOL;
    }

    if (config->cpha) {
        handle->CR1 |= SPI_CR1_CPHA;
    } else {
        handle->CR1 &= ~SPI_CR1_CPHA;
    }

    // 8 or 16 bit transfers
    if (config->use_8bit_mode) {
        handle->CR1 &= ~SPI_CR1_DFF;
    } else {
        handle->CR1 |= SPI_CR1_DFF;
    }

    // Motorolla mode and SS output disable
    handle->CR2 &= ~(SPI_CR2_FRF | SPI_CR2_SSOE);

    return HAL_OK;
}

hal_err_t spi_master_enable(SPI_TypeDef* handle, bool enable) {
    if (handle == NULL) {
        return HAL_ERR_INVALID_ARG;
    }
    if (enable) {
        handle->CR1 |= SPI_CR1_SPE;
    } else {
        handle->CR1 &= ~SPI_CR1_SPE;
    }
    return HAL_OK;
}

hal_err_t spi_master_dma_init(SPI_TypeDef* handle) {
    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU) {
        return HAL_ERR_INVALID_ARG;
    }

    // TX mapping
    DMA_TypeDef*        tx_controller = s_spi_dma_map[idx].tx.controller;
    DMA_Stream_TypeDef* tx_stream     = s_spi_dma_map[idx].tx.stream;
    const uint8_t       tx_channel    = s_spi_dma_map[idx].tx.channel;
    const uint8_t       tx_stream_no  = s_spi_dma_map[idx].tx.stream_no;
    const IRQn_Type     tx_irq_type   = s_spi_dma_map[idx].tx.irq_type;

    // RX mapping
    DMA_TypeDef*        rx_controller = s_spi_dma_map[idx].rx.controller;
    DMA_Stream_TypeDef* rx_stream     = s_spi_dma_map[idx].rx.stream;
    const uint8_t       rx_channel    = s_spi_dma_map[idx].rx.channel;
    const uint8_t       rx_stream_no  = s_spi_dma_map[idx].rx.stream_no;
    const IRQn_Type     rx_irq_type   = s_spi_dma_map[idx].rx.irq_type;

    if (tx_controller == NULL || tx_stream == NULL || rx_controller == NULL || rx_stream == NULL) {
        return HAL_ERR_NOT_SUPPORTED;
    }

    // TX stream configuration
    TRY(dmax_clk_enable(tx_controller, true));
    TRY(dma_clear_flags(tx_controller, tx_stream_no));
    TRY(dma_disable_stream(tx_stream));
    dma_set_channel(tx_stream, tx_channel);
    dma_set_direct_mode(tx_stream, true);
    dma_set_direction(tx_stream, DMA_DIR_M_P);
    dma_set_flow_controller(tx_stream, true);
    dma_set_stream_priority(tx_stream, DMA_PRIORITY_VERY_HIGH);
    dma_enable_circm_dbm(tx_stream, false, false);
    dma_set_increment(tx_stream, false, true);
    dma_enable_irqs(tx_stream, true, true, false, true);

    // RX stream configuration
    TRY(dmax_clk_enable(rx_controller, true));
    TRY(dma_clear_flags(rx_controller, rx_stream_no));
    TRY(dma_disable_stream(rx_stream));
    dma_set_channel(rx_stream, rx_channel);
    dma_set_direct_mode(rx_stream, true);
    dma_set_direction(rx_stream, DMA_DIR_P_M);
    dma_set_flow_controller(rx_stream, true);
    dma_set_stream_priority(rx_stream, DMA_PRIORITY_VERY_HIGH);
    dma_enable_circm_dbm(rx_stream, false, false);
    dma_set_increment(rx_stream, false, true);
    dma_enable_irqs(rx_stream, true, true, false, true);

    // If the DFF bit in SPIx_CR1 is set, then it's a 16 bit transfer
    const dma_data_size_t dma_data_size = (handle->CR1 & SPI_CR1_DFF) ? DMA_SIZE_HWORD : DMA_SIZE_BYTE;
    dma_set_per_mem_size(tx_stream, dma_data_size, dma_data_size);
    dma_set_per_mem_size(rx_stream, dma_data_size, dma_data_size);

    // Enable SPI requests to DMA
    handle->CR2 |= (SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);

    // Enable the DMA stream interrupts
    NVIC_SetPriority(tx_irq_type, SPI_DMA_NVIC_IRQ_PRIORITY);
    NVIC_EnableIRQ(tx_irq_type);

    NVIC_SetPriority(rx_irq_type, SPI_DMA_NVIC_IRQ_PRIORITY);
    NVIC_EnableIRQ(rx_irq_type);

    return HAL_OK;
}


// Polling API
hal_err_t spi_master_transmit_poll(SPI_TypeDef* handle, const void* data, size_t size) {
    if (handle == NULL || data == NULL || size == 0) {
        return HAL_ERR_INVALID_ARG;
    }

    const bool is_16bit_data = handle->CR1 & SPI_CR1_DFF;

    if (is_16bit_data) {
        // Cast to appropriate type
        const uint16_t* buf = (const uint16_t*)data;

        for (size_t i = 0; i < size; i++) {
            // Write data
            handle->DR = buf[i];

            // Poll till the data has been transferred out
            uint32_t timeout = TIMEOUT_CYCLES;
            while (!(handle->SR & SPI_SR_TXE) && (--timeout));
            if (timeout == 0) {
                return HAL_ERR_TIMEOUT;
            }

            // Poll till the data has been received
            timeout = TIMEOUT_CYCLES;
            while (!(handle->SR & SPI_SR_RXNE) && (--timeout));
            if (timeout == 0) {
                return HAL_ERR_TIMEOUT;
            }

            // Read the data register and discard the value
            (void)handle->DR;
        }

    } else {
        // Cast to appropriate type
        const uint8_t* buf = (const uint8_t*)data;

        for (size_t i = 0; i < size; i++) {
            // Write data
            handle->DR = buf[i];

            // Poll till data has been transferred out
            uint32_t timeout = TIMEOUT_CYCLES;
            while (!(handle->SR & SPI_SR_TXE) && (--timeout));
            if (timeout == 0) {
                return HAL_ERR_TIMEOUT;
            }

            // Wait till data has been received
            timeout = TIMEOUT_CYCLES;
            while (!(handle->SR & SPI_SR_RXNE) && (--timeout));
            if (timeout == 0) {
                return HAL_ERR_TIMEOUT;
            }

            // Read the data register and discard the value
            (void)handle->DR;
        }
    }

    // Wait for the TXE and BSY bits
    // TXE bit
    uint32_t timeout = TIMEOUT_CYCLES;
    while (!(handle->SR & SPI_SR_TXE) && (--timeout));
    if (timeout == 0) {
        return HAL_ERR_TIMEOUT;
    }
    // BSY bit
    timeout = TIMEOUT_CYCLES;
    while ((handle->SR & SPI_SR_BSY) && (--timeout));
    if (timeout == 0) {
        return HAL_ERR_TIMEOUT;
    }

    return HAL_OK;
}

hal_err_t spi_master_receive_poll(SPI_TypeDef* handle, void* data, size_t size) {
    if (handle == NULL || data == NULL || size == 0) {
        return HAL_ERR_INVALID_ARG;
    }

    const bool is_16bit_data = handle->CR1 & SPI_CR1_DFF;

    if (is_16bit_data) {
        // Cast to appropriate type
        uint16_t* buf = (uint16_t*)data;

        for (size_t i = 0; i < size; i++) {
            // Write dummy data
            handle->DR = 0x00UL;

            // Poll till the data has been transferred out
            uint32_t timeout = TIMEOUT_CYCLES;
            while (!(handle->SR & SPI_SR_TXE) && (--timeout));
            if (timeout == 0) {
                return HAL_ERR_TIMEOUT;
            }

            // Poll till the data has been received
            timeout = TIMEOUT_CYCLES;
            while (!(handle->SR & SPI_SR_RXNE) && (--timeout));
            if (timeout == 0) {
                return HAL_ERR_TIMEOUT;
            }

            // Read data
            buf[i] = (uint16_t)handle->DR;
        }

    } else {
        // Cast to appropriate type
        uint8_t* buf = (uint8_t*)data;

        for (size_t i = 0; i < size; i++) {
            // Write dummy data
            handle->DR = 0x00UL;

            // Poll till data has been transferred out
            uint32_t timeout = TIMEOUT_CYCLES;
            while (!(handle->SR & SPI_SR_TXE) && (--timeout));
            if (timeout == 0) {
                return HAL_ERR_TIMEOUT;
            }

            // Wait till data has been received
            timeout = TIMEOUT_CYCLES;
            while (!(handle->SR & SPI_SR_RXNE) && (--timeout));
            if (timeout == 0) {
                return HAL_ERR_TIMEOUT;
            }

            // Read data
            buf[i] = (uint8_t)handle->DR;
        }
    }

    // Wait for the TXE and BSY bits
    // TXE bit
    uint32_t timeout = TIMEOUT_CYCLES;
    while (!(handle->SR & SPI_SR_TXE) && (--timeout));
    if (timeout == 0) {
        return HAL_ERR_TIMEOUT;
    }
    // BSY bit
    timeout = TIMEOUT_CYCLES;
    while ((handle->SR & SPI_SR_BSY) && (--timeout));
    if (timeout == 0) {
        return HAL_ERR_TIMEOUT;
    }

    return HAL_OK;
}

hal_err_t spi_master_transceive_poll(SPI_TypeDef* handle, const void* tx_data, void* rx_data, size_t size) {
    if (handle == NULL || tx_data == NULL || rx_data == NULL || size == 0) {
        return HAL_ERR_INVALID_ARG;
    }

    const bool is_16bit_data = handle->CR1 & SPI_CR1_DFF;

    if (is_16bit_data) {
        // Cast to appropriate type
        const uint16_t* tx_buf = (const uint16_t*)tx_data;
        uint16_t*       rx_buf = (uint16_t*)rx_data;

        for (size_t i = 0; i < size; i++) {
            // Write data
            handle->DR = tx_buf[i];

            // Poll till data has been transferred out
            uint32_t timeout = TIMEOUT_CYCLES;
            while (!(handle->SR & SPI_SR_TXE) && (--timeout));
            if (timeout == 0) {
                return HAL_ERR_TIMEOUT;
            }

            // Poll till the data has been received
            timeout = TIMEOUT_CYCLES;
            while (!(handle->SR & SPI_SR_RXNE) && (--timeout));
            if (timeout == 0) {
                return HAL_ERR_TIMEOUT;
            }

            // Finally, read the data register
            rx_buf[i] = (uint16_t)handle->DR;
        }

    } else {
        // Cast to appropriate type
        const uint8_t* tx_buf = (const uint8_t*)tx_data;
        uint8_t*       rx_buf = (uint8_t*)rx_data;

        for (size_t i = 0; i < size; i++) {
            // Write data
            handle->DR = tx_buf[i];

            // Poll till data has been transferred out
            uint32_t timeout = TIMEOUT_CYCLES;
            while (!(handle->SR & SPI_SR_TXE) && (--timeout));
            if (timeout == 0) {
                return HAL_ERR_TIMEOUT;
            }

            // Wait till the data has been received
            timeout = TIMEOUT_CYCLES;
            while (!(handle->SR & SPI_SR_RXNE) && (--timeout));
            if (timeout == 0) {
                return HAL_ERR_TIMEOUT;
            }

            // Finally, read the data register
            rx_buf[i] = (uint8_t)handle->DR;
        }
    }

    // Wait for the TXE and BSY bits
    // TXE bit
    uint32_t timeout = TIMEOUT_CYCLES;
    while (!(handle->SR & SPI_SR_TXE) && (--timeout));
    if (timeout == 0) {
        return HAL_ERR_TIMEOUT;
    }
    // BSY bit
    timeout = TIMEOUT_CYCLES;
    while ((handle->SR & SPI_SR_BSY) && (--timeout));
    if (timeout == 0) {
        return HAL_ERR_TIMEOUT;
    }

    return HAL_OK;
}


// DMA transfers API
hal_err_t spi_master_transmit_dma(SPI_TypeDef* handle, const void* data, uint16_t size, dma_done_cb_t cb, void* arg) {
    // Get index for DMA stream mapping
    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU) {
        return HAL_ERR_INVALID_ARG;
    }

    if (data == NULL || size == 0) {
        return HAL_ERR_INVALID_ARG;
    }

    // TX mapping
    DMA_Stream_TypeDef* stream = s_spi_dma_map[idx].tx.stream;
    if (stream == NULL) {
        return HAL_ERR_NOT_SUPPORTED;
    }

    // Set the memory address and transaction length
    dma_set_addresses(stream, &handle->DR, data, NULL);
    dma_set_trans_length(stream, size);

    // Save the user passed callback
    if (cb) {
        s_dma_tx_done_cbs[idx] = cb;
        s_tx_args[idx]         = arg;
    }

    // Enable the DMA TX stream
    return dma_enable_stream(stream);
}

hal_err_t spi_master_receive_dma(SPI_TypeDef* handle, void* data, uint16_t size, dma_done_cb_t cb, void* arg) {
    // Get index for DMA stream mapping
    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU) {
        return HAL_ERR_INVALID_ARG;
    }

    if (data == NULL || size == 0) {
        return HAL_ERR_INVALID_ARG;
    }

    // RX mapping
    DMA_Stream_TypeDef* stream = s_spi_dma_map[idx].rx.stream;
    if (stream == NULL) {
        return HAL_ERR_NOT_SUPPORTED;
    }

    // Set the memory address and transaction length
    dma_set_addresses(stream, &handle->DR, data, NULL);
    dma_set_trans_length(stream, size);

    // Save the user passed callback
    if (cb) {
        s_dma_rx_done_cbs[idx] = cb;
        s_rx_args[idx]         = arg;
    }

    // Enable the DMA RX stream
    return dma_enable_stream(stream);
}

hal_err_t spi_master_transceive_dma(SPI_TypeDef* handle, const void* tx_data, void* rx_data, uint16_t size, dma_done_cb_t cb, void* arg) {
    // Get index for DMA stream mapping
    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU) {
        return HAL_ERR_INVALID_ARG;
    }

    if (tx_data == NULL || rx_data == NULL || size == 0) {
        return HAL_ERR_INVALID_ARG;
    }

    // TX and RX mapping
    DMA_Stream_TypeDef* tx_stream = s_spi_dma_map[idx].tx.stream;
    DMA_Stream_TypeDef* rx_stream = s_spi_dma_map[idx].rx.stream;
    if (tx_stream == NULL || rx_stream == NULL) {
        return HAL_ERR_NOT_SUPPORTED;
    }

    // Set the memory address and transaction length
    dma_set_addresses(tx_stream, &handle->DR, tx_data, NULL);
    dma_set_trans_length(tx_stream, size);

    dma_set_addresses(rx_stream, &handle->DR, rx_data, NULL);
    dma_set_trans_length(rx_stream, size);

    // Save the user passed callback
    if (cb) {
        // Save cb to the TX DMA irq only
        s_dma_tx_done_cbs[idx] = cb;
        s_tx_args[idx]         = arg;
    }

    // Enable DMA TX and RX streams
    TRY(dma_enable_stream(tx_stream));
    TRY_WITH_FUNC(dma_enable_stream(rx_stream), dma_disable_stream(tx_stream));

    return HAL_OK;
}


// To be used only by i2s.c
void spi_master_register_callback(dma_done_cb_t cb, void* arg, uint8_t idx, bool tx) {
    if (tx) {
        s_dma_tx_done_cbs[idx] = cb;
        s_tx_args[idx]         = arg;
    } else {
        s_dma_rx_done_cbs[idx] = cb;
        s_rx_args[idx]         = arg;
    }
}


// DMA interrupts
// SPI2: TX
void DMA1_Stream4_IRQHandler(void) {
    hal_err_t ret = dma_isr_helper(DMA1_Stream4, &DMA1->HIFCR, &DMA1->HISR, DMA_HISR_TCIF4, DMA_HISR_TEIF4, DMA_HISR_DMEIF4, DMA_HISR_HTIF4);
    isr_tx_helper(SPI2, ret, get_index(SPI2));
}

// SPI2: RX
void DMA1_Stream3_IRQHandler(void) {
    hal_err_t ret = dma_isr_helper(DMA1_Stream3, &DMA1->LIFCR, &DMA1->LISR, DMA_LISR_TCIF3, DMA_LISR_TEIF3, DMA_LISR_DMEIF3, DMA_LISR_HTIF3);
    isr_rx_helper(ret, get_index(SPI2));
}

// SPI3: TX
void DMA1_Stream7_IRQHandler(void) {
    hal_err_t ret = dma_isr_helper(DMA1_Stream7, &DMA1->HIFCR, &DMA1->HISR, DMA_HISR_TCIF7, DMA_HISR_TEIF7, DMA_HISR_DMEIF7, DMA_HISR_HTIF7);
    isr_tx_helper(SPI3, ret, get_index(SPI3));
}

// SPI3: RX
void DMA1_Stream2_IRQHandler(void) {
    hal_err_t ret = dma_isr_helper(DMA1_Stream2, &DMA1->LIFCR, &DMA1->LISR, DMA_LISR_TCIF2, DMA_LISR_TEIF2, DMA_LISR_DMEIF2, DMA_LISR_HTIF2);
    isr_rx_helper(ret, get_index(SPI3));
}

// SPI4: TX
void DMA2_Stream1_IRQHandler(void) {
    hal_err_t ret = dma_isr_helper(DMA2_Stream1, &DMA2->LIFCR, &DMA2->LISR, DMA_LISR_TCIF1, DMA_LISR_TEIF1, DMA_LISR_DMEIF1, DMA_LISR_HTIF1);
    isr_tx_helper(SPI4, ret, get_index(SPI4));
}

// SPI4: RX
void DMA2_Stream4_IRQHandler(void) {
    hal_err_t ret = dma_isr_helper(DMA2_Stream4, &DMA2->HIFCR, &DMA2->HISR, DMA_HISR_TCIF4, DMA_HISR_TEIF4, DMA_HISR_DMEIF4, DMA_HISR_HTIF4);
    isr_rx_helper(ret, get_index(SPI4));
}
