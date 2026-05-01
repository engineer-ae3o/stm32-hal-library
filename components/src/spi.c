#include "stm32f411xe.h"
#include "spi.h"
#include "dma.h"
#include "gpio.h"


// The 5 SPI instances: ISRs called when the DMA is done
// TX
spi_trans_done_cb_t s_dma_tx_done_cbs[5] = {};
void* s_tx_args[5] = {};
// RX
spi_trans_done_cb_t s_dma_rx_done_cbs[5] = {};
void* s_rx_args[5] = {};

// Utilities for mapping the SPI instances to DMA streams
typedef struct {
    DMA_TypeDef* controller;
    DMA_Stream_TypeDef* stream;
    uint8_t channel;
} spi_dma_t;

typedef struct {
    const spi_dma_t tx;
    const spi_dma_t rx;
} dma_stream_map_t;

#define SPI_DMA_NVIC_IRQ_PRIORITY 8U
#define TIMEOUT_CYCLES 100UL

// Mapping for the DMA channels for the 5 SPI channels
static const dma_stream_map_t s_spi_dma_map[5] = {
    // SPI1
    {
        .tx = { .controller = DMA2, .stream = DMA2_Stream5, .channel = 3 },
        .rx = { .controller = DMA2, .stream = DMA2_Stream2, .channel = 3 }
    },
    // SPI2
    {
        .tx = { .controller = DMA1, .stream = DMA1_Stream4, .channel = 0 },
        .rx = { .controller = DMA1, .stream = DMA1_Stream3, .channel = 0 }
    },
    // SPI3
    {
        .tx = { .controller = DMA1, .stream = DMA1_Stream7, .channel = 0 },
        .rx = { .controller = DMA1, .stream = DMA1_Stream2, .channel = 0 }
    },
    // SPI4
    {
        .tx = { .controller = DMA2, .stream = DMA2_Stream1, .channel = 4 },
        .rx = { .controller = DMA2, .stream = DMA2_Stream0, .channel = 4 }
    },
    // SPI5
    {
        .tx = { .controller = DMA2, .stream = DMA2_Stream4, .channel = 2 },
        .rx = { .controller = DMA2, .stream = DMA2_Stream3, .channel = 2 }
    }
};

// Helpers
static inline uint8_t get_index(const SPI_TypeDef* handle) {
    if      (handle == SPI1) return 0U;
    else if (handle == SPI2) return 1U;
    else if (handle == SPI3) return 2U;
    else if (handle == SPI4) return 3U;
    else if (handle == SPI5) return 4U;
    else                     return 0xFFU;
}

static inline spi_dma_err_t isr_helper(DMA_Stream_TypeDef* stream, volatile uint32_t* irq_clr_rg,
                                       volatile uint32_t* irq_sta_rg, uint32_t tc, uint32_t te, uint32_t dme) {

    spi_dma_err_t error = SPI_NO_ERROR;

    // Transfer complete
    if (*irq_sta_rg & tc) {
        // Clear DMA TC interrupt bit
        *irq_clr_rg = tc;
    }
    // Transfer error
    else if (*irq_sta_rg & te) {
        // Clear DMA TE interrupt bit
        *irq_clr_rg = te;
        error = SPI_DMA_TE;
    }
    // Direct mode error
    else if (*irq_sta_rg & dme) {
        // Clear DMA DME interrupt bit
        *irq_clr_rg = dme;
        error = SPI_DMA_DME;
    }
    // Unreachable, but default catch-all
    else {
        // Clear DMA TC, DME and TE interrupt bits
        *irq_clr_rg = (tc | te | dme);
        error = SPI_DMA_ERR_UNKNOWN;
    }
    
    if (dma_disable_stream(stream) != HAL_OK) error = SPI_DMA_TIMEOUT;

    return error;
}

static inline void isr_tx_helper(SPI_TypeDef* handle, spi_dma_err_t ret, uint8_t idx) {

    if (!s_dma_tx_done_cbs[idx]) return;

    // Transfers require us to poll on the TXE and BSY flags even after data has
    // been shifted out. If they're not in the required state, an error occurred
    
    // Poll till TXE has been set
    // Skip polling if an error has occurred
    if (ret == SPI_NO_ERROR) {
        uint32_t timeout = TIMEOUT_CYCLES;
        while (!(handle->SR & SPI_SR_TXE) && (--timeout));
        if (timeout == 0) ret = SPI_TXE_FAILED_TO_SET;
    }

    // Poll till BSY has been cleared
    // Skip polling BSY if TXE failed to set
    if (ret == SPI_NO_ERROR) {
        uint32_t timeout = TIMEOUT_CYCLES;
        while ((handle->SR & SPI_SR_BSY) && (--timeout));
        if (timeout == 0) ret = SPI_BSY_FAILED_TO_CLEAR;
    }

    // Invoke user callback
    s_dma_tx_done_cbs[idx](s_tx_args[idx], ret);

    // Clear user passed context
    s_dma_tx_done_cbs[idx] = NULL;
    s_tx_args[idx] = NULL;
}

static inline void isr_rx_helper(SPI_TypeDef* handle, spi_dma_err_t ret, uint8_t idx) {

    if (!s_dma_rx_done_cbs[idx]) return;
    
    // Invoke user callback
    s_dma_rx_done_cbs[idx](s_rx_args[idx], ret);

    // Clear user passed context
    s_dma_rx_done_cbs[idx] = NULL;
    s_rx_args[idx] = NULL;
}

// Public API
hal_err_t spi_master_init(SPI_TypeDef* handle, const spi_master_config_t* config) {
    
    // Enable SPI clock
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
        return HAL_INVALID_ARG;
    }

    __DSB();

    // Init GPIO
    hal_err_t ret = gpiox_clk_enable(config->gpio_port);
    if (ret != HAL_OK) return ret;

    // Alternate function value selection for the GPIOs
    uint8_t alt_val = 0;
    if ((handle == SPI1) || (handle == SPI2)) alt_val = 5U;
    else if (handle == SPI3) alt_val = (config->gpio_port == GPIOD) ? 5U : 6U;
    else if (handle == SPI4) alt_val = (config->gpio_port == GPIOE) ? 5U : 6U;
    else if (handle == SPI5) alt_val = 6U;
    else return HAL_INVALID_ARG;

    if (config->use_miso) {
        ret = gpio_set_alternate_function(config->gpio_port, config->miso, alt_val);
        if (ret != HAL_OK) return ret;
        gpio_enable_pullup(config->gpio_port, config->miso, true);
        gpio_set_speed_mode(config->gpio_port, config->miso, GPIO_HIGH_SPEED);
        gpio_set_output_type(config->gpio_port, config->miso, GPIO_PUSH_PULL);
    }
    
    if (config->use_mosi) {
        ret = gpio_set_alternate_function(config->gpio_port, config->mosi, alt_val);
        if (ret != HAL_OK) return ret;
        gpio_enable_pullup(config->gpio_port, config->mosi, true);
        gpio_set_speed_mode(config->gpio_port, config->mosi, GPIO_HIGH_SPEED);
        gpio_set_output_type(config->gpio_port, config->mosi, GPIO_PUSH_PULL);
    }

    ret = gpio_set_alternate_function(config->gpio_port, config->sclk, alt_val);
    if (ret != HAL_OK) return ret;
    gpio_set_speed_mode(config->gpio_port, config->sclk, GPIO_HIGH_SPEED);
    gpio_set_output_type(config->gpio_port, config->sclk, GPIO_PUSH_PULL);
    
    // Disable the SPI peripheral
    handle->CR1 &= ~SPI_CR1_SPE;

    // SPI mode
    handle->I2SCFGR &= ~SPI_I2SCFGR_I2SMOD;

    handle->CR1 |= (((config->clk_divider & 0b111U) << SPI_CR1_BR_Pos) | // Clock prescaler
                    SPI_CR1_SSM  | // Software slave select control
                    SPI_CR1_SSI  | // Set internal slave to high
                    SPI_CR1_MSTR); // SPI in master mode
    
    handle->CR1 &= ~(SPI_CR1_LSBFIRST | // MSB first
                     SPI_CR1_RXONLY   | // TX and RX
                     SPI_CR1_CRCEN    | // Hardware CRC disable
                     SPI_CR1_CRCNEXT  | // No CRC phase; data phase always
                     SPI_CR1_BIDIMODE | // MOSI and MISO used
                     SPI_CR1_BIDIOE);
    
    // CPOL and CPHA setting
    if (config->cpol) handle->CR1 |= SPI_CR1_CPOL;
    else              handle->CR1 &= ~SPI_CR1_CPOL;

    if (config->cpha) handle->CR1 |= SPI_CR1_CPHA;
    else              handle->CR1 &= ~SPI_CR1_CPHA;

    // 8 or 16 bit transfers
    if (config->use_8bit_mode) handle->CR1 &= ~SPI_CR1_DFF;
    else                       handle->CR1 |= SPI_CR1_DFF;
    
    // Motorolla mode and SS output disable
    handle->CR2 &= ~(SPI_CR2_FRF | SPI_CR2_SSOE);

    // Enable the SPI peripheral
    handle->CR1 |= SPI_CR1_SPE;

    return HAL_OK;
}

hal_err_t spi_master_dma_init(SPI_TypeDef* handle) {
    
    // DMA irq types
    IRQn_Type tx_dma_irq = 0;
    IRQn_Type rx_dma_irq = 0;
    
    if (handle == SPI1) {
        tx_dma_irq = DMA2_Stream7_IRQn;
        rx_dma_irq = DMA2_Stream5_IRQn;

    } else if (handle == SPI2) {
        tx_dma_irq = DMA1_Stream6_IRQn;
        rx_dma_irq = DMA1_Stream5_IRQn;

    } else if (handle == SPI3) {
        tx_dma_irq = DMA2_Stream6_IRQn;
        rx_dma_irq = DMA2_Stream1_IRQn;

    } else if (handle == SPI4) {
        tx_dma_irq = DMA2_Stream6_IRQn;
        rx_dma_irq = DMA2_Stream1_IRQn;

    } else if (handle == SPI5) {
        tx_dma_irq = DMA2_Stream6_IRQn;
        rx_dma_irq = DMA2_Stream1_IRQn;

    } else {
        return HAL_INVALID_ARG;
    }
    
    const uint8_t idx = get_index(handle);
    
    // TX mapping
    DMA_TypeDef* tx_controller = s_spi_dma_map[idx].tx.controller;
    DMA_Stream_TypeDef* tx_stream = s_spi_dma_map[idx].tx.stream;
    uint8_t tx_channel = s_spi_dma_map[idx].tx.channel;

    // RX mapping
    DMA_TypeDef* rx_controller = s_spi_dma_map[idx].rx.controller;
    DMA_Stream_TypeDef* rx_stream = s_spi_dma_map[idx].rx.stream;
    uint8_t rx_channel = s_spi_dma_map[idx].rx.channel;
    
    hal_err_t ret = dmax_clk_enable(tx_controller);
    if (ret != HAL_OK) return ret;
    ret = dmax_clk_enable(rx_controller);
    if (ret != HAL_OK) return ret;
    
    // Disable DMA stream before configuring
    ret = dma_disable_stream(tx_stream);
    if (ret != HAL_OK) return ret;
    ret = dma_disable_stream(rx_stream);
    if (ret != HAL_OK) return ret;

    dma_clear_flags(tx_controller);
    dma_set_channel(tx_stream, tx_channel);
    dma_set_direct_mode(tx_stream, true);
    dma_set_direction(tx_stream, DMA_DIR_M_P);
    dma_set_flow_controller(tx_stream, true);
    dma_set_stream_priority(tx_stream, DMA_PRIORITY_VERY_HIGH);
    dma_enable_circm_dbm(tx_stream, false, false);
    dma_set_increment(tx_stream, false, true);
    dma_enable_irqs(tx_stream, true, true, false, true);
    dma_set_addresses(tx_stream, &handle->DR, NULL, NULL);
    
    dma_clear_flags(rx_controller);
    dma_set_channel(rx_stream, rx_channel);
    dma_set_direct_mode(rx_stream, true);
    dma_set_direction(rx_stream, DMA_DIR_P_M);
    dma_set_flow_controller(rx_stream, true);
    dma_set_stream_priority(rx_stream, DMA_PRIORITY_VERY_HIGH);
    dma_enable_circm_dbm(rx_stream, false, false);
    dma_set_increment(rx_stream, false, true);
    dma_enable_irqs(rx_stream, true, true, false, true);
    dma_set_addresses(rx_stream, &handle->DR, NULL, NULL);
    
    // If the DFF bit in SPIx_CR1 is set, then it's a 16 bit transfer
    const dma_data_size_t dma_data_size = (handle->CR1 & SPI_CR1_DFF) ? DMA_SIZE_HWORD : DMA_SIZE_BYTE;
    dma_set_per_mem_size(tx_stream, dma_data_size, dma_data_size);
    dma_set_per_mem_size(rx_stream, dma_data_size, dma_data_size);
    
    // Enable SPI requests to DMA
    handle->CR2 |= (SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);
    
    // Enable DMA stream interrupts
    NVIC_EnableIRQ(tx_dma_irq);
    NVIC_SetPriority(tx_dma_irq, SPI_DMA_NVIC_IRQ_PRIORITY);

    NVIC_EnableIRQ(rx_dma_irq);
    NVIC_SetPriority(rx_dma_irq, SPI_DMA_NVIC_IRQ_PRIORITY);

    return HAL_OK;
}

// Polling API
hal_err_t spi_master_transmit_poll(SPI_TypeDef* handle, const void* data, size_t len) {
    
    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU) return HAL_INVALID_ARG;

    const bool is_16bit_data = handle->CR1 & SPI_CR1_DFF;

    if (is_16bit_data) {
        // Cast to appropriate type
        const uint16_t* buf = (const uint16_t*)data;
        
        for (size_t i = 0; i < len; i++) {
            // Write data
            handle->DR = buf[i];

            // Poll till data has been transferred out
            uint32_t timeout = TIMEOUT_CYCLES;
            while (!(handle->SR & SPI_SR_TXE) && (--timeout));
            if (timeout == 0) return HAL_TIMEOUT;

            // Poll till data has been received
            timeout = TIMEOUT_CYCLES;
            while (!(handle->SR & SPI_SR_RXNE) && (--timeout));
            if (timeout == 0) return HAL_TIMEOUT;

            // Read data and discard
            (void)handle->DR;
        }

    } else {
        // Cast to appropriate type
        const uint8_t* buf = (const uint8_t*)data;
        
        for (size_t i = 0; i < len; i++) {
            // Write data
            handle->DR = buf[i];

            // Poll till data has been transferred out
            uint32_t timeout = TIMEOUT_CYCLES;
            while (!(handle->SR & SPI_SR_TXE) && (--timeout));
            if (timeout == 0) return HAL_TIMEOUT;

            // Wait till data has been received
            timeout = TIMEOUT_CYCLES;
            while (!(handle->SR & SPI_SR_RXNE) && (--timeout));
            if (timeout == 0) return HAL_TIMEOUT;

            // Read data and discard
            (void)handle->DR;
        }
    }
    
    // Wait for the TXE and BSY bits
    // TXE bit
    uint32_t timeout = TIMEOUT_CYCLES;
    while (!(handle->SR & SPI_SR_TXE) && (--timeout));
    if (timeout == 0) return HAL_TIMEOUT;
    // BSY bit
    timeout = TIMEOUT_CYCLES;
    while ((handle->SR & SPI_SR_BSY) && (--timeout));
    if (timeout == 0) return HAL_TIMEOUT;

    return HAL_OK;
}

hal_err_t spi_master_receive_poll(SPI_TypeDef* handle, void* data, size_t len) {
    
    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU) return HAL_INVALID_ARG;

    const bool is_16bit_data = handle->CR1 & SPI_CR1_DFF;

    if (is_16bit_data) {
        // Cast to appropriate type
        uint16_t* buf = (uint16_t*)data;
        
        for (size_t i = 0; i < len; i++) {
            // Write dummy data
            handle->DR = 0x00UL;

            // Poll till data has been transferred out
            uint32_t timeout = TIMEOUT_CYCLES;
            while (!(handle->SR & SPI_SR_TXE) && (--timeout));
            if (timeout == 0) return HAL_TIMEOUT;

            // Poll till data has been received
            timeout = TIMEOUT_CYCLES;
            while (!(handle->SR & SPI_SR_RXNE) && (--timeout));
            if (timeout == 0) return HAL_TIMEOUT;

            // Read data
            buf[i] = (uint16_t)handle->DR;
        }

    } else {
        // Cast to appropriate type
        uint8_t* buf = (uint8_t*)data;
        
        for (size_t i = 0; i < len; i++) {
            // Write dummy data
            handle->DR = 0x00UL;

            // Poll till data has been transferred out
            uint32_t timeout = TIMEOUT_CYCLES;
            while (!(handle->SR & SPI_SR_TXE) && (--timeout));
            if (timeout == 0) return HAL_TIMEOUT;

            // Wait till data has been received
            timeout = TIMEOUT_CYCLES;
            while (!(handle->SR & SPI_SR_RXNE) && (--timeout));
            if (timeout == 0) return HAL_TIMEOUT;

            // Read data
            buf[i] = (uint8_t)handle->DR;
        }
    }
    
    // Wait for the TXE and BSY bits
    // TXE bit
    uint32_t timeout = TIMEOUT_CYCLES;
    while (!(handle->SR & SPI_SR_TXE) && (--timeout));
    if (timeout == 0) return HAL_TIMEOUT;
    // BSY bit
    timeout = TIMEOUT_CYCLES;
    while ((handle->SR & SPI_SR_BSY) && (--timeout));
    if (timeout == 0) return HAL_TIMEOUT;

    return HAL_OK;
}

hal_err_t spi_master_transmit_receive_poll(SPI_TypeDef* handle, const void* tx_data,
                                           void* rx_data, size_t len) {
    
    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU) return HAL_INVALID_ARG;

    const bool is_16bit_data = handle->CR1 & SPI_CR1_DFF;

    if (is_16bit_data) {
        // Cast to appropriate type
        const uint16_t* tx_buf = (const uint16_t*)tx_data;
        uint16_t* rx_buf = (uint16_t*)rx_data;

        for (size_t i = 0; i < len; i++) {
            // Write data
            handle->DR = tx_buf[i];

            // Poll till data has been transferred out
            uint32_t timeout = TIMEOUT_CYCLES;
            while (!(handle->SR & SPI_SR_TXE) && (--timeout));
            if (timeout == 0) return HAL_TIMEOUT;

            // Poll till data has been received
            timeout = TIMEOUT_CYCLES;
            while (!(handle->SR & SPI_SR_RXNE) && (--timeout));
            if (timeout == 0) return HAL_TIMEOUT;

            // Finally, read data
            rx_buf[i] = (uint16_t)handle->DR;
        }

    } else {
        // Cast to appropriate type
        const uint8_t* tx_buf = (const uint8_t*)tx_data;
        uint8_t* rx_buf = (uint8_t*)rx_data;

        for (size_t i = 0; i < len; i++) {
            // Write data
            handle->DR = tx_buf[i];

            // Poll till data has been transferred out
            uint32_t timeout = TIMEOUT_CYCLES;
            while (!(handle->SR & SPI_SR_TXE) && (--timeout));
            if (timeout == 0) return HAL_TIMEOUT;

            // Wait till data has been received
            timeout = TIMEOUT_CYCLES;
            while (!(handle->SR & SPI_SR_RXNE) && (--timeout));
            if (timeout == 0) return HAL_TIMEOUT;

            // Finally, read data
            rx_buf[i] = (uint8_t)handle->DR;
        }
    }
    
    // Wait for the TXE and BSY bits
    // TXE bit
    uint32_t timeout = TIMEOUT_CYCLES;
    while (!(handle->SR & SPI_SR_TXE) && (--timeout));
    if (timeout == 0) return HAL_TIMEOUT;
    // BSY bit
    timeout = TIMEOUT_CYCLES;
    while ((handle->SR & SPI_SR_BSY) && (--timeout));
    if (timeout == 0) return HAL_TIMEOUT;

    return HAL_OK;
}

// DMA transfers API
hal_err_t spi_master_transmit_dma(SPI_TypeDef* handle, const void* data, uint16_t len,
                                  spi_trans_done_cb_t callback, void* arg) {
    
    // Get index for DMA stream mapping
    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU) return HAL_INVALID_ARG;
    
    // Save user passed callback
    if (callback) {
        s_dma_tx_done_cbs[idx] = callback;
        s_tx_args[idx] = arg;
    }
    
    // TX mapping
    DMA_Stream_TypeDef* stream = s_spi_dma_map[idx].tx.stream;
    
    // Set memory address and length
    dma_set_addresses(stream, NULL, data, NULL);
    dma_set_trans_length(stream, len);
    
    // Enable DMA TX stream
    return dma_enable_stream(stream);
}

hal_err_t spi_master_receive_dma(SPI_TypeDef* handle, void* data, uint16_t len,
                                 spi_trans_done_cb_t callback, void* arg) {
    
    // Get index for DMA stream mapping
    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU) return HAL_INVALID_ARG;
    
    // Save user passed callback
    if (callback) {
        s_dma_rx_done_cbs[idx] = callback;
        s_rx_args[idx] = arg;
    }
    
    // RX mapping
    DMA_Stream_TypeDef* stream = s_spi_dma_map[idx].rx.stream;
    
    // Set memory address and length
    dma_set_addresses(stream, NULL, data, NULL);
    dma_set_trans_length(stream, len);
    
    // Enable DMA RX stream
    return dma_enable_stream(stream);
}

hal_err_t spi_master_transmit_receive_dma(SPI_TypeDef* handle, const void* tx_data, void* rx_data,
                                          uint16_t len, spi_trans_done_cb_t callback, void* arg) {
    
    // Get index for DMA stream mapping
    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU) return HAL_INVALID_ARG;
    
    // Save user passed callback
    if (callback) {
        // Save callback to the TX DMA irq only
        s_dma_tx_done_cbs[idx] = callback;
        s_tx_args[idx] = arg;
    }
    
    // TX and RX mapping
    DMA_Stream_TypeDef* tx_stream = s_spi_dma_map[idx].tx.stream;
    DMA_Stream_TypeDef* rx_stream = s_spi_dma_map[idx].rx.stream;
    
    // Set memory address and length
    dma_set_addresses(tx_stream, NULL, tx_data, NULL);
    dma_set_trans_length(tx_stream, len);

    dma_set_addresses(rx_stream, NULL, rx_data, NULL);
    dma_set_trans_length(rx_stream, len);
    
    // Enable DMA TX and RX streams
    hal_err_t ret = dma_enable_stream(tx_stream);
    if (ret != HAL_OK) return ret;

    return dma_enable_stream(rx_stream);
}

// DMA interrupts
// SPI1: TX
void DMA2_Stream5_IRQHandler(void) {
    spi_dma_err_t ret = isr_helper(DMA2_Stream5, &DMA2->HIFCR, &DMA2->HISR, DMA_HISR_TCIF5, DMA_HISR_TEIF5, DMA_HISR_DMEIF5);
    isr_tx_helper(SPI1, ret, 0);
}

// SPI1: RX
void DMA2_Stream2_IRQHandler(void) {
    spi_dma_err_t ret = isr_helper(DMA2_Stream2, &DMA2->LIFCR, &DMA2->LISR, DMA_LISR_TCIF2, DMA_LISR_TEIF2, DMA_LISR_DMEIF2);
    isr_rx_helper(SPI1, ret, 0);
}

// SPI2: TX
void DMA1_Stream4_IRQHandler(void) {
    spi_dma_err_t ret = isr_helper(DMA1_Stream4, &DMA1->HIFCR, &DMA1->HISR, DMA_HISR_TCIF4, DMA_HISR_TEIF4, DMA_HISR_DMEIF4);
    isr_tx_helper(SPI2, ret, 1);
}

// SPI2: RX
void DMA1_Stream3_IRQHandler(void) {
    spi_dma_err_t ret = isr_helper(DMA1_Stream3, &DMA2->LIFCR, &DMA2->LISR, DMA_LISR_TCIF3, DMA_LISR_TEIF3, DMA_LISR_DMEIF3);
    isr_rx_helper(SPI2, ret, 1);
}

// SPI3: TX
void DMA1_Stream7_IRQHandler(void) {
    spi_dma_err_t ret = isr_helper(DMA1_Stream7, &DMA1->HIFCR, &DMA1->HISR, DMA_HISR_TCIF7, DMA_HISR_TEIF7, DMA_HISR_DMEIF7);
    isr_tx_helper(SPI3, ret, 2);
}

// SPI3: RX
void DMA1_Stream2_IRQHandler(void) {
    spi_dma_err_t ret = isr_helper(DMA1_Stream2, &DMA1->LIFCR, &DMA1->LISR, DMA_LISR_TCIF2, DMA_LISR_TEIF2, DMA_LISR_DMEIF2);
    isr_rx_helper(SPI3, ret, 2);
}

// SPI4: TX
void DMA2_Stream1_IRQHandler(void) {
    spi_dma_err_t ret = isr_helper(DMA2_Stream1, &DMA2->LIFCR, &DMA2->LISR, DMA_LISR_TCIF1, DMA_LISR_TEIF1, DMA_LISR_DMEIF1);
    isr_tx_helper(SPI4, ret, 3);
}

// SPI4: RX
void DMA2_Stream0_IRQHandler(void) {
    spi_dma_err_t ret = isr_helper(DMA2_Stream0, &DMA2->LIFCR, &DMA2->LISR, DMA_LISR_TCIF0, DMA_LISR_TEIF0, DMA_LISR_DMEIF0);
    isr_rx_helper(SPI4, ret, 3);
}

// SPI5: TX
void DMA2_Stream4_IRQHandler(void) {
    spi_dma_err_t ret = isr_helper(DMA2_Stream4, &DMA2->HIFCR, &DMA2->HISR, DMA_HISR_TCIF4, DMA_HISR_TEIF4, DMA_HISR_DMEIF4);
    isr_tx_helper(SPI5, ret, 4);
}

// SPI5: RX
void DMA2_Stream3_IRQHandler(void) {
    spi_dma_err_t ret = isr_helper(DMA2_Stream3, &DMA2->LIFCR, &DMA2->LISR, DMA_LISR_TCIF3, DMA_LISR_TEIF3, DMA_LISR_DMEIF3);
    isr_rx_helper(SPI5, ret, 4);
}
