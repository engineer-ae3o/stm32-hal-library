#include "stm32f411xe.h"
#include "i2s.h"
#include "gpio.h"

#include <stddef.h>


// The 5 I2S instances: ISRs called when the DMA is done
// TX
dma_trans_done_cb_t s_dma_tx_done_cbs[5] = {};
void* s_tx_args[5] = {};
// RX
dma_trans_done_cb_t s_dma_rx_done_cbs[5] = {};
void* s_rx_args[5] = {};
// RX for Double Buffering mode
dma_dbm_done_cb_t s_dma_dmb_done_cb[5] = {};
void* s_dmb_args[5] = {};

// Mapping for the DMA channels for the 5 I2S channels
static const dma_stream_map_t s_i2s_dma_map[5] = {
    // I2S1
    {
        .tx = { .controller = DMA2, .stream = DMA2_Stream5, .stream_no = 5, .irq_type = DMA2_Stream5_IRQn, .channel = 3 },
        .rx = { .controller = DMA2, .stream = DMA2_Stream2, .stream_no = 2, .irq_type = DMA2_Stream2_IRQn, .channel = 3 }
    },
    // I2S2
    {
        .tx = { .controller = DMA1, .stream = DMA1_Stream4, .stream_no = 4, .irq_type = DMA1_Stream4_IRQn, .channel = 0 },
        .rx = { .controller = DMA1, .stream = DMA1_Stream3, .stream_no = 3, .irq_type = DMA1_Stream3_IRQn, .channel = 0 }
    },
    // I2S3
    {
        .tx = { .controller = DMA1, .stream = DMA1_Stream7, .stream_no = 7, .irq_type = DMA1_Stream7_IRQn, .channel = 0 },
        .rx = { .controller = DMA1, .stream = DMA1_Stream2, .stream_no = 2, .irq_type = DMA1_Stream2_IRQn, .channel = 0 }
    },
    // I2S4
    {
        .tx = { .controller = DMA2, .stream = DMA2_Stream1, .stream_no = 1, .irq_type = DMA2_Stream1_IRQn, .channel = 4 },
        .rx = { .controller = DMA2, .stream = DMA2_Stream0, .stream_no = 0, .irq_type = DMA2_Stream0_IRQn, .channel = 4 }
    },
    // I2S5
    {
        .tx = { .controller = DMA2, .stream = DMA2_Stream4, .stream_no = 4, .irq_type = DMA2_Stream4_IRQn, .channel = 2 },
        .rx = { .controller = DMA2, .stream = DMA2_Stream3, .stream_no = 3, .irq_type = DMA2_Stream3_IRQn, .channel = 2 }
    }
};

#define TIMEOUT_CYCLES 100U
#define I2S_DMA_NVIC_IRQ_PRIORITY 12U

// Helpers
static inline uint8_t get_index(const I2S_TypeDef* handle) {
    if      (handle == I2S1) return 0U;
    else if (handle == I2S2) return 1U;
    else if (handle == I2S3) return 2U;
    else if (handle == I2S4) return 3U;
    else if (handle == I2S5) return 4U;
    else                     return 0xFFU;
}


// Public API
hal_err_t i2s_master_init(I2S_TypeDef* handle, const i2s_master_config_t* config) {
    
    // Enable I2S clock
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
        return HAL_INVALID_ARG;
    }

    __DSB();
    
    // Disable the SPI and I2S peripheral before modifying its registers
    handle->CR1 &= ~SPI_CR1_SPE;
    handle->I2SCFGR &= ~SPI_I2SCFGR_I2SE;

    // I2S mode
    handle->I2SCFGR |= SPI_I2SCFGR_I2SMOD;

    // Clock setup
    // Use the internal PLLI2S clock
    RCC->PLLI2SCFGR = RCC_PLLI2SCFGR_PLLI2SM;

    // Set the clock prescaler
    handle->I2SPR = 0;
    
    // Clear used bits once in one write op
    handle->I2SCFGR &= ~(SPI_I2SCFGR_I2SCFG |
                         SPI_I2SCFGR_CKPOL  |
                         SPI_I2SCFGR_CHLEN  |
                         SPI_I2SCFGR_I2SSTD |
                         SPI_I2SCFGR_DATLEN);
    
    // Get frame size: It's can only be 16 bits when the data is 16 bits
    const uint32_t frame_size_mask = (config->data_frame == I2S_DATA_16_BITS_FRAME_16_BITS) ? 0 : SPI_I2SCFGR_CHLEN;
    const uint32_t cpol_mask = (config->cpol) ? SPI_I2SCFGR_CKPOL : 0;
    
    // Apply user settings
    handle->I2SCFGR |= (((uint32_t)config->dir << SPI_I2SCFGR_I2SCFG_Pos)        | // Direction: TX or RX in master mode
                        ((uint32_t)config->mode << SPI_I2SCFGR_I2SSTD_Pos)       | // I2S standard: Phillips, left or right justified
                        ((uint32_t)config->data_frame << SPI_I2SCFGR_DATLEN_Pos) | // Data length: 16, 24 or 32 bits
                        frame_size_mask                                          | // Frame size: 16 or 32 bits
                        cpol_mask);                                                // Clock polarity
    
    // Configure GPIO pins
    hal_err_t ret = gpiox_clk_enable(config->gpio_port);
    if (ret != HAL_OK) return ret;

    // Alternate function value selection for the GPIOs
    uint8_t alt_val = 0;
    if ((handle == SPI1) || (handle == SPI2)) alt_val = 5U;
    else if (handle == SPI3) alt_val = (config->gpio_port == GPIOD) ? 5U : 6U;
    else if (handle == SPI4) alt_val = (config->gpio_port == GPIOE) ? 5U : 6U;
    else if (handle == SPI5) alt_val = 6U;
    else return HAL_INVALID_ARG;
    
    // MCK
    if (config->use_mck) {
        // Enable the MCK output
        handle->I2SPR |= SPI_I2SPR_MCKOE;

        ret = gpio_set_alternate_function(config->gpio_port, config->mck, alt_val);
        if (ret != HAL_OK) return ret;
        gpio_enable_pullup(config->gpio_port, config->mck, true);
        gpio_set_speed_mode(config->gpio_port, config->mck, GPIO_MEDIUM_SPEED);
        gpio_set_output_type(config->gpio_port, config->mck, GPIO_PUSH_PULL);
    }

    // SD pin
    ret = gpio_set_alternate_function(config->gpio_port, config->sd, alt_val);
    if (ret != HAL_OK) return ret;
    gpio_enable_pullup(config->gpio_port, config->sd, true);
    gpio_set_speed_mode(config->gpio_port, config->sd, GPIO_MEDIUM_SPEED);
    gpio_set_output_type(config->gpio_port, config->sd, GPIO_PUSH_PULL);
    
    // WS pin
    ret = gpio_set_alternate_function(config->gpio_port, config->ws, alt_val);
    if (ret != HAL_OK) return ret;
    gpio_enable_pullup(config->gpio_port, config->ws, true);
    gpio_set_speed_mode(config->gpio_port, config->ws, GPIO_MEDIUM_SPEED);
    gpio_set_output_type(config->gpio_port, config->ws, GPIO_PUSH_PULL);
    
    // SCK pin
    ret = gpio_set_alternate_function(config->gpio_port, config->sck, alt_val);
    if (ret != HAL_OK) return ret;
    gpio_set_speed_mode(config->gpio_port, config->sck, GPIO_MEDIUM_SPEED);
    gpio_set_output_type(config->gpio_port, config->sck, GPIO_PUSH_PULL);

    // Enable the I2S peripheral
    handle->I2SCFGR |= SPI_I2SCFGR_I2SE;
    
    return HAL_OK;
}

hal_err_t i2s_master_dma_init(I2S_TypeDef* handle) {
    
    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU) return HAL_INVALID_ARG;
    
    // TX mapping
    DMA_TypeDef* tx_controller = s_i2s_dma_map[idx].tx.controller;
    DMA_Stream_TypeDef* tx_stream = s_i2s_dma_map[idx].tx.stream;
    uint8_t tx_channel = s_i2s_dma_map[idx].tx.channel;

    // RX mapping
    DMA_TypeDef* rx_controller = s_i2s_dma_map[idx].rx.controller;
    DMA_Stream_TypeDef* rx_stream = s_i2s_dma_map[idx].rx.stream;
    uint8_t rx_channel = s_i2s_dma_map[idx].rx.channel;
    
    hal_err_t ret = dmax_clk_enable(tx_controller);
    if (ret != HAL_OK) return ret;
    ret = dmax_clk_enable(rx_controller);
    if (ret != HAL_OK) return ret;
    
    // Disable DMA stream before configuring
    ret = dma_disable_stream(tx_stream);
    if (ret != HAL_OK) return ret;
    ret = dma_disable_stream(rx_stream);
    if (ret != HAL_OK) return ret;

    // TX stream configuration
    dma_clear_flags(tx_controller, s_i2s_dma_map[idx].tx.stream_no);
    dma_set_channel(tx_stream, tx_channel);
    dma_set_direct_mode(tx_stream, true);
    dma_set_direction(tx_stream, DMA_DIR_M_P);
    dma_set_flow_controller(tx_stream, true);
    dma_set_stream_priority(tx_stream, DMA_PRIORITY_LOW);
    dma_enable_circm_dbm(tx_stream, false, false);
    dma_set_increment(tx_stream, false, true);
    dma_enable_irqs(tx_stream, true, true, false, true);
    dma_set_addresses(tx_stream, &handle->DR, NULL, NULL);
    
    // RX stream configuration
    dma_clear_flags(rx_controller, s_i2s_dma_map[idx].rx.stream_no);
    dma_set_channel(rx_stream, rx_channel);
    dma_set_direct_mode(rx_stream, true);
    dma_set_direction(rx_stream, DMA_DIR_P_M);
    dma_set_flow_controller(rx_stream, true);
    dma_set_stream_priority(rx_stream, DMA_PRIORITY_LOW);
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
    NVIC_EnableIRQ(s_i2s_dma_map[idx].tx.irq_type);
    NVIC_SetPriority(s_i2s_dma_map[idx].tx.irq_type, I2S_DMA_NVIC_IRQ_PRIORITY);

    NVIC_EnableIRQ(s_i2s_dma_map[idx].rx.irq_type);
    NVIC_SetPriority(s_i2s_dma_map[idx].rx.irq_type, I2S_DMA_NVIC_IRQ_PRIORITY);

    return HAL_OK;
}

hal_err_t i2s_master_transmit(I2S_TypeDef* handle, const void* buf, uint16_t len,
                              dma_trans_done_cb_t callback, void* arg) {
    
    // Get index for DMA stream mapping
    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU) return HAL_INVALID_ARG;
    
    // Save user passed callback
    if (callback) {
        s_dma_tx_done_cbs[idx] = callback;
        s_tx_args[idx] = arg;
    }
    
    // TX mapping
    DMA_Stream_TypeDef* stream = s_i2s_dma_map[idx].tx.stream;
    
    // Set memory address and length
    dma_set_addresses(stream, NULL, buf, NULL);
    dma_set_trans_length(stream, len);
    
    // Enable DMA TX stream
    return dma_enable_stream(stream);
}

hal_err_t i2s_master_receive(I2S_TypeDef* handle, void* buf, uint16_t len,
                             dma_trans_done_cb_t callback, void* arg) {
    
    // Get index for DMA stream mapping
    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU) return HAL_INVALID_ARG;
    
    // Save user passed callback
    if (callback) {
        s_dma_rx_done_cbs[idx] = callback;
        s_rx_args[idx] = arg;
    }
    
    // RX mapping
    DMA_Stream_TypeDef* stream = s_i2s_dma_map[idx].rx.stream;
    
    // Set memory address and length
    dma_set_addresses(stream, NULL, buf, NULL);
    dma_set_trans_length(stream, len);
    
    // Enable DMA RX stream
    return dma_enable_stream(stream);
}

hal_err_t i2s_master_transceive(I2S_TypeDef* handle, const void* tx_data, void* rx_data,
                                uint16_t len, dma_trans_done_cb_t callback, void* arg) {
    
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
    DMA_Stream_TypeDef* tx_stream = s_i2s_dma_map[idx].tx.stream;
    DMA_Stream_TypeDef* rx_stream = s_i2s_dma_map[idx].rx.stream;
    
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

hal_err_t i2s_master_dbm_init(I2S_TypeDef* handle, void* buf_a, void* buf_b,
                              uint16_t len, dma_dbm_done_cb_t cb, void* arg) {

    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU) return HAL_INVALID_ARG;

    DMA_Stream_TypeDef* stream = s_i2s_dma_map[idx].rx.stream;
    
    hal_err_t ret = dma_disable_stream(stream);
    if (ret != HAL_OK) return ret;

    dma_enable_circm_dbm(stream, true, true);
    dma_set_addresses(stream, &handle->DR, buf_a, buf_b);
    dma_set_trans_length(stream, len);
    
    if (cb) {
        s_dma_dmb_done_cb[idx] = cb;
        s_dmb_args[idx] = arg;
    }
    
    return HAL_OK;
}

hal_err_t i2s_master_dbm_deinit(I2S_TypeDef* handle) {

    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU) return HAL_INVALID_ARG;

    DMA_Stream_TypeDef* stream = s_i2s_dma_map[idx].rx.stream;

    hal_err_t ret = dma_disable_stream(stream);
    if (ret != HAL_OK) return ret;

    dma_enable_circm_dbm(stream, false, false);
    dma_set_trans_length(stream, 0);

    s_dma_rx_done_cbs[idx] = NULL;
    s_rx_args[idx] = NULL;
    
    return HAL_OK;
}

hal_err_t i2s_master_dbm_start(I2S_TypeDef* handle) {
    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU) return HAL_INVALID_ARG;
    return dma_enable_stream(s_i2s_dma_map[idx].rx.stream);
}

hal_err_t i2s_master_dbm_stop(I2S_TypeDef* handle) {
    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU) return HAL_INVALID_ARG;
    return dma_disable_stream(s_i2s_dma_map[idx].rx.stream);
}
