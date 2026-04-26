#include "stm32f411xe.h"
#include "i2s.h"
#include "spi.h"


// The 5 SPI instances: ISRs called when the DMA is done
// TX
spi_trans_done_cb_t s_spi_tx_done_cbs[5] = {};
void* s_tx_args[5] = {};
// RX
spi_trans_done_cb_t s_spi_rx_done_cbs[5] = {};
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


// Mapping for the DMA channels for the 3 SPI channels
__attribute__((unused))
static const dma_stream_map_t s_spi_dma_map[5] = {
    // SPI1
    {
        .tx = { .controller = DMA2, .stream = DMA2_Stream7, .channel = 4 },
        .rx = { .controller = DMA2, .stream = DMA2_Stream5, .channel = 4 }
    },
    // SPI2
    {
        .tx = { .controller = DMA1, .stream = DMA1_Stream6, .channel = 4 },
        .rx = { .controller = DMA1, .stream = DMA1_Stream5, .channel = 4 }
    },
    // SPI3
    {
        .tx = { .controller = DMA2, .stream = DMA2_Stream6, .channel = 5 },
        .rx = { .controller = DMA2, .stream = DMA2_Stream1, .channel = 5 }
    },
    // SPI4
    {
        .tx = { .controller = DMA2, .stream = DMA2_Stream4, .channel = 5 },
        .rx = { .controller = DMA2, .stream = DMA2_Stream2, .channel = 5 }
    },
    // SPI5
    {
        .tx = { .controller = DMA2, .stream = DMA2_Stream6, .channel = 5 },
        .rx = { .controller = DMA2, .stream = DMA2_Stream1, .channel = 5 }
    }
};

// Helper
static inline uint8_t get_index(const SPI_TypeDef* handle) {
    if      (handle == SPI1) return 0U;
    else if (handle == SPI2) return 1U;
    else if (handle == SPI3) return 2U;
    else if (handle == SPI4) return 3U;
    else if (handle == SPI5) return 4U;
    else                     return 0xFFU;
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

    // Disable SPI before modifying the registers
    handle->CR1 &= ~SPI_CR1_SPE;
    
    (void)config;
    
    // Enable the SPI peripheral
    handle->CR1 |= SPI_CR1_SPE;

    return HAL_OK;
}

hal_err_t spi_master_dma_init(SPI_TypeDef* handle) {

    (void)handle;

    return HAL_OK;
}

// Polling API
hal_err_t spi_master_transmit_poll(SPI_TypeDef* handle, const uint8_t* data, size_t len) {

    (void)handle;
    (void)data;
    (void)len;

    return HAL_OK;
}

hal_err_t spi_master_receive_poll(SPI_TypeDef* handle, uint8_t* data, size_t len) {

    (void)handle;
    (void)data;
    (void)len;

    return HAL_OK;
}

hal_err_t spi_master_transmit_receive_poll(SPI_TypeDef* handle, const uint8_t* tx_data,
                                           size_t tx_len, uint8_t* rx_data, size_t rx_len) {

    (void)handle;
    (void)tx_data;
    (void)tx_len;
    (void)rx_data;
    (void)rx_len;

    return HAL_OK;
}

// DMA transfers API
hal_err_t spi_master_transmit_dma(SPI_TypeDef* handle, const uint8_t* data, uint16_t len,
                                  spi_trans_done_cb_t callback, void* arg) {

    (void)handle;
    (void)data;
    (void)len;
    (void)callback;
    (void)arg;

    return HAL_OK;
}

hal_err_t spi_master_receive_dma(SPI_TypeDef* handle, const uint8_t* data, uint16_t len,
                                 spi_trans_done_cb_t callback, void* arg) {

    (void)handle;
    (void)data;
    (void)len;
    (void)callback;
    (void)arg;

    return HAL_OK;
}

hal_err_t spi_master_transmit_receive_dma(SPI_TypeDef* handle, const uint8_t* tx_data, size_t tx_len, uint8_t* rx_data,
                                          size_t rx_len, spi_trans_done_cb_t callback, void* arg) {

    (void)handle;
    (void)tx_data;
    (void)tx_len;
    (void)rx_data;
    (void)rx_len;
    (void)callback;
    (void)arg;

    return HAL_OK;
}
