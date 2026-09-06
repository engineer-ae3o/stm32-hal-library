#ifndef SPI_H_
#define SPI_H_


#ifdef __cplusplus
extern "C" {
#endif


#include "stm32f411xe.h"
#include "drivers/dma.h"
#include "utils/err.h"

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>


typedef struct {
    bool cpol;
    bool cpha;
    bool use_miso;
    bool use_mosi;
    bool use_8bit_mode;

    uint8_t clk_divider;

    uint8_t miso_pin;
    uint8_t mosi_pin;
    uint8_t sclk_pin;

    GPIO_TypeDef* gpio_port;
} spi_master_config_t;

hal_err_t spix_clk_enable(SPI_TypeDef* handle, bool enable);
hal_err_t spi_master_init(SPI_TypeDef* handle, const spi_master_config_t* config);
hal_err_t spi_master_enable(SPI_TypeDef* handle, bool enable);
hal_err_t spi_master_dma_init(SPI_TypeDef* handle, dma_priority_t priority);

// Polling API
hal_err_t spi_master_transmit_poll(SPI_TypeDef* handle, const void* data, size_t size);
hal_err_t spi_master_receive_poll(SPI_TypeDef* handle, void* data, size_t size);
hal_err_t spi_master_transceive_poll(SPI_TypeDef* handle, const void* tx_data, void* rx_data, size_t size);

// DMA transfer API
hal_err_t spi_master_transmit_dma(SPI_TypeDef* handle, const void* data, uint16_t size, dma_done_cb_t cb, void* arg);
hal_err_t spi_master_receive_dma(SPI_TypeDef* handle, void* data, uint16_t size, dma_done_cb_t cb, void* arg);
hal_err_t spi_master_transceive_dma(SPI_TypeDef* handle, const void* tx_data, void* rx_data, uint16_t size, dma_done_cb_t cb, void* arg);


#ifdef __cplusplus
}
#endif


#endif // SPI_H_