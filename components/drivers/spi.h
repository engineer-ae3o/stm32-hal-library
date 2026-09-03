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
void      spi_master_enable(SPI_TypeDef* handle, bool enable);
hal_err_t spi_master_dma_init(SPI_TypeDef* handle);

// Polling API
hal_err_t spi_master_transmit_poll(SPI_TypeDef* handle, const void* data, size_t size);
hal_err_t spi_master_receive_poll(SPI_TypeDef* handle, void* data, size_t size);
hal_err_t spi_master_transceive_poll(SPI_TypeDef* handle, const void* tx_data, void* rx_data, size_t size);

// DMA transfer API
hal_err_t spi_master_transmit_dma(SPI_TypeDef* handle, const void* data, uint16_t size, dma_trans_done_cb_t cb, void* arg);
hal_err_t spi_master_receive_dma(SPI_TypeDef* handle, void* data, uint16_t size, dma_trans_done_cb_t cb, void* arg);
hal_err_t spi_master_transceive_dma(SPI_TypeDef* handle, const void* tx_data, void* rx_data, uint16_t size, dma_trans_done_cb_t cb, void* arg);

// To be used by i2s.c. Not to be called directly from user application
void spi_master_register_callback(dma_trans_done_cb_t cb, void* arg, uint8_t idx, bool tx);


#ifdef __cplusplus
}
#endif


#endif // SPI_H_