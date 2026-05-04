#ifndef _SPI_H_
#define _SPI_H_


#ifdef __cplusplus
extern "C" {
#endif


#include "stm32f411xe.h"
#include "common.h"
#include "dma.h"

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>


typedef struct {
    bool use_miso;
    bool use_mosi;
    bool use_8bit_mode;
    
    bool cpol;
    bool cpha;
    uint8_t clk_divider;
    
    uint8_t miso;
    uint8_t mosi;
    uint8_t sclk;
    GPIO_TypeDef* gpio_port;
} spi_master_config_t;

hal_err_t spi_master_init(SPI_TypeDef* handle, const spi_master_config_t* config);
hal_err_t spi_master_dma_init(SPI_TypeDef* handle);

// Polling API
hal_err_t spi_master_transmit_poll(SPI_TypeDef* handle, const void* data, size_t len);
hal_err_t spi_master_receive_poll(SPI_TypeDef* handle, void* data, size_t len);
hal_err_t spi_master_transceive_poll(SPI_TypeDef* handle, const void* tx_data,
                                     void* rx_data, size_t len);

// DMA transfer API
hal_err_t spi_master_transmit_dma(SPI_TypeDef* handle, const void* data, uint16_t len,
                                  dma_trans_done_cb_t callback, void* arg);
hal_err_t spi_master_receive_dma(SPI_TypeDef* handle, void* data, uint16_t len,
                                 dma_trans_done_cb_t callback, void* arg);
hal_err_t spi_master_transceive_dma(SPI_TypeDef* handle, const void* tx_data, void* rx_data,
                                    uint16_t len, dma_trans_done_cb_t callback, void* arg);


#ifdef __cplusplus
}
#endif


#endif // _SPI_H_