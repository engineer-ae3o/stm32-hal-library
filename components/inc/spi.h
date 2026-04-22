#ifndef _SPI_H_
#define _SPI_H_


#ifdef __cplusplus
extern "C" {
#endif


#include "stm32f411xe.h"
#include "stddef.h"
#include "stdint.h"
#include "stdbool.h"
#include "common.h"


typedef struct {
    bool use_miso;
    bool use_mosi;
    uint8_t cpol_cpha;
    
    uint8_t miso;
    uint8_t mosi;
    uint8_t sclk;
    GPIO_TypeDef* gpio_port;
    
    uint32_t clk_speed_hz;
} spi_master_config_t;

hal_err_t spi_master_init(SPI_TypeDef* handle, const spi_master_config_t* config);
hal_err_t spi_master_dma_init(SPI_TypeDef* handle);

// Polling API
hal_err_t spi_master_transmit_poll(SPI_TypeDef* handle, const uint8_t* data, size_t len);
hal_err_t spi_master_receive_poll(SPI_TypeDef* handle, uint8_t* data, size_t len);
hal_err_t spi_master_transmit_receive_poll(SPI_TypeDef* handle, const uint8_t* tx_data,
                                           size_t tx_len, uint8_t* rx_data, size_t rx_len);

// DMA transfer API
// Callback for DMA transmission and reception completion
typedef void (*spi_trans_done_cb_t)(void* arg);

hal_err_t spi_master_transmit_dma(SPI_TypeDef* handle, const uint8_t* data, uint16_t len,
                                  spi_trans_done_cb_t callback, void* arg);
hal_err_t spi_master_receive_dma(SPI_TypeDef* handle, const uint8_t* data, uint16_t len,
                                 spi_trans_done_cb_t callback, void* arg);
hal_err_t spi_master_transmit_receive_dma(SPI_TypeDef* handle, const uint8_t* tx_data, size_t tx_len, uint8_t* rx_data,
                                          size_t rx_len, spi_trans_done_cb_t callback, void* arg);


#ifdef __cplusplus
}
#endif


#endif // _SPI_H_