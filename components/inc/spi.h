#ifndef _SPI_H_
#define _SPI_H_


#ifdef __cplusplus
extern "C" {
#endif


#include "stm32f411xe.h"
#include "stddef.h"
#include "stdint.h"
#include "stdbool.h"


typedef struct {
    uint8_t cpol_cpha;
    
    bool use_miso;
    uint32_t miso;
    uint32_t mosi;
    uint32_t sclk;
    GPIO_TypeDef* gpio_port;
} spi_master_config_t;

void spi_master_init(SPI_TypeDef* handle, const spi_master_config_t* config);

// Polling API
void spi_master_transmit_poll(SPI_TypeDef* handle, const uint8_t* data, size_t len);
void spi_master_receive_poll(SPI_TypeDef* handle, uint8_t* data, size_t len);
void spi_master_transmit_receive_poll(SPI_TypeDef* handle, const uint8_t* tx_data,
                                 size_t tx_len, uint8_t* rx_data, size_t rx_len);

// DMA transfer API
void spi_master_transmit_dma(SPI_TypeDef* handle, const uint8_t* data, size_t len);
void spi_master_receive_dma(SPI_TypeDef* handle, uint8_t* data, size_t len);
void spi_master_transmit_receive_dma(SPI_TypeDef* handle, const uint8_t* tx_data,
                                     size_t tx_len, uint8_t* rx_data, size_t rx_len);



#ifdef __cplusplus
}
#endif


#endif // _SPI_H_