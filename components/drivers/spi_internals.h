#ifndef SPI_INTERNALS_H_
#define SPI_INTERNALS_H_


#ifdef __cplusplus
extern "C" {
#endif


#include "drivers/dma_types.h"
#include "utils/err.h"


#define ENABLE_SPI()                                                                                                                                 \
    do {                                                                                                                                             \
        handle->CR1 |= SPI_CR1_SPE;                                                                                                                  \
        __DSB();                                                                                                                                     \
    } while (0)

#define DISABLE_SPI()                                                                                                                                \
    do {                                                                                                                                             \
        handle->CR1 &= ~SPI_CR1_SPE;                                                                                                                 \
        __DSB();                                                                                                                                     \
    } while (0)

#define ENABLE_I2S()                                                                                                                                 \
    do {                                                                                                                                             \
        handle->I2SCFGR |= SPI_I2SCFGR_I2SE;                                                                                                         \
        __DSB();                                                                                                                                     \
    } while (0)

#define DISABLE_I2S()                                                                                                                                \
    do {                                                                                                                                             \
        handle->I2SCFGR &= ~SPI_I2SCFGR_I2SE;                                                                                                        \
        __DSB();                                                                                                                                     \
    } while (0)

#define ENABLE_SPI_DMA()                                                                                                                             \
    do {                                                                                                                                             \
        handle->CR2 |= (SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);                                                                                          \
    } while (0)

#define DISABLE_SPI_DMA()                                                                                                                            \
    do {                                                                                                                                             \
        handle->CR2 &= ~(SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);                                                                                         \
    } while (0)


hal_err_t spi_master_register_callback(dma_done_cb_t callback, void* arg, uint8_t idx, bool is_tx);
hal_err_t spi_master_get_dma_stream_map(dma_stream_map_t* map, uint32_t idx);


#ifdef __cplusplus
}
#endif


#endif // SPI_INTERNALS_H_