#ifndef _I2S_H_
#define _I2S_H_


#ifdef __cplusplus
extern "C" {
#endif


#include "stm32f411xe.h"
#include "common.h"
#include "dma.h"

#include <stdint.h>
#include <stdbool.h>


// Typedefing to make it clearer
typedef SPI_TypeDef I2S_TypeDef;

#define I2S1 SPI1
#define I2S2 SPI2
#define I2S3 SPI3
#define I2S4 SPI4
#define I2S5 SPI5

typedef enum : uint8_t {
    I2S_MODE_PHILLIPS = 0,
    I2S_MODE_LEFT_MSB,
    I2S_MODE_RIGHT_MSB,
    I2S_MODE_PCM
} i2s_mode_t;

typedef enum : uint8_t {
    I2S_DIR_FULL_DUPLEX = 0,
    I2S_DIR_HALF_DUPLEX_TX,
    I2S_DIR_HALF_DUPLEX_RX
} i2s_dir_t;

typedef enum : uint8_t {
    I2S_FREQ_8kHz = 0,
    I2S_FREQ_16kHz,
    I2S_FREQ_20kHz,
    I2S_FREQ_44kHz,
    I2S_FREQ_48kHz,
    I2S_FREQ_96kHz,
    I2S_FREQ_192kHz,
} i2s_freq_t;

typedef struct {
    i2s_dir_t dir;
    i2s_mode_t mode;
    i2s_freq_t freq;

    bool cpol;
    bool use_mck;
    uint8_t mck;

    uint8_t ws;
    uint8_t sd;
    uint8_t sck;
    uint8_t miso;
    GPIO_TypeDef* gpio_port;
} i2s_master_config_t;

hal_err_t i2s_master_init(I2S_TypeDef* handle, const i2s_master_config_t* config);
hal_err_t i2s_master_dma_init(I2S_TypeDef* handle);

// DMA backed transfers API
hal_err_t i2s_master_dma_transmit(I2S_TypeDef* handle, const void* buf, uint16_t len,
                                  dma_trans_done_cb_t callback, void* arg);
hal_err_t i2s_master_dma_receive(I2S_TypeDef* handle, void* buf, uint16_t len,
                                 dma_trans_done_cb_t callback, void* arg);
hal_err_t i2s_master_transceive(I2S_TypeDef* handle, const void* tx_data, void* rx_data,
                                uint16_t len, dma_trans_done_cb_t callback, void* arg);

// Double buffering API
// NOTE: These API are mutually exclusive with the API above
// NOTE: Transfers and M-M are not supported
hal_err_t i2s_master_dbm_init(I2S_TypeDef* handle, void* buf_a, void* buf_b,
                              uint16_t len, dma_dbm_done_cb_t cb, void* arg);
hal_err_t i2s_master_dbm_deinit(I2S_TypeDef* handle);

// When start is called, the DMA starts filling buffer A, and
// then the isr is fired on completion, the starts filling B
// NOTE: Calling stop can cause the peripheral to drop samples
hal_err_t i2s_master_dbm_start(I2S_TypeDef* handle);
hal_err_t i2s_master_dbm_stop(I2S_TypeDef* handle);


#ifdef __cplusplus
}
#endif


#endif // _I2S_H_