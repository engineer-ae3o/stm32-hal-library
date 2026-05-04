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

#define I2S1 ((I2S_TypeDef*)SPI1)
#define I2S2 ((I2S_TypeDef*)SPI2)
#define I2S3 ((I2S_TypeDef*)SPI3)
#define I2S4 ((I2S_TypeDef*)SPI4)
#define I2S5 ((I2S_TypeDef*)SPI5)


typedef enum : uint8_t {
    I2S_MODE_PHILLIPS        = 0b00U,
    I2S_MODE_LEFT_JUSTIFIED  = 0b01U,
    I2S_MODE_RIGHT_JUSTIFIED = 0b10U
} i2s_mode_t;

typedef enum : uint8_t {
    I2S_DATA_16_BITS_FRAME_16_BITS = 0b00U,
    I2S_DATA_16_BITS_FRAME_32_BITS = 0b00U,
    I2S_DATA_24_BITS_FRAME_32_BITS = 0b01U,
    I2S_DATA_32_BITS_FRAME_32_BITS = 0b10U
} i2s_data_frame_t;

typedef enum : uint8_t {
    I2S_DIR_HALF_DUPLEX_TX = 0b10U,
    I2S_DIR_HALF_DUPLEX_RX = 0b11U
} i2s_dir_t;

typedef enum : uint32_t {
    I2S_FREQ_8kHz   = 8'000UL,
    I2S_FREQ_16kHz  = 16'000UL,
    I2S_FREQ_22kHz  = 22'050UL,
    I2S_FREQ_32kHz  = 32'000UL,
    I2S_FREQ_44kHz  = 44'100UL,
    I2S_FREQ_48kHz  = 48'000UL,
    I2S_FREQ_96kHz  = 96'000UL,
    I2S_FREQ_192kHz = 192'000UL
} i2s_freq_t;

typedef struct {
    i2s_dir_t dir;
    i2s_mode_t mode;
    i2s_data_frame_t data_frame;
    i2s_freq_t freq;

    bool cpol;
    bool use_mck;
    uint8_t mck;

    uint8_t ws;
    uint8_t sd;
    uint8_t sck;
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
// NOTE: Only reception is supported
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