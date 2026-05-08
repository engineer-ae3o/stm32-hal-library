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


// Typedefing to make it clearer that we are working with I2S not SPI
typedef SPI_TypeDef I2S_TypeDef;

#define I2S1 ((I2S_TypeDef*)SPI1)
#define I2S2 ((I2S_TypeDef*)SPI2)
#define I2S3 ((I2S_TypeDef*)SPI3)
#define I2S4 ((I2S_TypeDef*)SPI4)
#define I2S5 ((I2S_TypeDef*)SPI5)


typedef enum : uint8_t {
    I2S_MODE_PHILIPS         = 0b00U,
    I2S_MODE_LEFT_JUSTIFIED  = 0b01U,
    I2S_MODE_RIGHT_JUSTIFIED = 0b10U
} i2s_mode_t;

typedef enum : uint8_t {
    I2S_DATA_16_BITS_FRAME_16_BITS = 0b00U,
    I2S_DATA_16_BITS_FRAME_32_BITS = 0b11U,
    I2S_DATA_24_BITS_FRAME_32_BITS = 0b01U,
    I2S_DATA_32_BITS_FRAME_32_BITS = 0b10U
} i2s_data_frame_t;

// Full duplex not supported
typedef enum : uint8_t {
    I2S_DIR_HALF_DUPLEX_TX = 0b10U,
    I2S_DIR_HALF_DUPLEX_RX = 0b11U
} i2s_dir_t;

typedef enum : uint8_t {
    I2S_FREQ_8kHz = 0U,
    I2S_FREQ_16kHz,
    I2S_FREQ_22kHz, // 22.05kHz
    I2S_FREQ_32kHz,
    I2S_FREQ_44kHz, // 44.1kHz
    I2S_FREQ_48kHz,
    I2S_FREQ_96kHz,
    I2S_FREQ_192kHz
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

void i2s_pll_init(void);
hal_err_t i2sx_clk_enable(I2S_TypeDef* handle, bool enable);

hal_err_t i2s_master_init(I2S_TypeDef* handle, const i2s_master_config_t* config);
void i2s_master_enable(I2S_TypeDef* handle, bool enable);
hal_err_t i2s_master_dma_init(I2S_TypeDef* handle);
hal_err_t i2s_master_get_dma_stream(I2S_TypeDef* handle, DMA_Stream_TypeDef** tx, DMA_Stream_TypeDef** rx);

// DMA backed transfers API
hal_err_t i2s_master_transmit(I2S_TypeDef* handle, const void* buf, uint16_t len,
                              dma_trans_done_cb_t callback, void* arg);
hal_err_t i2s_master_receive(I2S_TypeDef* handle, void* buf, uint16_t len,
                             dma_trans_done_cb_t callback, void* arg);

// Double buffering API
// @note These APIs are mutually exclusive with the DMA oneshot functions
// @note Only data reception is supported. User should determine
// which buffer is free with `i2s_master_dbm_get_filled_buffer()`
hal_err_t i2s_master_dbm_init(I2S_TypeDef* handle, void* buf_a, void* buf_b,
                              uint16_t len, dma_trans_done_cb_t callback, void* arg);
hal_err_t i2s_master_dbm_deinit(I2S_TypeDef* handle);

// When start is called, the DMA starts filling buffer A, and
// then the isr is fired on completion, the starts filling B
// @note Calling stop can cause the peripheral to drop samples
hal_err_t i2s_master_dbm_start(I2S_TypeDef* handle);
hal_err_t i2s_master_dbm_stop(I2S_TypeDef* handle);

// @return `0x0` if buf_a is filled and the DMA controller has started filling buf_b,
// `0x1` if buf_b is filled and buf_a is in use, and `0xFF` if an invalid arg is passed
uint32_t i2s_master_dbm_get_filled_buffer(I2S_TypeDef* handle);


#ifdef __cplusplus
}
#endif


#endif // _I2S_H_