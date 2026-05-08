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


typedef struct {
    bool cpol;
    bool use_mck;
    uint8_t mck;

    uint8_t ws;
    uint8_t sd;
    uint8_t sck;
    GPIO_TypeDef* gpio_port;
} adc_config_t;

hal_err_t adc_clk_enable(bool enable);

hal_err_t adc_init(const adc_config_t* config);
hal_err_t adc_dma_init(void);
DMA_Stream_TypeDef* adc_get_dma_stream(void);

// Polling oneshot function
uint16_t adc_get_sample_oneshot(void);

// DMA backed oneshot API
hal_err_t adc_get_sample_continuous(void* buf, uint16_t len, dma_trans_done_cb_t callback, void* arg);

// Double buffering API
// @note These APIs are mutually exclusive with the DMA oneshot function
// User should determine which buffer is free with `adc_dbm_get_filled_buffer()`
hal_err_t adc_dbm_init(void* buf_a, void* buf_b, uint16_t len, dma_trans_done_cb_t callback, void* arg);
hal_err_t adc_dbm_deinit(void);

// When start is called, the DMA starts filling buffer A, and
// then the isr is fired on completion, the starts filling B
// @note Calling stop can cause the peripheral to drop samples
hal_err_t adc_dbm_start(void);
hal_err_t adc_dbm_stop(void);

// @return `0x0` if buf_a is filled and the DMA controller has started filling buf_b,
// `0x1` if buf_b is filled and buf_a is in use, and `0xFF` if an invalid arg is passed
uint32_t adc_dbm_get_filled_buffer(void);


#ifdef __cplusplus
}
#endif


#endif // _I2S_H_