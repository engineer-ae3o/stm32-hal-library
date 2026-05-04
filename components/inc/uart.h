#ifndef _UART_H_
#define _UART_H_


#ifdef __cplusplus
extern "C" {
#endif


#include "stm32f411xe.h"
#include "common.h"

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>


typedef struct {
    uint8_t tx_pin;
    uint8_t rx_pin;
    GPIO_TypeDef* gpio_port;

    uint16_t over_sampling;
    uint32_t clock_freq_hz;
    uint32_t baud_rate;
} uart_config_t;

hal_err_t uart_init(USART_TypeDef* handle, const uart_config_t* config);
hal_err_t uart_dma_init(USART_TypeDef* handle);

void uart_enable(USART_TypeDef* handle);
void uart_disable(USART_TypeDef* handle);

// Polling API
void uart_transmit_byte(USART_TypeDef* handle, uint8_t byte);
void uart_transmit_poll(USART_TypeDef* handle, const uint8_t* data, size_t len);

// DMA transfer API
hal_err_t uart_transmit_dma(USART_TypeDef* handle, const uint8_t* data, uint16_t len,
                            dma_trans_done_cb_t callback, void* arg);
hal_err_t uart_receive_dma(USART_TypeDef* handle, uint8_t* data, uint16_t len,
                           dma_trans_done_cb_t callback, void* arg);


#ifdef __cplusplus
}
#endif


#endif // _UART_H_