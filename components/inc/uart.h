#ifndef _UART_H_
#define _UART_H_


#ifdef __cplusplus
extern "C" {
#endif


#include "stdint.h"
#include "stddef.h"
#include "stdbool.h"
#include "stm32f411xe.h"


typedef void (*uart_dma_transmit_done_cb_t)(void* arg);

typedef struct {
    uint16_t over_sampling;
    uint32_t baud_rate;

    GPIO_TypeDef* uart_gpio_chan;
    uint32_t tx_pin;
    uint32_t rx_pin;

    bool with_dma;
} uart_config_t;

void uart_init(USART_TypeDef* handle, const uart_config_t* config);
void uart_transmit_byte(USART_TypeDef* handle, uint8_t byte);
void uart_transmit_poll(USART_TypeDef* handle, const uint8_t* data, size_t len);
void uart_transmit_dma(USART_TypeDef* handle, const uint8_t* data, size_t len,
                       uart_dma_transmit_done_cb_t cb, void* arg);
void uart_receive_dma(USART_TypeDef* handle, uint8_t* data, size_t len,
                       uart_dma_transmit_done_cb_t cb, void* arg);


#ifdef __cplusplus
}
#endif


#endif // _UART_H_