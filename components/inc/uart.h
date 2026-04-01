#ifndef _UART_H_
#define _UART_H_


#ifdef __cplusplus
extern "C" {
#endif


#include "stdint.h"
#include "stddef.h"
#include "stdbool.h"
#include "stm32f411xe.h"


// Callback for DMA transmission and reception
typedef void (*uart_dma_transmit_done_cb_t)(void* arg);

// Better type name
typedef USART_TypeDef* USART_TypeDef_t;

typedef struct {
    uint16_t over_sampling;
    uint32_t clock_freq_hz;
    uint32_t baud_rate;

    uint32_t tx_pin;
    uint32_t rx_pin;
    GPIO_TypeDef* uart_gpio_chan;

} uart_config_t;

void uart_init(USART_TypeDef_t handle, const uart_config_t* config);
void uart_dma_init(USART_TypeDef_t handle);
void uart_dma_deinit(USART_TypeDef_t handle);
void uart_enable(USART_TypeDef_t handle);
void uart_disable(USART_TypeDef_t handle);
void uart_transmit_byte(USART_TypeDef_t handle, uint8_t byte);
void uart_transmit_poll(USART_TypeDef_t handle, const uint8_t* data, size_t len);
void uart_transmit_dma(USART_TypeDef_t handle, const uint8_t* data, size_t len,
                       uart_dma_transmit_done_cb_t callback, void* arg);
void uart_receive_dma(USART_TypeDef_t handle, uint8_t* data, size_t len,
                      uart_dma_transmit_done_cb_t callback, void* arg);


#ifdef __cplusplus
}
#endif


#endif // _UART_H_