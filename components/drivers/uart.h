#ifndef UART_H_
#define UART_H_


#ifdef __cplusplus
extern "C" {
#endif


#include "stm32f411xe.h"
#include "drivers/dma_types.h"
#include "utils/err.h"

#include <stdint.h>
#include <stddef.h>


typedef enum : uint8_t {
    UART_OVER_SAMPLING_8  = 8,
    UART_OVER_SAMPLING_16 = 16,
} uart_over_sample_t;

typedef struct {
    uart_over_sample_t over_sampling;

    uint8_t       tx_pin;
    uint8_t       rx_pin;
    GPIO_TypeDef* gpio_port;

    uint32_t clock_freq_hz;
    uint32_t baud_rate;
} uart_config_t;

hal_err_t uartx_clk_enable(USART_TypeDef* handle, bool enable);
hal_err_t uart_init(USART_TypeDef* handle, const uart_config_t* config);
hal_err_t uart_enable(USART_TypeDef* handle, bool enable);
hal_err_t uart_dma_init(USART_TypeDef* handle, dma_priority_t priority);

// Polling TX API
hal_err_t uart_transmit_byte(USART_TypeDef* handle, uint8_t byte);
hal_err_t uart_transmit_poll(USART_TypeDef* handle, const uint8_t* data, size_t size);

// DMA transfer API
hal_err_t uart_transmit_dma(USART_TypeDef* handle, const uint8_t* data, uint16_t size, dma_done_cb_t callback, void* arg);
hal_err_t uart_receive_dma(USART_TypeDef* handle, uint8_t* data, uint16_t size, dma_done_cb_t callback, void* arg);


#ifdef __cplusplus
}
#endif


#endif // UART_H_