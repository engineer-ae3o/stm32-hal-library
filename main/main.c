#include "stm32f411xe.h"

#include "iwdg.h"
#include "uart.h"
#include "timer.h"
#include "printf.h"

#include <stdint.h>


#define LOGGING_UART_PORT USART1

uint32_t SystemCoreClock;


void putchar_(char c) {
    uart_transmit_byte(LOGGING_UART_PORT, (uint8_t)c);
}

int main(void) {

    __disable_irq();

    const uart_config_t config = {
        .over_sampling = 16UL,
        .clock_freq_hz = SystemCoreClock,
        .baud_rate = 115200UL,

        .tx_pin = 2UL,
        .rx_pin = 3UL,
        .gpio_port = GPIOC,
    };
    uart_init(LOGGING_UART_PORT, &config);
    uart_dma_init(LOGGING_UART_PORT);

    const uint32_t reload_time_s = 2;
    iwdg_start(reload_time_s);
    
    __enable_irq();
    
    while (1) {
        iwdg_kick();
    }
}
