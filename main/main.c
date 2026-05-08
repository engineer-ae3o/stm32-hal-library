#include "stm32f411xe.h"

#include "i2c.h"
#include "i2s.h"
#include "spi.h"
#include "iwdg.h"
#include "uart.h"
#include "timer.h"
#include "printf.h"

#include <stdint.h>


uint32_t SystemCoreClock;

#define LOGGING_UART_PORT USART1

void putchar_(char c) {
    uart_transmit_byte(LOGGING_UART_PORT, (uint8_t)c);
}

int main(void) {

    __disable_irq();

    // UART
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
    
    // SPI
    const spi_master_config_t spi_config = {

    };
    spi_master_init(SPI2, &spi_config);

    // I2C
    const i2c_master_config_t i2c_config = {

    };
    i2c_master_init(I2C2, &i2c_config);
    
    // I2S
    const i2s_master_config_t i2s_config = {

    };
    i2s_master_init(I2S3, &i2s_config);
    
    // IWDG
    const uint32_t reload_time_s = 2UL;
    iwdg_start(reload_time_s);
    
    __enable_irq();
    
    while (1) {
        iwdg_kick();
    }
}
