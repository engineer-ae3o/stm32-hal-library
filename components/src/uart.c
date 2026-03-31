#include "stm32f411xe.h"
#include "uart.h"

#include <math.h>


// For the 3 UART channels
uart_dma_transmit_done_cb_t s_uart_done_cb[3] = {};

void uart_init(USART_TypeDef* handle, const uart_config_t* config) {

    // Enable appropriate UART channel clock
    if (handle == USART1) {
        RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    } else if (handle == USART2) {
        RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    } else if (handle == USART6) {
        RCC->APB2ENR |= RCC_APB2ENR_USART6EN;
    } else {
        while (1);
    }

    // 9 bit UART, parity bit enabled and odd parity
    handle->CR1 |= (USART_CR1_M | USART_CR1_PCE | USART_CR1_PS);

    // Baud rate generator
    const float uart_div = (float)config->clock_freq_hz / (float)(config->baud_rate * config->over_sampling);

    // Get the mantissa and the fractional parts of the uart clock divider
    const uint16_t mantissa = (uint16_t)floorf(uart_div);
    const uint16_t fraction = (uint16_t)((uart_div - (float)mantissa) * config->over_sampling);

    // Set oversampling and baud rate divider
    // The fractional part can only be 3 bits if oversampling is 8
    if (config->over_sampling == 8) {
        handle->CR1 |= USART_CR1_OVER8;
        handle->BRR = (mantissa << USART_BRR_DIV_Mantissa_Pos) | (fraction & 0x07U);
    } else if (config->over_sampling == 16) {
        handle->CR1 &= ~USART_CR1_OVER8;
        handle->BRR = (mantissa << USART_BRR_DIV_Mantissa_Pos) | (fraction & 0x0FU);
    } else {
        while (1);
    }

    // Initialize GPIO pins for UART
    // Enable gpio channel clock
    if (config->uart_gpio_chan == GPIOA) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    } else if (config->uart_gpio_chan == GPIOB) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    } else if (config->uart_gpio_chan == GPIOC) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    } else if (config->uart_gpio_chan == GPIOD) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    } else {
        while (1);
    }

    // Set gpio pin to alternate function
    // TX pin
    config->uart_gpio_chan->MODER &= ~(0b11UL << (config->tx_pin * 2UL));
    config->uart_gpio_chan->MODER |= (0b10UL << (config->tx_pin * 2UL));
    // RX pin
    config->uart_gpio_chan->MODER &= ~(0b11UL << (config->rx_pin * 2UL));
    config->uart_gpio_chan->MODER |= (0b10UL << (config->rx_pin * 2UL));

    // Set pullup
    // TX pin
    config->uart_gpio_chan->PUPDR &= ~(0b11UL << (config->tx_pin * 2UL));
    config->uart_gpio_chan->PUPDR |= (0b01UL << (config->tx_pin * 2UL));
    // RX pin
    config->uart_gpio_chan->PUPDR &= ~(0b11UL << (config->rx_pin * 2UL));
    config->uart_gpio_chan->PUPDR |= (0b01UL << (config->rx_pin * 2UL));

    // Configure the alternate mode
    // Get the alternate function value as it
    // varies for each peripheral instance
    uint32_t alt_val = 0;
    if (handle == USART1 || handle == USART2) {
        alt_val = 7UL;
    } else if (handle == USART6) {
        alt_val = 8UL;
    } else {
        while (1);
    }

    // Set the alternate function
    // TX pin
    if (config->tx_pin <= 7) {
        config->uart_gpio_chan->AFR[0] &= ~(0b1111UL << (config->tx_pin * 4UL));
        config->uart_gpio_chan->AFR[0] |= (alt_val << (config->tx_pin * 4UL));
    } else if (config->tx_pin <= 15) {
        config->uart_gpio_chan->AFR[1] &= ~(0b1111UL << ((config->tx_pin - 8) * 4UL));
        config->uart_gpio_chan->AFR[1] |= (alt_val << ((config->tx_pin - 8) * 4UL));
    } else {
        while (1);
    }
    // RX pin
    if (config->rx_pin <= 7) {
        config->uart_gpio_chan->AFR[0] &= ~(0b1111UL << (config->rx_pin * 4UL));
        config->uart_gpio_chan->AFR[0] |= (alt_val << (config->rx_pin * 4UL));
    } else if (config->rx_pin <= 15) {
        config->uart_gpio_chan->AFR[1] &= ~(0b1111UL << ((config->rx_pin - 8) * 4UL));
        config->uart_gpio_chan->AFR[1] |= (alt_val << ((config->rx_pin - 8) * 4UL));
    } else {
        while (1);
    }
    
    if (config->with_dma) {
        // Enable dma for transmitter and receiver
        handle->CR3 |= (USART_CR3_DMAT | USART_CR3_DMAR);
    }

    // Enable UART
    handle->CR1 |= USART_CR1_UE;
}

void uart_enable(USART_TypeDef* handle) {
    // Enable UART TX
    handle->CR1 |= USART_CR1_TE;

    // Enable UART RX
    handle->CR1 |= USART_CR1_RE;
}

void uart_disable(USART_TypeDef* handle) {
    // Disable UART TX
    handle->CR1 &= ~USART_CR1_TE;

    // Disable UART RX
    handle->CR1 &= ~USART_CR1_RE;
}

void uart_transmit_byte(USART_TypeDef* handle, uint8_t byte) {
    // Put byte in data register
    handle->DR = byte;

    // Wait till byte is fully transmitted
    while (!(handle->SR & USART_SR_TC));
}

void uart_transmit_poll(USART_TypeDef* handle, const uint8_t* data, size_t len) {

    // Put byte in data register
    handle->DR = data[0];
    
    for (size_t i = 1U; i < (len - 1U); i++) {
        // Block until the byte has been sent to the shift register
        while (!(handle->SR & USART_SR_TXE));

        // Put byte in data register
        handle->DR = data[i];
    }

    // Wait for all bytes to be fully transmitted
    while (!(handle->SR & USART_SR_TC));
}

void uart_transmit_dma(USART_TypeDef* handle, const uint8_t* data, size_t len, uart_dma_transmit_done_cb_t cb, void* arg) {
    (void)handle;
    (void)data;
    (void)len;
    (void)cb;
    (void)arg;
}

void uart_receive_dma(USART_TypeDef* handle, uint8_t* data, size_t len, uart_dma_transmit_done_cb_t cb, void* arg) {
    (void)handle;
    (void)data;
    (void)len;
    (void)cb;
    (void)arg;
}
