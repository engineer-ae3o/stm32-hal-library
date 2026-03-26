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
    const float uart_div = (float)SystemCoreClock / (float)(config->baud_rate * config->over_sampling);
    const uint16_t mantissa = (uint16_t)floorf(uart_div);
    const uint16_t fraction = (uint16_t)((uart_div - (float)mantissa) * config->over_sampling);
    handle->BRR = (mantissa << USART_BRR_DIV_Mantissa_Pos) | (fraction & 0x0FU);

    // Set oversampling and baud rate divider
    if (config->over_sampling == 8) {
        handle->CR1 |= USART_CR1_OVER8;
        handle->BRR = (mantissa << USART_BRR_DIV_Mantissa_Pos) | (fraction & 0x07U);
    } else if (config->over_sampling == 16) {
        handle->CR1 &= ~USART_CR1_OVER8;
        handle->BRR = (mantissa << USART_BRR_DIV_Mantissa_Pos) | (fraction & 0x0FU);
    } else {
        while (1);
    }

    // Initialize TX pin
    // Enable TX gpio channel clock
    if (config->tx_chan == GPIOA) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    } else if (config->tx_chan == GPIOB) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    } else if (config->tx_chan == GPIOC) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    } else if (config->tx_chan == GPIOD) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    } else {
        while (1);
    }

    // Set gpio pin to alternate function
    config->tx_chan->MODER &= ~(0b11UL << (config->tx_pin * 2UL));
    config->tx_chan->MODER |= (0b10UL << (config->tx_pin * 2UL));

    // Set pullup
    config->tx_chan->PUPDR &= ~(0b11UL << (config->tx_pin * 2UL));
    config->tx_chan->PUPDR |= (0b01UL << (config->tx_pin * 2UL));

    // Configure the alternate mode
    if (config->tx_pin <= 7) {
        config->tx_chan->AFR[0] &= ~(0b1111UL << (config->tx_pin * 4UL));
        config->tx_chan->AFR[0] |= (0b1111UL << (config->tx_pin * 4UL));
    } else if (config->tx_pin <= 15) {
        config->tx_chan->AFR[1] &= ~(0b1111UL << (config->tx_pin * 4UL));
        config->tx_chan->AFR[1] |= (0b1111UL << (config->tx_pin * 4UL));
    } else {
        while (1);
    }
    
    // Initialize RX pin
    // Enable RX gpio channel clock
    if (config->rx_chan == GPIOA) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    } else if (config->rx_chan == GPIOB) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    } else if (config->rx_chan == GPIOC) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    } else if (config->rx_chan == GPIOD) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    } else {
        while (1);
    }

    // 


    if (config->with_dma) {
        
    }

    // Enable UART
    handle->CR1 |= USART_CR1_UE;
}

void uart_transmit_byte(USART_TypeDef* handle, uint8_t byte) {
    // Enable UART TX
    handle->CR1 |= USART_CR1_TE;
    while (!(handle->CR1 & USART_CR1_TE));

    // Put byte in data register
    handle->DR = byte;

    // Wait till byte is transmitted
    while (!(handle->SR & USART_SR_TXE));

    // Disable UART TX
    handle->CR1 &= ~USART_CR1_TE;
    while (handle->CR1 & USART_CR1_TE);
}

void uart_transmit_poll(USART_TypeDef* handle, const uint8_t* data, size_t len) {
    // Enable UART TX
    handle->CR1 |= USART_CR1_TE;
    while (!(handle->CR1 & USART_CR1_TE));

    for (size_t i = 0; i < len; i++) {
        // Put byte in data register
        handle->DR = data[i];
        // Wait till byte is transmitted
        while (!(handle->SR & USART_SR_TXE));
    }

    // Disable UART TX
    handle->CR1 &= ~USART_CR1_TE;
    while (handle->CR1 & USART_CR1_TE);
}

void uart_transmit_dma(USART_TypeDef* handle, const uint8_t* data, size_t len, uart_dma_transmit_done_cb_t cb, void* arg) {
    (void)handle;
    (void)data;
    (void)len;
    (void)cb;
    (void)arg;
}

void uart_receive_dma(USART_TypeDef* handle, uint8_t* data, size_t len, uart_dma_transmit_done_cb_t cb, void* arg) {
    // Enable UART RX
    handle->CR1 |= USART_CR1_RE;
    while (!(handle->CR1 & USART_CR1_RE));
    
    (void)data;
    (void)len;
    (void)cb;
    (void)arg;

    // Disable UART RX
    handle->CR1 &= ~USART_CR1_RE;
    while (handle->CR1 & USART_CR1_RE);
}
