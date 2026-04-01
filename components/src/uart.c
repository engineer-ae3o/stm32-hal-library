#include "stm32f411xe.h"
#include "uart.h"

#include <math.h>


// The 3 UART channels: ISRs called when the DMA is done
uart_dma_transmit_done_cb_t s_uart_done_cbs[3] = {};
void* s_args[3] = {};

// Utilities for mapping the USART channels to DMA streams
typedef struct {
    DMA_TypeDef* controller;
    DMA_Stream_TypeDef* stream;
    uint8_t channel;
} uart_tx_rx_t;

typedef struct {
    const uart_tx_rx_t tx;
    const uart_tx_rx_t rx;
} dma_stream_map_t;

// Mapping for the DMA channels for the 3 USART channels
static const dma_stream_map_t s_uart_dma_map[3] = {
    // USART1
    {
        .tx = { .controller = DMA2, .stream = DMA2_Stream7, .channel = 4 },
        .rx = { .controller = DMA2, .stream = DMA2_Stream5, .channel = 4 }
    },
    // USART2
    {
        .tx = { .controller = DMA1, .stream = DMA1_Stream6, .channel = 4 },
        .rx = { .controller = DMA1, .stream = DMA1_Stream5, .channel = 4 }
    },
    // USART6
    {
        .tx = { .controller = DMA2, .stream = DMA2_Stream6, .channel = 5 },
        .rx = { .controller = DMA2, .stream = DMA2_Stream1, .channel = 5 }
    }
};

// Forward declarations
static uint8_t get_index(const USART_TypeDef_t handle);


// Public API
void uart_init(USART_TypeDef_t handle, const uart_config_t* config) {

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
    const uint16_t mantissa = (uint16_t)uart_div;
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
    
    // Enable the UART peripheral
    handle->CR1 |= USART_CR1_UE;
}

void uart_dma_init(USART_TypeDef_t handle) {
    // Index for DMA mapping
    uint8_t idx = 0;

    // Enable DMA clock
    if (handle == USART1) {
        RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
        idx = 0;
    } else if (handle == USART2) {
        RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
        idx = 1;
    } else if (handle == USART6) {
        RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
        idx = 2;
    } else {
        while (1);
    }
    
    // TX mapping
    DMA_TypeDef* tx_controller = s_uart_dma_map[idx].tx.controller;
    DMA_Stream_TypeDef* tx_stream = s_uart_dma_map[idx].tx.stream;
    uint8_t tx_channel = s_uart_dma_map[idx].tx.channel;

    // RX mapping
    DMA_TypeDef* rx_controller = s_uart_dma_map[idx].rx.controller;
    DMA_Stream_TypeDef* rx_stream = s_uart_dma_map[idx].rx.stream;
    uint8_t rx_channel = s_uart_dma_map[idx].rx.channel;

    // Disable DMA streams for tx and rx
    tx_stream->CR &= ~DMA_SxCR_EN;
    while (tx_stream->CR & DMA_SxCR_EN);

    rx_stream->CR &= ~DMA_SxCR_EN;
    while (rx_stream->CR & DMA_SxCR_EN);

    // Clear global DMA interrupt flags
    tx_controller->LIFCR = 0xFFFFFFFFUL;
    tx_controller->HIFCR = 0xFFFFFFFFUL;
    rx_controller->LIFCR = 0xFFFFFFFFUL;
    rx_controller->HIFCR = 0xFFFFFFFFUL;
    
    // Configuration
    tx_stream->CR |= ((tx_channel & 0b111U) << DMA_SxCR_CHSEL_Pos) |
                     (0b11U << DMA_SxCR_PL_Pos)                    |
                     (0U << );

    // PSIZE and MSIZE
    tx_stream->CR &= ~();
    
    // Enable dma for transmitter and receiver
    handle->CR3 |= (USART_CR3_DMAT | USART_CR3_DMAR);
}

void uart_enable(USART_TypeDef_t handle) {
    // Enable UART TX
    handle->CR1 |= USART_CR1_TE;

    // Enable UART RX
    handle->CR1 |= USART_CR1_RE;
}

void uart_disable(USART_TypeDef_t handle) {
    // Disable UART TX
    handle->CR1 &= ~USART_CR1_TE;

    // Disable UART RX
    handle->CR1 &= ~USART_CR1_RE;
}

void uart_transmit_byte(USART_TypeDef_t handle, uint8_t byte) {
    // Put byte in data register
    handle->DR = byte;

    // Poll till byte has been put into the USART's FIFO
    while (!(handle->SR & USART_SR_TXE));
}

void uart_transmit_poll(USART_TypeDef_t handle, const uint8_t* data, size_t len) {
    for (size_t i = 0U; i < len; i++) {
        uart_transmit_byte(handle, data[i]);
    }
}

void uart_transmit_dma(USART_TypeDef_t handle, const uint8_t* data, size_t len,
                       uart_dma_transmit_done_cb_t callback, void* arg) {
    // Map UART handle to index for DMA mapping
    const uint8_t idx = get_index(handle);
    
    // Save user passed callback
    if (callback) {
        s_uart_done_cbs[idx] = callback;
        s_args[idx] = arg;
    }
    
}

void uart_receive_dma(USART_TypeDef_t handle, uint8_t* data, size_t len,
                      uart_dma_transmit_done_cb_t callback, void* arg) {
    (void)handle;
    (void)data;
    (void)len;
    (void)callback;
    (void)arg;
}

// Static helpers
static uint8_t get_index(const USART_TypeDef_t handle) {
    if (handle == USART1)      return 0U;
    else if (handle == USART2) return 1U;
    else if (handle == USART6) return 2U;
    else while (1);
}
