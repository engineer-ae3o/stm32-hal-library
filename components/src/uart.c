#include "stm32f411xe.h"
#include "uart.h"


// The 3 UART channels: ISRs called when the DMA is done
// TX
uart_dma_transmit_done_cb_t s_uart_tx_done_cbs[3] = {};
void* s_tx_args[3] = {};
// RX
uart_dma_transmit_done_cb_t s_uart_rx_done_cbs[3] = {};
void* s_rx_args[3] = {};

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

// Helper
static inline uint8_t get_index(const USART_TypeDef* handle) {
    if (handle == USART1)      return 0U;
    else if (handle == USART2) return 1U;
    else if (handle == USART6) return 2U;
    else while (1);
}


// Public API
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

    // 8 bit UART and parity bit disabled
    handle->CR1 &= ~(USART_CR1_M | USART_CR1_PCE);

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

void uart_dma_init(USART_TypeDef* handle) {
    
    // DMA interrupt types
    IRQn_Type tx_dma_irq = 0;
    IRQn_Type rx_dma_irq = 0;

    // Enable DMA clock
    if (handle == USART1) {
        RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
        tx_dma_irq = DMA2_Stream7_IRQn;
        rx_dma_irq = DMA2_Stream5_IRQn;

    } else if (handle == USART2) {
        RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
        tx_dma_irq = DMA1_Stream6_IRQn;
        rx_dma_irq = DMA1_Stream5_IRQn;

    } else if (handle == USART6) {
        RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
        tx_dma_irq = DMA2_Stream6_IRQn;
        rx_dma_irq = DMA2_Stream1_IRQn;

    } else {
        while (1);
    }
    
    // Index for DMA mapping
    const uint8_t idx = get_index(handle);
    
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
    
    // TX
    // Configuration
    tx_stream->CR |= (tx_channel << DMA_SxCR_CHSEL_Pos) | // Channel select: 3 bits
                     (0b11U <<      DMA_SxCR_PL_Pos)    | // Priority select: High priority
                     (0b1U <<       DMA_SxCR_DIR_Pos)   | // Direction: M-P for TX
                     (0b1U <<       DMA_SxCR_TCIE_Pos)  | // Enable interrupt on TC
                     (0b1U <<       DMA_SxCR_TEIE_Pos)  | // Enable interrupt on TE
                     (0b1U <<       DMA_SxCR_DMEIE_Pos) | // Enable interrupt on DME
                     (0b1U <<       DMA_SxCR_MINC_Pos);   // Increment memory address

    // PSIZE and MSIZE
    tx_stream->CR &= ~((0b11U << DMA_SxCR_PSIZE_Pos)    | // Set PSIZE to 00 for 8 bit data
                       (0b11U << DMA_SxCR_MSIZE_Pos)    | // Set MSIZE to 00 for 8 bit data
                       (0b1U <<  DMA_SxCR_CIRC_Pos)     | // No circular mode
                       (0b1U <<  DMA_SxCR_PFCTRL_Pos)   | // DMA is the flow controller
                       (0b1U <<  DMA_SxCR_PINC_Pos));     // Peripheral address doesn't get incremented

    // Enable direct mode
    tx_stream->FCR &= ~DMA_SxFCR_DMDIS;

    // Set peripheral address
    tx_stream->PAR = (uint32_t)(&handle->DR);

    // RX
    // Configuration
    rx_stream->CR |= (rx_channel << DMA_SxCR_CHSEL_Pos) | // Channel select: 3 bits
                     (0b11U <<      DMA_SxCR_PL_Pos)    | // Priority select: High priority
                     (0b1U <<       DMA_SxCR_TCIE_Pos)  | // Enable interrupt on TC
                     (0b1U <<       DMA_SxCR_TEIE_Pos)  | // Enable interrupt on TE
                     (0b1U <<       DMA_SxCR_DMEIE_Pos) | // Enable interrupt on DME
                     (0b1U <<       DMA_SxCR_MINC_Pos);   // Increment memory address

    // PSIZE and MSIZE
    rx_stream->CR &= ~((0b11U << DMA_SxCR_PSIZE_Pos)    | // Set PSIZE to 00 for 8 bit data
                       (0b11U << DMA_SxCR_MSIZE_Pos)    | // Set MSIZE to 00 for 8 bit data
                       (0b11U << DMA_SxCR_DIR_Pos)      | // Direction: P-M for RX
                       (0b1U <<  DMA_SxCR_CIRC_Pos)     | // No circular mode
                       (0b1U <<  DMA_SxCR_PFCTRL_Pos)   | // DMA is the flow controller
                       (0b1U <<  DMA_SxCR_PINC_Pos));     // Peripheral address doesn't get incremented
    
    // Enable direct mode
    rx_stream->FCR &= ~DMA_SxFCR_DMDIS;

    // Set peripheral address
    rx_stream->PAR = (uint32_t)(&handle->DR);
    
    // Enable USART DMA
    handle->CR3 |= (USART_CR3_DMAT | USART_CR3_DMAR);

    // Enable DMA stream interrupts
    NVIC_SetPriority(tx_dma_irq, 10);
    NVIC_EnableIRQ(tx_dma_irq);

    NVIC_SetPriority(rx_dma_irq, 10);
    NVIC_EnableIRQ(rx_dma_irq);
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
    // Check if the data register is empty
    while (!(handle->SR & USART_SR_TXE));
    // Put byte in data register
    handle->DR = byte;
}

void uart_transmit_poll(USART_TypeDef* handle, const uint8_t* data, size_t len) {
    for (size_t i = 0U; i < len; i++) {
        uart_transmit_byte(handle, data[i]);
    }
}

void uart_transmit_dma(USART_TypeDef* handle, const uint8_t* data, uint16_t len,
                       uart_dma_transmit_done_cb_t callback, void* arg) {
    
    // Get index for DMA stream mapping
    const uint8_t idx = get_index(handle);
    
    // Save user passed callback
    if (callback) {
        s_uart_tx_done_cbs[idx] = callback;
        s_tx_args[idx] = arg;
    }
    
    // TX mapping
    DMA_Stream_TypeDef* stream = s_uart_dma_map[idx].tx.stream;
    
    // Set memory address and length
    stream->M0AR = (uint32_t)data;
    stream->NDTR = (uint32_t)len;
    
    // Enable DMA TX stream
    stream->CR |= DMA_SxCR_EN;
    while (!(stream->CR & DMA_SxCR_EN));
}

void uart_receive_dma(USART_TypeDef* handle, uint8_t* data, uint16_t len,
                      uart_dma_transmit_done_cb_t callback, void* arg) {

    // Get index for DMA stream mapping
    const uint8_t idx = get_index(handle);
    
    // Save user passed callback
    if (callback) {
        s_uart_rx_done_cbs[idx] = callback;
        s_rx_args[idx] = arg;
    }
    
    // RX mapping
    DMA_Stream_TypeDef* stream = s_uart_dma_map[idx].rx.stream;

    // Set memory address and length
    stream->M0AR = (uint32_t)data;
    stream->NDTR = (uint32_t)len;
    
    // Enable DMA RX stream
    stream->CR |= DMA_SxCR_EN;
    while (!(stream->CR & DMA_SxCR_EN));
}

// DMA interrupts
// USART1: TX
void DMA2_Stream7_IRQHandler(void) {

    if (DMA2->HISR & DMA_HISR_TCIF7) {
        // Clear interrupt flags
        DMA2->HIFCR = (DMA_HIFCR_CFEIF7 | DMA_HIFCR_CDMEIF7 |
                       DMA_HIFCR_CTEIF7 | DMA_HIFCR_CHTIF7  |
                       DMA_HIFCR_CTCIF7);

        // Poll till transmission is complete
        while (!(USART1->SR & USART_SR_TC));
        
        if (s_uart_tx_done_cbs[0]) {
            // Invoke user callback
            s_uart_tx_done_cbs[0](s_tx_args[0]);

            // Clear user passed context
            s_uart_tx_done_cbs[0] = NULL;
            s_tx_args[0] = NULL;
        }

    } else {
        while (1);
    }

    // Disable DMA TX stream
    DMA2_Stream7->CR &= ~DMA_SxCR_EN;
    while (DMA2_Stream7->CR & DMA_SxCR_EN);
}

// USART1: RX
void DMA2_Stream5_IRQHandler(void) {

    if (DMA2->HISR & DMA_HISR_TCIF5) {
        // Clear interrupt flags
        DMA2->HIFCR = (DMA_HIFCR_CFEIF5 | DMA_HIFCR_CDMEIF5 |
                       DMA_HIFCR_CTEIF5 | DMA_HIFCR_CHTIF5  |
                       DMA_HIFCR_CTCIF5);
        
        if (s_uart_rx_done_cbs[0]) {
            // Invoke user callback
            s_uart_rx_done_cbs[0](s_rx_args[0]);

            // Clear user passed context
            s_uart_rx_done_cbs[0] = NULL;
            s_rx_args[0] = NULL;
        }

    } else {
        while (1);
    }
    
    // Disable DMA RX stream
    DMA2_Stream5->CR &= ~DMA_SxCR_EN;
    while (DMA2_Stream5->CR & DMA_SxCR_EN);
}

// USART2: TX
void DMA1_Stream6_IRQHandler(void) {

    if (DMA1->HISR & DMA_HISR_TCIF6) {
        // Clear interrupt flags
        DMA1->HIFCR = (DMA_HIFCR_CFEIF6 | DMA_HIFCR_CDMEIF6 |
                       DMA_HIFCR_CTEIF6 | DMA_HIFCR_CHTIF6  |
                       DMA_HIFCR_CTCIF6);
        
        // Poll till transmission is complete
        while (!(USART2->SR & USART_SR_TC));

        if (s_uart_tx_done_cbs[1]) {
            // Invoke user callback
            s_uart_tx_done_cbs[1](s_tx_args[1]);

            // Clear user passed context
            s_uart_tx_done_cbs[1] = NULL;
            s_tx_args[1] = NULL;
        }

    } else {
        while (1);
    }

    // Disable DMA TX stream
    DMA1_Stream6->CR &= ~DMA_SxCR_EN;
    while (DMA1_Stream6->CR & DMA_SxCR_EN);
}

// USART2: RX
void DMA1_Stream5_IRQHandler(void) {

    if (DMA1->HISR & DMA_HISR_TCIF5) {
        // Clear interrupt flags
        DMA1->HIFCR = (DMA_HIFCR_CFEIF5 | DMA_HIFCR_CDMEIF5 |
                       DMA_HIFCR_CTEIF5 | DMA_HIFCR_CHTIF5  |
                       DMA_HIFCR_CTCIF5);
        
        if (s_uart_rx_done_cbs[1]) {
            // Invoke user callback
            s_uart_rx_done_cbs[1](s_rx_args[1]);

            // Clear user passed context
            s_uart_rx_done_cbs[1] = NULL;
            s_rx_args[1] = NULL;
        }

    } else {
        while (1);
    }
    
    // Disable DMA RX stream
    DMA1_Stream5->CR &= ~DMA_SxCR_EN;
    while (DMA1_Stream5->CR & DMA_SxCR_EN);
}

// USART6: TX
void DMA2_Stream6_IRQHandler(void) {

    if (DMA2->HISR & DMA_HISR_TCIF6) {
        // Clear interrupt flags
        DMA2->HIFCR = (DMA_HIFCR_CFEIF6 | DMA_HIFCR_CDMEIF6 |
                       DMA_HIFCR_CTEIF6 | DMA_HIFCR_CHTIF6  |
                       DMA_HIFCR_CTCIF6);
        
        // Poll till transmission is complete
        while (!(USART6->SR & USART_SR_TC));

        if (s_uart_tx_done_cbs[2]) {
            // Invoke user callback
            s_uart_tx_done_cbs[2](s_tx_args[2]);

            // Clear user passed context
            s_uart_tx_done_cbs[2] = NULL;
            s_tx_args[2] = NULL;
        }

    } else {
        while (1);
    }

    // Disable DMA TX stream
    DMA2_Stream6->CR &= ~DMA_SxCR_EN;
    while (DMA2_Stream6->CR & DMA_SxCR_EN);
}

// USART6: RX
void DMA2_Stream1_IRQHandler(void) {

    if (DMA2->LISR & DMA_LISR_TCIF1) {
        // Clear interrupt flags
        DMA2->LIFCR = (DMA_LIFCR_CFEIF1 | DMA_LIFCR_CDMEIF1 |
                       DMA_LIFCR_CTEIF1 | DMA_LIFCR_CHTIF1  |
                       DMA_LIFCR_CTCIF1);
        
        if (s_uart_rx_done_cbs[2]) {
            // Invoke user callback
            s_uart_rx_done_cbs[2](s_rx_args[2]);

            // Clear user passed context
            s_uart_rx_done_cbs[2] = NULL;
            s_rx_args[2] = NULL;
        }

    } else {
        while (1);
    }
    
    // Disable DMA RX stream
    DMA2_Stream1->CR &= ~DMA_SxCR_EN;
    while (DMA2_Stream1->CR & DMA_SxCR_EN);
}
