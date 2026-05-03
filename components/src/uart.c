#include "stm32f411xe.h"
#include "dma.h"
#include "gpio.h"
#include "uart.h"


// The 3 UART channels: ISRs called when the DMA is done
// TX
uart_dma_trans_done_cb_t s_dma_tx_done_cbs[3] = {};
void* s_tx_args[3] = {};
// RX
uart_dma_trans_done_cb_t s_dma_rx_done_cbs[3] = {};
void* s_rx_args[3] = {};

// Utilities for mapping the USART channels to DMA streams
typedef struct {
    DMA_TypeDef* controller;
    DMA_Stream_TypeDef* stream;
    uint8_t stream_no;
    IRQn_Type irq_type;
    uint8_t channel;
} uart_tx_rx_t;

typedef struct {
    const uart_tx_rx_t tx;
    const uart_tx_rx_t rx;
} dma_stream_map_t;

#define UART_DMA_NVIC_IRQ_PRIORITY 8U
#define TIMEOUT_CYCLES 100UL

// Mapping for the DMA channels for the 3 USART channels
static const dma_stream_map_t s_uart_dma_map[3] = {
    // USART1
    {
        .tx = { .controller = DMA2, .stream = DMA2_Stream7, .stream_no = 7, .irq_type = DMA2_Stream7_IRQn, .channel = 4 },
        .rx = { .controller = DMA2, .stream = DMA2_Stream5, .stream_no = 5, .irq_type = DMA2_Stream5_IRQn, .channel = 4 }
    },
    // USART2
    {
        .tx = { .controller = DMA1, .stream = DMA1_Stream6, .stream_no = 6, .irq_type = DMA1_Stream6_IRQn, .channel = 4 },
        .rx = { .controller = DMA1, .stream = DMA1_Stream5, .stream_no = 5, .irq_type = DMA1_Stream5_IRQn, .channel = 4 }
    },
    // USART6
    {
        .tx = { .controller = DMA2, .stream = DMA2_Stream6, .stream_no = 6, .irq_type = DMA2_Stream6_IRQn, .channel = 5 },
        .rx = { .controller = DMA2, .stream = DMA2_Stream1, .stream_no = 1, .irq_type = DMA2_Stream1_IRQn, .channel = 5 }
    }
};

// Helper
static inline uint8_t get_index(const USART_TypeDef* handle) {
    if      (handle == USART1) return 0U;
    else if (handle == USART2) return 1U;
    else if (handle == USART6) return 2U;
    else                       return 0xFFU;
}

static inline hal_err_t isr_helper(DMA_Stream_TypeDef* stream, volatile uint32_t* irq_clr_rg,
                                   volatile uint32_t* irq_sta_rg, uint32_t tc, uint32_t te, uint32_t dme) {

    hal_err_t error = HAL_OK;

    // Transfer complete
    if (*irq_sta_rg & tc) {
        // Clear DMA TC interrupt bit
        *irq_clr_rg = tc;
    }
    // Transfer error
    else if (*irq_sta_rg & te) {
        // Clear DMA TE interrupt bit
        *irq_clr_rg = te;
        error = HAL_UART_DMA_TE;
    }
    // Direct mode error
    else if (*irq_sta_rg & dme) {
        // Clear DMA DME interrupt bit
        *irq_clr_rg = dme;
        error = HAL_UART_DMA_DME;
    }
    // Unreachable, but default catch-all
    else {
        // Clear DMA TC, DME and TE interrupt bits
        *irq_clr_rg = (tc | te | dme);
        error = HAL_UART_DMA_ERR_UNKNOWN;
    }
    
    if (dma_disable_stream(stream) != HAL_OK) error = HAL_TIMEOUT;

    return error;
}

static inline void isr_tx_helper(USART_TypeDef* handle, hal_err_t ret, uint8_t idx) {

    if (!s_dma_tx_done_cbs[idx]) return;

    // Transfers require us to poll on the TC flag
    // even after data has been shifted out
    
    // Poll till TC has been set
    // Skip polling if an error has occurred
    if (ret == HAL_OK) {
        uint32_t timeout = TIMEOUT_CYCLES;
        while (!(handle->SR & USART_SR_TC) && (--timeout));
        if (timeout == 0) ret = HAL_UART_TC_FAILED_TO_SET;
    }
    
    // Invoke user callback
    s_dma_tx_done_cbs[idx](s_tx_args[idx], ret);

    // Clear user passed context
    s_dma_tx_done_cbs[idx] = NULL;
    s_tx_args[idx] = NULL;
}

static inline void isr_rx_helper(hal_err_t ret, uint8_t idx) {

    if (!s_dma_rx_done_cbs[idx]) return;
    
    // Invoke user callback
    s_dma_rx_done_cbs[idx](s_rx_args[idx], ret);

    // Clear user passed context
    s_dma_rx_done_cbs[idx] = NULL;
    s_rx_args[idx] = NULL;
}


// Public API
hal_err_t uart_init(USART_TypeDef* handle, const uart_config_t* config) {

    // Get the alternate function value as it
    // varies for each peripheral instance
    uint8_t alt_val = 0;
    
    // Enable appropriate UART channel clock
    if (handle == USART1) {
        RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
        alt_val = 7UL;
    } else if (handle == USART2) {
        RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
        alt_val = 7UL;
    } else if (handle == USART6) {
        RCC->APB2ENR |= RCC_APB2ENR_USART6EN;
        alt_val = 8UL;
    } else {
        return HAL_INVALID_ARG;
    }

    __DSB();
    
    // Disable the UART peripheral
    handle->CR1 &= ~USART_CR1_UE;

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
        return HAL_INVALID_ARG;
    }

    // Initialize GPIO pins for UART
    // Enable gpio channel clock
    hal_err_t ret = gpiox_clk_enable(config->gpio_port);
    if (ret != HAL_OK) return ret;

    // Set gpio pin to alternate function
    ret = gpio_set_alternate_function(config->gpio_port, config->tx_pin, alt_val);
    if (ret != HAL_OK) return ret;
    ret = gpio_set_alternate_function(config->gpio_port, config->rx_pin, alt_val);
    if (ret != HAL_OK) return ret;

    // Set pullup
    gpio_enable_pullup(config->gpio_port, config->tx_pin, true);
    gpio_enable_pullup(config->gpio_port, config->rx_pin, true);
    
    // Speed mode
    gpio_set_speed_mode(config->gpio_port, config->tx_pin, GPIO_HIGH_SPEED);
    gpio_set_speed_mode(config->gpio_port, config->rx_pin, GPIO_HIGH_SPEED);
    
    // Enable the UART peripheral
    handle->CR1 |= USART_CR1_UE;

    return HAL_OK;
}

hal_err_t uart_dma_init(USART_TypeDef* handle) {
    
    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU) return HAL_INVALID_ARG;
    
    // TX mapping
    DMA_TypeDef* tx_controller = s_uart_dma_map[idx].tx.controller;
    DMA_Stream_TypeDef* tx_stream = s_uart_dma_map[idx].tx.stream;
    uint8_t tx_channel = s_uart_dma_map[idx].tx.channel;

    // RX mapping
    DMA_TypeDef* rx_controller = s_uart_dma_map[idx].rx.controller;
    DMA_Stream_TypeDef* rx_stream = s_uart_dma_map[idx].rx.stream;
    uint8_t rx_channel = s_uart_dma_map[idx].rx.channel;

    // Enable DMA clock
    hal_err_t ret = dmax_clk_enable(tx_controller);
    if (ret != HAL_OK) return ret;
    ret = dmax_clk_enable(rx_controller);
    if (ret != HAL_OK) return ret;

    // Disable DMA streams for tx and rx
    ret = dma_disable_stream(tx_stream);
    if (ret != HAL_OK) return ret;
    ret = dma_disable_stream(rx_stream);
    if (ret != HAL_OK) return ret;
    
    // Clear global DMA interrupt flags
    dma_clear_flags(tx_controller, s_uart_dma_map[idx].tx.stream_no);
    dma_clear_flags(rx_controller, s_uart_dma_map[idx].rx.stream_no);
    
    // Configuration
    // TX
    dma_set_channel(tx_stream, tx_channel);
    dma_set_stream_priority(tx_stream, DMA_PRIORITY_MEDIUM);
    dma_set_direction(tx_stream, DMA_DIR_M_P);
    dma_enable_irqs(tx_stream, true, true, false, true);
    dma_set_increment(tx_stream, false, true);
    dma_set_per_mem_size(tx_stream, DMA_SIZE_BYTE, DMA_SIZE_BYTE);
    dma_enable_circm_dbm(tx_stream, false, false);
    dma_set_flow_controller(tx_stream, true);
    dma_set_direct_mode(tx_stream, true);
    dma_set_addresses(tx_stream, &handle->DR, NULL, NULL);

    // RX
    dma_set_channel(rx_stream, rx_channel);
    dma_set_stream_priority(rx_stream, DMA_PRIORITY_MEDIUM);
    dma_set_direction(rx_stream, DMA_DIR_P_M);
    dma_enable_irqs(rx_stream, true, true, false, true);
    dma_set_increment(rx_stream, false, true);
    dma_set_per_mem_size(rx_stream, DMA_SIZE_BYTE, DMA_SIZE_BYTE);
    dma_enable_circm_dbm(rx_stream, false, false);
    dma_set_flow_controller(rx_stream, true);
    dma_set_direct_mode(rx_stream, true);
    dma_set_addresses(rx_stream, &handle->DR, NULL, NULL);
    
    // Enable USART DMA
    handle->CR3 |= (USART_CR3_DMAT | USART_CR3_DMAR);

    // Enable DMA stream interrupts
    NVIC_EnableIRQ(s_uart_dma_map[idx].tx.irq_type);
    NVIC_SetPriority(s_uart_dma_map[idx].tx.irq_type, UART_DMA_NVIC_IRQ_PRIORITY);

    NVIC_EnableIRQ(s_uart_dma_map[idx].rx.irq_type);
    NVIC_SetPriority(s_uart_dma_map[idx].rx.irq_type, UART_DMA_NVIC_IRQ_PRIORITY);

    return HAL_OK;
}

void uart_enable(USART_TypeDef* handle) {
    handle->CR1 |= (USART_CR1_TE | USART_CR1_RE);
}

void uart_disable(USART_TypeDef* handle) {
    handle->CR1 &= ~(USART_CR1_TE | USART_CR1_RE);
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

hal_err_t uart_transmit_dma(USART_TypeDef* handle, const uint8_t* data, uint16_t len,
                            uart_dma_trans_done_cb_t callback, void* arg) {
    
    // Get index for DMA stream mapping
    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU) return HAL_INVALID_ARG;
    
    // Save user passed callback
    if (callback) {
        s_dma_tx_done_cbs[idx] = callback;
        s_tx_args[idx] = arg;
    }
    
    // TX mapping
    DMA_Stream_TypeDef* stream = s_uart_dma_map[idx].tx.stream;
    
    // Set memory address and length
    dma_set_addresses(stream, NULL, data, NULL);
    dma_set_trans_length(stream, len);
    
    // Enable DMA TX stream
    return dma_enable_stream(stream);
}

hal_err_t uart_receive_dma(USART_TypeDef* handle, uint8_t* data, uint16_t len,
                           uart_dma_trans_done_cb_t callback, void* arg) {
    
    // Get index for DMA stream mapping
    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU) return HAL_INVALID_ARG;
    
    // Save user passed callback
    if (callback) {
        s_dma_rx_done_cbs[idx] = callback;
        s_rx_args[idx] = arg;
    }
    
    // RX mapping
    DMA_Stream_TypeDef* stream = s_uart_dma_map[idx].rx.stream;

    // Set memory address and length
    dma_set_addresses(stream, NULL, data, NULL);
    dma_set_trans_length(stream, len);
    
    // Enable DMA RX stream
    return dma_enable_stream(stream);
}

// DMA interrupts
// USART1: TX
void DMA2_Stream7_IRQHandler(void) {
    hal_err_t ret = isr_helper(DMA2_Stream7, &DMA2->HIFCR, &DMA2->HISR, DMA_HISR_TCIF7, DMA_HISR_TEIF7, DMA_HISR_DMEIF7);
    isr_tx_helper(USART1, ret, 0);
}

// USART1: RX
void DMA2_Stream5_IRQHandler(void) {
    hal_err_t ret = isr_helper(DMA2_Stream5, &DMA2->HIFCR, &DMA2->HISR, DMA_HISR_TCIF5, DMA_HISR_TEIF5, DMA_HISR_DMEIF5);
    isr_rx_helper(ret, 0);
}

// USART2: TX
void DMA1_Stream6_IRQHandler(void) {
    hal_err_t ret = isr_helper(DMA1_Stream6, &DMA1->HIFCR, &DMA1->HISR, DMA_HISR_TCIF6, DMA_HISR_TEIF6, DMA_HISR_DMEIF6);
    isr_tx_helper(USART2, ret, 1);
}

// USART2: RX
void DMA1_Stream5_IRQHandler(void) {
    hal_err_t ret = isr_helper(DMA1_Stream5, &DMA1->HIFCR, &DMA1->HISR, DMA_HISR_TCIF5, DMA_HISR_TEIF5, DMA_HISR_DMEIF5);
    isr_rx_helper(ret, 1);
}

// USART6: TX
void DMA2_Stream6_IRQHandler(void) {
    hal_err_t ret = isr_helper(DMA2_Stream6, &DMA2->HIFCR, &DMA2->HISR, DMA_HISR_TCIF6, DMA_HISR_TEIF6, DMA_HISR_DMEIF6);
    isr_tx_helper(USART6, ret, 2);
}

// USART6: RX
void DMA2_Stream1_IRQHandler(void) {
    hal_err_t ret = isr_helper(DMA2_Stream1, &DMA2->LIFCR, &DMA2->LISR, DMA_LISR_TCIF1, DMA_LISR_TEIF1, DMA_LISR_DMEIF1);
    isr_rx_helper(ret, 2);
}
