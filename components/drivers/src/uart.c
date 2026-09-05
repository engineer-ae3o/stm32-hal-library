#include "stm32f411xe.h"
#include "drivers/gpio.h"
#include "drivers/uart.h"
#include "utils/common.h"
#include "drivers/dma.h"
#include "utils/err.h"


// The 3 UaRT instances: ISRs called when the DMA is done
static dma_stream_ctx_t s_dma_stream_ctx[3] = {};

// Mapping for the DMA streams to the 3 UART peripheral instances
static const dma_stream_map_t s_uart_dma_map[] = {
    // USART1
    {
        .tx = {.stream = DMA2_Stream7, .channel = 4},
        .rx = {.stream = DMA2_Stream2, .channel = 4},
    },
    // USART2
    {
        .tx = {.stream = DMA1_Stream6, .channel = 4},
        .rx = {.stream = DMA1_Stream5, .channel = 4},
    },
    // USART6: DMA not supported: Not enough streams to go round other peripherals
    {
        .tx = {.stream = NULL, .channel = 0},
        .rx = {.stream = NULL, .channel = 0},
    },
};


// Helper
[[__gnu__::__always_inline__]] static inline uint8_t get_index(const USART_TypeDef* handle) {
    if (handle == USART1) {
        return 0U;
    } else if (handle == USART2) {
        return 1U;
    } else if (handle == USART6) {
        return 2U;
    } else {
        return 0xFFU;
    }
}

[[__gnu__::__always_inline__]] static inline void isr_tx_helper(USART_TypeDef* handle) {
    const uint8_t idx = get_index(handle);
    ASSERT(idx != 0xFFU);

    // Clear any flags that were set and get the error status
    hal_err_t ret = dma_isr_helper(s_uart_dma_map[idx].tx.stream);

    if (!s_dma_stream_ctx[idx].tx.callback) {
        return;
    }

    // Transfers require us to poll on the TC flag
    // even after data has been shifted out

    // Poll till the TC bit has been set
    // Skip polling if an error has occurred
    if (ret == HAL_OK) {
        uint32_t timeout = TIMEOUT_CYCLES;
        while (!(handle->SR & USART_SR_TC) && (--timeout));
        if (timeout == 0) {
            ret = HAL_ERR_UART_TC_FAILED_TO_SET;
        }
    }

    // Save the user callback so we can clear it's global array position
    const dma_done_cb_t local_cb  = s_dma_stream_ctx[idx].tx.callback;
    void* const         local_arg = s_dma_stream_ctx[idx].tx.arg;

    // Clear the user passed callback since this is a one-off event
    s_dma_stream_ctx[idx].tx.callback = NULL;
    s_dma_stream_ctx[idx].tx.arg      = NULL;

    // Disable UART TX DMA
    handle->CR3 &= ~USART_CR3_DMAT;

    // Finally, invoke the user callback
    local_cb(local_arg, ret);
}

[[__gnu__::__always_inline__]] static inline void isr_rx_helper(USART_TypeDef* handle) {
    const uint8_t idx = get_index(handle);
    ASSERT(idx != 0xFFU);

    // Clear any flags that were set and get the error status
    hal_err_t ret = dma_isr_helper(s_uart_dma_map[idx].rx.stream);

    if (!s_dma_stream_ctx[idx].rx.callback) {
        return;
    }

    // Save the user callback so we can clear it's global array position
    const dma_done_cb_t local_cb  = s_dma_stream_ctx[idx].rx.callback;
    void* const         local_arg = s_dma_stream_ctx[idx].rx.arg;

    // Clear the user passed callback since this is a one-off event
    s_dma_stream_ctx[idx].rx.callback = NULL;
    s_dma_stream_ctx[idx].rx.arg      = NULL;

    // Disable UART RX DMA
    handle->CR3 &= ~USART_CR3_DMAR;

    // Finally, invoke the user callback
    local_cb(local_arg, ret);
}

// Public API
hal_err_t uartx_clk_enable(USART_TypeDef* handle, bool enable) {
    if (enable) {
        if (handle == USART1) {
            RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
        } else if (handle == USART2) {
            RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
        } else if (handle == USART6) {
            RCC->APB2ENR |= RCC_APB2ENR_USART6EN;
        } else {
            return HAL_ERR_INVALID_ARG;
        }

    } else {
        if (handle == USART1) {
            RCC->APB2ENR &= ~RCC_APB2ENR_USART1EN;
        } else if (handle == USART2) {
            RCC->APB1ENR &= ~RCC_APB1ENR_USART2EN;
        } else if (handle == USART6) {
            RCC->APB2ENR &= ~RCC_APB2ENR_USART6EN;
        } else {
            return HAL_ERR_INVALID_ARG;
        }
    }

    __DSB();
    return HAL_OK;
}

hal_err_t uart_init(USART_TypeDef* handle, const uart_config_t* config) {
    if (handle == NULL || config == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    // Disable the UART peripheral before modifying its registers
    handle->CR1 &= ~USART_CR1_UE;

    // 8 bit UART and parity bit disabled
    handle->CR1 &= ~(USART_CR1_M | USART_CR1_PCE);

    // Baud rate generator
    const float uart_div = (float)config->clock_freq_hz / (float)(config->baud_rate * config->over_sampling);

    // Get the mantissa and the fractional parts of the uart clock divider
    const uint16_t mantissa = (uint16_t)uart_div;
    const uint16_t fraction = (uint16_t)((uart_div - (float)mantissa) * (float)config->over_sampling);

    // Set oversampling and baud rate divider
    // The fractional part can only be 3 bits if oversampling is 8
    if (config->over_sampling == UART_OVER_SAMPLING_8) {
        handle->CR1 |= USART_CR1_OVER8;
        handle->BRR = (uint32_t)(mantissa << USART_BRR_DIV_Mantissa_Pos) | (fraction & 0x07U);
    } else if (config->over_sampling == UART_OVER_SAMPLING_16) {
        handle->CR1 &= ~USART_CR1_OVER8;
        handle->BRR = (uint32_t)(mantissa << USART_BRR_DIV_Mantissa_Pos) | (fraction & 0x0FU);
    } else {
        return HAL_ERR_INVALID_ARG;
    }

    // Initialize GPIO pins for UART
    // Enable gpio channel clock
    TRY(gpiox_clk_enable(config->gpio_port, true));

    // Get the alternate function value as it
    // varies for each peripheral instance
    const uint8_t alt_val = (handle == USART6) ? 8U : 7U;

    // Set gpio pin to alternate function
    TRY(gpio_set_alternate_function(config->gpio_port, config->tx_pin, alt_val));
    TRY(gpio_set_alternate_function(config->gpio_port, config->rx_pin, alt_val));

    // Set as push pull
    gpio_set_output_type(config->gpio_port, config->tx_pin, GPIO_PUSH_PULL);
    gpio_set_output_type(config->gpio_port, config->rx_pin, GPIO_PUSH_PULL);

    // Set pullup
    gpio_enable_pullup(config->gpio_port, config->tx_pin, true);
    gpio_enable_pullup(config->gpio_port, config->rx_pin, true);

    // Speed mode
    gpio_set_speed_mode(config->gpio_port, config->tx_pin, GPIO_MEDIUM_SPEED);
    gpio_set_speed_mode(config->gpio_port, config->rx_pin, GPIO_MEDIUM_SPEED);

    // Enable the UART peripheral
    handle->CR1 |= USART_CR1_UE;

    return HAL_OK;
}

hal_err_t uart_enable(USART_TypeDef* handle, bool enable) {
    if (handle == NULL) {
        return HAL_ERR_INVALID_ARG;
    }
    if (enable) {
        handle->CR1 |= (USART_CR1_TE | USART_CR1_RE);
    } else {
        handle->CR1 &= ~(USART_CR1_TE | USART_CR1_RE);
    }
    return HAL_OK;
}

hal_err_t uart_dma_init(USART_TypeDef* handle, dma_priority_t priority) {
    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU) {
        return HAL_ERR_INVALID_ARG;
    }

    // TX mapping
    DMA_Stream_TypeDef* tx_stream  = s_uart_dma_map[idx].tx.stream;
    const uint8_t       tx_channel = s_uart_dma_map[idx].tx.channel;

    // RX mapping
    DMA_Stream_TypeDef* rx_stream  = s_uart_dma_map[idx].rx.stream;
    const uint8_t       rx_channel = s_uart_dma_map[idx].rx.channel;

    if (tx_stream == NULL || rx_stream == NULL) {
        return HAL_ERR_NOT_SUPPORTED;
    }

    // TX
    TRY(dma_disable_stream(tx_stream));
    dma_set_channel(tx_stream, tx_channel);
    dma_set_stream_priority(tx_stream, priority);
    dma_set_direction(tx_stream, DMA_DIR_M_P);
    dma_enable_irqs(tx_stream, true, true, false, true);
    dma_set_increment(tx_stream, false, true);
    dma_set_per_mem_size(tx_stream, DMA_SIZE_BYTE, DMA_SIZE_BYTE);
    dma_enable_circm_dbm(tx_stream, false, false);
    dma_set_flow_controller(tx_stream, true);
    dma_set_direct_mode(tx_stream, true);

    // RX
    TRY(dma_disable_stream(rx_stream));
    dma_set_channel(rx_stream, rx_channel);
    dma_set_stream_priority(rx_stream, priority);
    dma_set_direction(rx_stream, DMA_DIR_P_M);
    dma_enable_irqs(rx_stream, true, true, false, true);
    dma_set_increment(rx_stream, false, true);
    dma_set_per_mem_size(rx_stream, DMA_SIZE_BYTE, DMA_SIZE_BYTE);
    dma_enable_circm_dbm(rx_stream, false, false);
    dma_set_flow_controller(rx_stream, true);
    dma_set_direct_mode(rx_stream, true);

    return HAL_OK;
}

hal_err_t uart_transmit_byte(USART_TypeDef* handle, uint8_t byte) {
    if (handle == NULL) {
        return HAL_ERR_INVALID_ARG;
    }
    // Wait till the data register is empty
    while (!(handle->SR & USART_SR_TXE));
    handle->DR = byte;
    return HAL_OK;
}

hal_err_t uart_transmit_poll(USART_TypeDef* handle, const uint8_t* data, size_t size) {
    if (handle == NULL || data == NULL || size == 0) {
        return HAL_ERR_INVALID_ARG;
    }
    for (size_t i = 0; i < size; i++) {
        uart_transmit_byte(handle, data[i]);
    }
    // Wait till all bytes have been fully transmitted
    while (!(handle->SR & USART_SR_TC));
    return HAL_OK;
}

hal_err_t uart_transmit_dma(USART_TypeDef* handle, const uint8_t* data, uint16_t size, dma_done_cb_t callback, void* arg) {
    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU) {
        return HAL_ERR_INVALID_ARG;
    }

    if (data == NULL || size == 0) {
        return HAL_ERR_INVALID_ARG;
    }

    // TX mapping
    DMA_Stream_TypeDef* stream = s_uart_dma_map[idx].tx.stream;
    if (stream == NULL) {
        return HAL_ERR_NOT_SUPPORTED;
    }

    dma_set_addresses(stream, &handle->DR, data, NULL);
    dma_set_trans_length(stream, size);

    // Save the user passed callback
    if (callback) {
        s_dma_stream_ctx[idx].tx.callback = callback;
        s_dma_stream_ctx[idx].tx.arg      = arg;
    }

    // Enable the DMA TX stream
    TRY(dma_enable_stream(stream));

    // Enable USART DMA
    handle->CR3 |= USART_CR3_DMAT;

    return HAL_OK;
}

hal_err_t uart_receive_dma(USART_TypeDef* handle, uint8_t* data, uint16_t size, dma_done_cb_t callback, void* arg) {
    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU) {
        return HAL_ERR_INVALID_ARG;
    }

    if (data == NULL || size == 0) {
        return HAL_ERR_INVALID_ARG;
    }

    // RX mapping
    DMA_Stream_TypeDef* stream = s_uart_dma_map[idx].rx.stream;
    if (stream == NULL) {
        return HAL_ERR_NOT_SUPPORTED;
    }

    dma_set_addresses(stream, &handle->DR, data, NULL);
    dma_set_trans_length(stream, size);

    // Save the user passed callback
    if (callback) {
        s_dma_stream_ctx[idx].rx.callback = callback;
        s_dma_stream_ctx[idx].rx.arg      = arg;
    }

    // Enable the DMA RX stream
    TRY(dma_enable_stream(stream));

    // Enable USART DMA
    handle->CR3 |= USART_CR3_DMAR;

    return HAL_OK;
}

// DMA interrupts
// USART1: TX
void DMA2_Stream7_IRQHandler(void) {
    isr_tx_helper(USART1);
}

// USART1: RX
void DMA2_Stream2_IRQHandler(void) {
    isr_rx_helper(USART1);
}

// USART2: TX
void DMA1_Stream6_IRQHandler(void) {
    isr_tx_helper(USART2);
}

// USART2: RX
void DMA1_Stream5_IRQHandler(void) {
    isr_rx_helper(USART2);
}
