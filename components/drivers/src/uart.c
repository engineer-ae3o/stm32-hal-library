#include "stm32f411xe.h"
#include "drivers/gpio.h"
#include "drivers/uart.h"
#include "utils/common.h"
#include "utils/clock.h"
#include "drivers/dma.h"
#include "utils/err.h"

#include <string.h>
#include <stddef.h>


// Mapping for the DMA streams to the UART peripheral instances
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

// The UaRT instances: The ISRs invoked when a DMA event occurred
static dma_stream_ctx_t s_dma_stream_ctx[ARRAY_SIZE(s_uart_dma_map)] = {};

#define ENABLE_UART_TX()                                                                                                                             \
    do {                                                                                                                                             \
        handle->CR1 |= USART_CR1_TE;                                                                                                                 \
        __DSB();                                                                                                                                     \
    } while (0)

#define DISABLE_UART_TX()                                                                                                                            \
    do {                                                                                                                                             \
        handle->CR1 &= ~USART_CR1_TE;                                                                                                                \
        __DSB();                                                                                                                                     \
    } while (0)

#define ENABLE_UART_RX()                                                                                                                             \
    do {                                                                                                                                             \
        handle->CR1 |= USART_CR1_RE;                                                                                                                 \
        __DSB();                                                                                                                                     \
    } while (0)

#define DISABLE_UART_RX()                                                                                                                            \
    do {                                                                                                                                             \
        handle->CR1 &= ~USART_CR1_RE;                                                                                                                \
        __DSB();                                                                                                                                     \
    } while (0)


// Helpers
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

    // Clear the status flags
    handle->SR &= ~(USART_SR_TC | USART_SR_TXE);

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

    // Disable the UART TX peripheral after all transactions have completed
    DISABLE_UART_TX();

    __disable_irq();

    // Return if no callback registered
    if (s_dma_stream_ctx[idx].tx.callback == NULL) {
        __enable_irq();
        return;
    }

    // Save the user callback so we can clear it's global array position
    const dma_done_cb_t local_cb  = s_dma_stream_ctx[idx].tx.callback;
    void* const         local_arg = s_dma_stream_ctx[idx].tx.arg;

    // Clear the user passed callback since this is a one-off event
    s_dma_stream_ctx[idx].tx.callback = NULL;
    s_dma_stream_ctx[idx].tx.arg      = NULL;

    __enable_irq();

    // Finally, invoke the user callback
    local_cb(local_arg, ret);
}

[[__gnu__::__always_inline__]] static inline void isr_rx_helper(USART_TypeDef* handle) {
    const uint8_t idx = get_index(handle);
    ASSERT(idx != 0xFFU);

    // Clear any flags that were set and get the error status
    hal_err_t ret = dma_isr_helper(s_uart_dma_map[idx].rx.stream);

    // Clear the status flag
    handle->SR &= ~USART_SR_RXNE;

    // Disable the UART RX peripheral
    DISABLE_UART_RX();

    __disable_irq();

    // Return if no callback registered
    if (s_dma_stream_ctx[idx].rx.callback == NULL) {
        __enable_irq();
        return;
    }

    // Save the user callback so we can clear it's global array position
    const dma_done_cb_t local_cb  = s_dma_stream_ctx[idx].rx.callback;
    void* const         local_arg = s_dma_stream_ctx[idx].rx.arg;

    // Clear the user passed callback since this is a one-off event
    s_dma_stream_ctx[idx].rx.callback = NULL;
    s_dma_stream_ctx[idx].rx.arg      = NULL;

    __enable_irq();

    // Finally, invoke the user callback
    local_cb(local_arg, ret);
}


// General API
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

    // Get the frequency of the bus clock on which the uart peripheral lives on
    const uint32_t bus_clock_freq_hz = (handle == USART1 || handle == USART6) ? get_apb2_core_clock() : get_apb1_core_clock();

    // Baud rate generator: Get the UART bus divider from the target baud rate and bus clock frequency
    const float uart_div = (float)bus_clock_freq_hz / (float)(config->baud_rate * config->over_sampling);

    // Get the mantissa and the fractional parts of the uart clock divider
    const uint16_t mantissa = (uint16_t)uart_div;
    const uint16_t fraction = (uint16_t)((uart_div - (float)mantissa) * (float)config->over_sampling);

    // Set oversampling and baud rate divider
    // The fractional part can only be 3 bits if oversampling is 8
    handle->BRR &= ~(USART_BRR_DIV_Mantissa | USART_BRR_DIV_Fraction);
    if (config->over_sampling == UART_OVER_SAMPLING_8) {
        handle->CR1 |= USART_CR1_OVER8;
        handle->BRR |= ((uint32_t)(mantissa << USART_BRR_DIV_Mantissa_Pos) & USART_BRR_DIV_Mantissa) | (fraction & 0x07U);
    } else if (config->over_sampling == UART_OVER_SAMPLING_16) {
        handle->CR1 &= ~USART_CR1_OVER8;
        handle->BRR |= ((uint32_t)(mantissa << USART_BRR_DIV_Mantissa_Pos) & USART_BRR_DIV_Mantissa) | (fraction & 0x0FU);
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

hal_err_t uart_deinit(USART_TypeDef* handle) {
    if (handle == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    DISABLE_UART_RX();
    DISABLE_UART_TX();

    handle->CR1 &= ~(USART_CR1_SBK | USART_CR1_RWU | USART_CR1_RE | USART_CR1_TE | USART_CR1_IDLEIE | USART_CR1_RXNEIE | USART_CR1_TCIE |
                     USART_CR1_TXEIE | USART_CR1_PEIE | USART_CR1_PS | USART_CR1_PCE | USART_CR1_WAKE | USART_CR1_M | USART_CR1_OVER8);
    handle->BRR &= ~(USART_BRR_DIV_Fraction | USART_BRR_DIV_Mantissa);
    handle->SR &= ~(USART_SR_TC | USART_SR_TXE | USART_SR_RXNE);

    // Disable the UART peripheral
    handle->CR1 &= ~USART_CR1_UE;

    return HAL_OK;
}

hal_err_t uart_dma_init(USART_TypeDef* handle, dma_priority_t priority) {
    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU) {
        return HAL_ERR_INVALID_ARG;
    }

    // TX mapping
    DMA_Stream_TypeDef* tx_stream = s_uart_dma_map[idx].tx.stream;
    DMA_Stream_TypeDef* rx_stream = s_uart_dma_map[idx].rx.stream;

    if (tx_stream == NULL || rx_stream == NULL) {
        return HAL_ERR_NOT_SUPPORTED;
    }

    // DMA TX stream configuration
    const dma_stream_config_t tx_stream_config = {
        .deconfigure   = false,
        .enable_stream = false,

        .per_addr_incement = false,
        .mem_addr_incement = true,

        .tc_irq_enable  = true,
        .ht_irq_enable  = false,
        .te_irq_enable  = true,
        .dme_irq_enable = true,
        .fe_irq_enable  = false,

        .mode            = DMA_MODE_DIRECT,
        .priority        = priority,
        .direction       = DMA_DIR_M_P,
        .per_data_size   = DMA_SIZE_BYTE,
        .mem_data_size   = DMA_SIZE_BYTE,
        .circular_mode   = DMA_MODE_NO_CIRCULAR,
        .flow_controller = DMA_FLOW_CONTROLLER_DMA,

        .buffer_size       = 0,
        .channel           = s_uart_dma_map[idx].tx.channel,
        .nvic_irq_priority = UART_DMA_NVIC_IRQ_PRIORITY,

        .per_addr  = NULL,
        .mem_buf_0 = NULL,
        .mem_buf_1 = NULL,
    };

    // DMA RX stream configuration
    const dma_stream_config_t rx_stream_config = {
        .deconfigure   = false,
        .enable_stream = false,

        .per_addr_incement = false,
        .mem_addr_incement = true,

        .tc_irq_enable  = true,
        .ht_irq_enable  = false,
        .te_irq_enable  = true,
        .dme_irq_enable = true,
        .fe_irq_enable  = false,

        .mode            = DMA_MODE_DIRECT,
        .priority        = priority,
        .direction       = DMA_DIR_P_M,
        .per_data_size   = DMA_SIZE_BYTE,
        .mem_data_size   = DMA_SIZE_BYTE,
        .circular_mode   = DMA_MODE_NO_CIRCULAR,
        .flow_controller = DMA_FLOW_CONTROLLER_DMA,

        .buffer_size       = 0,
        .channel           = s_uart_dma_map[idx].rx.channel,
        .nvic_irq_priority = UART_DMA_NVIC_IRQ_PRIORITY,

        .per_addr  = NULL,
        .mem_buf_0 = NULL,
        .mem_buf_1 = NULL,
    };

    TRY(dma_configure_stream(tx_stream, &tx_stream_config));
    TRY(dma_configure_stream(rx_stream, &rx_stream_config));

    // Enable USART TX and RX DMA requests
    handle->CR3 |= (USART_CR3_DMAT | USART_CR3_DMAR);

    return HAL_OK;
}

hal_err_t uart_dma_deinit(USART_TypeDef* handle) {
    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU) {
        return HAL_ERR_INVALID_ARG;
    }

    DMA_Stream_TypeDef* tx_stream = s_uart_dma_map[idx].tx.stream;
    DMA_Stream_TypeDef* rx_stream = s_uart_dma_map[idx].rx.stream;

    if (tx_stream == NULL || rx_stream == NULL) {
        return HAL_ERR_NOT_SUPPORTED;
    }

    // Disable USART TX and RX DMA requests
    handle->CR3 &= ~(USART_CR3_DMAT | USART_CR3_DMAR);

    dma_stream_config_t tx_stream_config = {};
    tx_stream_config.deconfigure         = true;

    dma_stream_config_t rx_stream_config = {};
    rx_stream_config.deconfigure         = true;

    TRY(dma_configure_stream(tx_stream, &tx_stream_config));
    TRY(dma_configure_stream(rx_stream, &rx_stream_config));

    // Zero out all stored callbacks
    __disable_irq();
    memset(&s_dma_stream_ctx[idx], 0, sizeof(s_dma_stream_ctx[idx]));
    __enable_irq();

    return HAL_OK;
}


// Polling TX API
hal_err_t uart_transmit_byte(USART_TypeDef* handle, uint8_t byte) {
    if (handle == NULL) {
        return HAL_ERR_INVALID_ARG;
    }
    ENABLE_UART_TX();

    // Wait till the data register is empty
    while (!(handle->SR & USART_SR_TXE));
    handle->DR = byte;

    DISABLE_UART_TX();
    return HAL_OK;
}

hal_err_t uart_transmit_poll(USART_TypeDef* handle, const uint8_t* data, size_t size) {
    if (handle == NULL || data == NULL || size == 0) {
        return HAL_ERR_INVALID_ARG;
    }
    ENABLE_UART_TX();

    for (size_t i = 0; i < size; i++) {
        // Wait till the data register is empty
        while (!(handle->SR & USART_SR_TXE));
        handle->DR = data[i];
    }

    // Wait till all bytes have been fully transmitted
    while (!(handle->SR & USART_SR_TC));

    // Clar the status flags when done
    handle->SR &= ~(USART_SR_TC | USART_SR_TXE);

    DISABLE_UART_TX();
    return HAL_OK;
}


// DMA transfers API
hal_err_t uart_transmit_dma(USART_TypeDef* handle, const uint8_t* data, uint16_t size, dma_done_cb_t callback, void* arg) {
    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU || data == NULL || size == 0) {
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
        __disable_irq();
        s_dma_stream_ctx[idx].tx.callback = callback;
        s_dma_stream_ctx[idx].tx.arg      = arg;
        __enable_irq();
    }

    // Clear the status flags before starting
    handle->SR &= ~(USART_SR_TC | USART_SR_TXE);

    // Enable the DMA TX stream
    TRY(dma_enable_stream(stream));

    // Enable the UART TX peripheral
    ENABLE_UART_TX();

    return HAL_OK;
}

hal_err_t uart_receive_dma(USART_TypeDef* handle, uint8_t* data, uint16_t size, dma_done_cb_t callback, void* arg) {
    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU || data == NULL || size == 0) {
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
        __disable_irq();
        s_dma_stream_ctx[idx].rx.callback = callback;
        s_dma_stream_ctx[idx].rx.arg      = arg;
        __enable_irq();
    }

    // Clear the status flag before starting
    handle->SR &= ~USART_SR_RXNE;

    // Enable the DMA RX stream
    TRY(dma_enable_stream(stream));

    // Enable the UART RX peripheral
    ENABLE_UART_RX();

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
