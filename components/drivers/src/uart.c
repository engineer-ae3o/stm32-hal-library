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

#define ENABLE_UART_DMA_TX()                                                                                                                         \
    do {                                                                                                                                             \
        handle->CR3 |= USART_CR3_DMAT;                                                                                                               \
    } while (0)

#define DISABLE_UART_DMA_TX()                                                                                                                        \
    do {                                                                                                                                             \
        handle->CR3 &= ~USART_CR3_DMAT;                                                                                                              \
    } while (0)

#define ENABLE_UART_DMA_RX()                                                                                                                         \
    do {                                                                                                                                             \
        handle->CR3 |= USART_CR3_DMAR;                                                                                                               \
    } while (0)

#define DISABLE_UART_DMA_RX()                                                                                                                        \
    do {                                                                                                                                             \
        handle->CR3 &= ~USART_CR3_DMAR;                                                                                                              \
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
    handle->SR = ~(USART_SR_TC | USART_SR_TXE);

    // Transfers require us to poll on the TC flag
    // even after data has been shifted out

    // Poll till the TC bit has been set
    // Skip polling if an error has occurred
    if (ret == HAL_OK) {
        uint32_t timeout = TIMEOUT;
        while (!(handle->SR & USART_SR_TC) && (--timeout));
        if (timeout == 0) {
            ret = HAL_ERR_UART_TC_FAILED_TO_SET;
        }
    }

    DISABLE_UART_DMA_TX();
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
    handle->SR = ~USART_SR_RXNE;

    DISABLE_UART_DMA_RX();
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

    // Get the frequency of the bus clock on which the xcurrent uart peripheral instance lives on
    const uint32_t bus_clock_freq_hz = (handle == USART1 || handle == USART6) ? get_apb2_core_clock() : get_apb1_core_clock();

    // Baud rate generator: Get the UART bus divider from the target baud rate and bus clock frequency
    const float divisor = (float)bus_clock_freq_hz / (float)(config->baud_rate * config->over_sampling);

    // Get the mantissa and the fractional parts of the uart clock divider
    const uint16_t mantissa = (uint16_t)divisor;
    const uint16_t fraction = (uint16_t)(((divisor - (float)mantissa) * (float)config->over_sampling) + 0.5F);
    if (mantissa > 0xFFFUL || mantissa == 0 || fraction > 0xFUL) {
        return HAL_ERR_INVALID_ARG;
    }

    // Disable the UART peripheral before modifying its registers
    handle->CR1 &= ~(USART_CR1_UE | USART_CR1_TE | USART_CR1_RE);

    // 8 bit UART and parity bit disabled
    handle->CR1 &= ~(USART_CR1_M | USART_CR1_PCE);

    // Disable UART DMA requests by default
    handle->CR3 &= ~(USART_CR3_DMAT | USART_CR3_DMAR);

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

    // Configure the GPIO pins
    TRY(gpiox_clk_enable(config->tx_pin.port, true));
    TRY(gpio_set_alternate_function(config->tx_pin.port, config->tx_pin.pin, config->tx_pin.af));
    gpio_enable_pullups(config->tx_pin.port, config->tx_pin.pin, true);
    gpio_set_speed_mode(config->tx_pin.port, config->tx_pin.pin, GPIO_MEDIUM_SPEED);
    gpio_set_output_type(config->tx_pin.port, config->tx_pin.pin, GPIO_PUSH_PULL);

    TRY(gpiox_clk_enable(config->rx_pin.port, true));
    TRY(gpio_set_alternate_function(config->rx_pin.port, config->rx_pin.pin, config->rx_pin.af));
    gpio_enable_pullups(config->rx_pin.port, config->rx_pin.pin, true);
    gpio_set_speed_mode(config->rx_pin.port, config->rx_pin.pin, GPIO_MEDIUM_SPEED);
    gpio_set_output_type(config->rx_pin.port, config->rx_pin.pin, GPIO_PUSH_PULL);

    // Enable the UART peripheral
    handle->CR1 |= (USART_CR1_UE | USART_CR1_TE | USART_CR1_RE);
    return HAL_OK;
}

hal_err_t uart_deinit(USART_TypeDef* handle) {
    if (handle == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    // Disable the UART peripheral before disabling any other configuration
    handle->CR1 &= ~(USART_CR1_UE | USART_CR1_TE | USART_CR1_RE);

    handle->CR1 &= ~(USART_CR1_SBK | USART_CR1_RWU | USART_CR1_RE | USART_CR1_TE | USART_CR1_IDLEIE | USART_CR1_RXNEIE | USART_CR1_TCIE |
                     USART_CR1_TXEIE | USART_CR1_PEIE | USART_CR1_PS | USART_CR1_PCE | USART_CR1_WAKE | USART_CR1_M | USART_CR1_OVER8);
    handle->CR2 &= ~(USART_CR2_ADD | USART_CR2_LBDL | USART_CR2_LBDIE | USART_CR2_LBCL | USART_CR2_CPHA | USART_CR2_CPOL | USART_CR2_CLKEN |
                     USART_CR2_STOP | USART_CR2_LINEN);
    handle->CR3 &= ~(USART_CR3_EIE | USART_CR3_IREN | USART_CR3_IRLP | USART_CR3_HDSEL | USART_CR3_NACK | USART_CR3_SCEN | USART_CR3_DMAT |
                     USART_CR3_DMAR | USART_CR3_RTSE | USART_CR3_CTSE | USART_CR3_CTSIE | USART_CR3_ONEBIT);
    handle->BRR &= ~(USART_BRR_DIV_Fraction | USART_BRR_DIV_Mantissa);
    handle->SR = ~(USART_SR_TC | USART_SR_TXE | USART_SR_RXNE);

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
        .direction       = DMA_DIR_M2P,
        .per_data_size   = DMA_SIZE_BYTE,
        .mem_data_size   = DMA_SIZE_BYTE,
        .circular_mode   = DMA_MODE_ONESHOT,
        .flow_controller = DMA_FLOW_CONTROLLER_DMA,

        .buffer_size       = 0,
        .channel           = s_uart_dma_map[idx].tx.channel,
        .nvic_irq_priority = UART_DMA_NVIC_IRQ_PRIORITY,

        .per_addr  = &handle->DR,
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
        .direction       = DMA_DIR_P2M,
        .per_data_size   = DMA_SIZE_BYTE,
        .mem_data_size   = DMA_SIZE_BYTE,
        .circular_mode   = DMA_MODE_ONESHOT,
        .flow_controller = DMA_FLOW_CONTROLLER_DMA,

        .buffer_size       = 0,
        .channel           = s_uart_dma_map[idx].rx.channel,
        .nvic_irq_priority = UART_DMA_NVIC_IRQ_PRIORITY,

        .per_addr  = &handle->DR,
        .mem_buf_0 = NULL,
        .mem_buf_1 = NULL,
    };

    TRY(dma_configure_stream(tx_stream, &tx_stream_config));
    TRY(dma_configure_stream(rx_stream, &rx_stream_config));

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

    dma_stream_config_t tx_stream_config;
    tx_stream_config.deconfigure = true;

    dma_stream_config_t rx_stream_config;
    rx_stream_config.deconfigure = true;

    TRY(dma_configure_stream(tx_stream, &tx_stream_config));
    TRY(dma_configure_stream(rx_stream, &rx_stream_config));

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

    // Wait till the data register is empty
    uint32_t timeout = TIMEOUT;
    while (!(handle->SR & USART_SR_TXE) && --timeout);
    if (!(handle->SR & USART_SR_TXE) || (timeout == 0)) {
        return HAL_ERR_TIMEOUT;
    }

    handle->DR = byte;
    return HAL_OK;
}

hal_err_t uart_transmit_poll(USART_TypeDef* handle, const uint8_t* data, size_t size) {
    if (handle == NULL || data == NULL || size == 0) {
        return HAL_ERR_INVALID_ARG;
    }

    for (size_t i = 0; i < size; i++) {
        uint32_t timeout = TIMEOUT;
        while (!(handle->SR & USART_SR_TXE) && --timeout);
        if (!(handle->SR & USART_SR_TXE) || (timeout == 0)) {
            return HAL_ERR_TIMEOUT;
        }
        handle->DR = data[i];
    }

    // Wait till all bytes have been fully transmitted and clear the status flags
    uint32_t timeout = TIMEOUT;
    while (!(handle->SR & USART_SR_TC) && --timeout);
    if (!(handle->SR & USART_SR_TC) || (timeout == 0)) {
        return HAL_ERR_TIMEOUT;
    }
    handle->SR = ~USART_SR_TC;

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

    if (stream->CR & DMA_SxCR_EN) {
        return HAL_ERR_INVALID_STATE;
    }

    dma_set_addresses(stream, &handle->DR, data, NULL);
    dma_set_trans_length(stream, size);

    __disable_irq();
    s_dma_stream_ctx[idx].tx.callback = callback;
    s_dma_stream_ctx[idx].tx.arg      = arg;
    __enable_irq();

    // Clear the status flag before starting
    handle->SR = ~USART_SR_TC;

    // Enable the DMA stream, UART DMA requests and then the UART TX peripheral
    TRY(dma_enable_stream(stream));
    ENABLE_UART_DMA_TX();

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

    if (stream->CR & DMA_SxCR_EN) {
        return HAL_ERR_INVALID_STATE;
    }

    dma_set_addresses(stream, &handle->DR, data, NULL);
    dma_set_trans_length(stream, size);

    __disable_irq();
    s_dma_stream_ctx[idx].rx.callback = callback;
    s_dma_stream_ctx[idx].rx.arg      = arg;
    __enable_irq();

    // Clear the status flag before starting
    handle->SR = ~USART_SR_RXNE;

    // Enable the DMA stream, UART DMA requests and then the UART RX peripheral
    TRY(dma_enable_stream(stream));
    ENABLE_UART_DMA_RX();

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
