#include "stm32f411xe.h"
#include "drivers/gpio.h"
#include "utils/common.h"
#include "drivers/spi.h"
#include "drivers/i2s.h"
#include "utils/err.h"

#include <stddef.h>


// Audio PLL check. To use a different PLL clock speed
// provide a corresponding prescaler table, update the
// macro in "common.h" and this check
#if AUDIO_PLL_HZ == 76'800'000L
#define prescaler_table (s_prescaler_table_76_8mhz)
#else
#error "No pll table defined for used I2S PLL frequency"
#endif


// Clock prescaler table
typedef struct {
    uint16_t prescaler;
    uint16_t prescaler_with_mck;
} prescaler_mck_t;

// The table assumes an audio input PLL of 76.8MHz
// Modify that and everything breaks. It also encodes
// the bit for ODD and the SPI_I2SPR_MCKOE bit
// TODO: Compute the prescaler table
static const prescaler_mck_t s_prescaler_table_76_8mhz[] = {
    [I2S_FREQ_8kHz]   = {.prescaler = 0, .prescaler_with_mck = 0},
    [I2S_FREQ_16kHz]  = {.prescaler = 0, .prescaler_with_mck = 0},
    [I2S_FREQ_22kHz]  = {.prescaler = 0, .prescaler_with_mck = 0},
    [I2S_FREQ_32kHz]  = {.prescaler = 0, .prescaler_with_mck = 0},
    [I2S_FREQ_44kHz]  = {.prescaler = 0, .prescaler_with_mck = 0},
    [I2S_FREQ_48kHz]  = {.prescaler = 0, .prescaler_with_mck = 0},
    [I2S_FREQ_96kHz]  = {.prescaler = 0, .prescaler_with_mck = 0},
    [I2S_FREQ_192kHz] = {.prescaler = 0, .prescaler_with_mck = 0},
};

// Helper
[[__gnu__::__always_inline__]] static inline uint8_t get_index(const I2S_TypeDef* handle) {
    if (handle == I2S1) {
        return 0U;
    } else if (handle == I2S2) {
        return 1U;
    } else if (handle == I2S3) {
        return 2U;
    } else if (handle == I2S4) {
        return 3U;
    } else if (handle == I2S5) {
        return 4U;
    } else {
        return 0xFFU;
    }
}

// Defined in the SPI driver. Used to post DMA events or get info from the SPI driver since the I2S peripheral
// shares the same hardware block as the SPI peripheral, and consequently, share the same DMA streams.
// All the NVIC interrupt handlers are managed by the SPI driver.
extern hal_err_t spi_master_get_dma_stream_map(dma_stream_map_t* map, uint32_t idx);
extern hal_err_t spi_master_register_callback(dma_done_cb_t callback, void* arg, uint8_t idx, bool is_tx);


// General API
void i2s_pll_init_76_8mhz(void) {
    // Disable the audio PLL before setup
    RCC->CR &= ~RCC_CR_PLLI2SON;

#ifdef USE_HSE
    const uint8_t clock_mhz = HSE_VALUE_MHZ;
#else
    const uint8_t clock_mhz = HSI_VALUE_MHZ;
#endif

    // Divide the HSE or HSI clock by its value in MHz to get a Vco of 1MHz regardless of its value
    RCC->PLLI2SCFGR &= ~(RCC_PLLI2SCFGR_PLLI2SM | RCC_PLLI2SCFGR_PLLI2SN | RCC_PLLI2SCFGR_PLLI2SR);
    RCC->PLLI2SCFGR |= (clock_mhz << RCC_PLLI2SCFGR_PLLI2SM_Pos) | // PLLI2SM of the main PLL: Divides either the HSE or the HSI to get a 1MHz Vco
                       (384UL << RCC_PLLI2SCFGR_PLLI2SN_Pos) |     // PLLI2SN of 384: Multiplies Vco by 384 to get 384MHz
                       (5UL << RCC_PLLI2SCFGR_PLLI2SR_Pos);        // PLLI2SR of 5: Divides the 384MHz Vco by 5 to get us 76.8MHz

    // Enable the audio PLL
    RCC->CR |= RCC_CR_PLLI2SON;
    while (!(RCC->CR & RCC_CR_PLLI2SRDY));

    // Use the internal I2S PLL for the I2S peripherals
    RCC->CFGR &= ~RCC_CFGR_I2SSRC;
}

hal_err_t i2sx_clk_enable(I2S_TypeDef* handle, bool enable) {
    if (enable) {
        if (handle == I2S1) {
            RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
        } else if (handle == I2S2) {
            RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
        } else if (handle == I2S3) {
            RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;
        } else if (handle == I2S4) {
            RCC->APB2ENR |= RCC_APB2ENR_SPI4EN;
        } else if (handle == I2S5) {
            RCC->APB2ENR |= RCC_APB2ENR_SPI5EN;
        } else {
            return HAL_ERR_INVALID_ARG;
        }

    } else {
        if (handle == I2S1) {
            RCC->APB2ENR &= ~RCC_APB2ENR_SPI1EN;
        } else if (handle == I2S2) {
            RCC->APB1ENR &= ~RCC_APB1ENR_SPI2EN;
        } else if (handle == I2S3) {
            RCC->APB1ENR &= ~RCC_APB1ENR_SPI3EN;
        } else if (handle == I2S4) {
            RCC->APB2ENR &= ~RCC_APB2ENR_SPI4EN;
        } else if (handle == I2S5) {
            RCC->APB2ENR &= ~RCC_APB2ENR_SPI5EN;
        } else {
            return HAL_ERR_INVALID_ARG;
        }
    }

    __DSB();
    return HAL_OK;
}

hal_err_t i2s_master_init(I2S_TypeDef* handle, const i2s_master_config_t* config) {
    if (handle == NULL || config == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    // Configure the GPIO pins
    TRY(gpiox_clk_enable(config->gpio_port, true));

    // Alternate function value selection for the GPIOs
    uint8_t alt_val = 0;
    if ((handle == I2S1) || (handle == I2S2)) {
        alt_val = 5U;
    } else if (handle == I2S3) {
        alt_val = (config->gpio_port == GPIOD) ? 5U : 6U;
    } else if (handle == I2S4) {
        alt_val = (config->gpio_port == GPIOE) ? 5U : 6U;
    } else if (handle == I2S5) {
        alt_val = 6U;
    } else {
        return HAL_ERR_INVALID_ARG;
    }

    // MCK
    if (config->use_mck) {
        TRY(gpio_set_alternate_function(config->gpio_port, config->mck_pin, alt_val));
        gpio_enable_pullup(config->gpio_port, config->mck_pin, true);
        gpio_set_speed_mode(config->gpio_port, config->mck_pin, GPIO_MEDIUM_SPEED);
        gpio_set_output_type(config->gpio_port, config->mck_pin, GPIO_PUSH_PULL);
    }

    // SD pin: Can be input or output
    TRY(gpio_set_alternate_function(config->gpio_port, config->sd_pin, alt_val));
    gpio_enable_pullup(config->gpio_port, config->sd_pin, true);
    gpio_set_speed_mode(config->gpio_port, config->sd_pin, GPIO_MEDIUM_SPEED);
    // Only set output type as push pull when we are driving, that is, in TX mode
    if (config->dir == I2S_DIR_HALF_DUPLEX_TX) {
        gpio_set_output_type(config->gpio_port, config->sd_pin, GPIO_PUSH_PULL);
    }

    // WS pin
    TRY(gpio_set_alternate_function(config->gpio_port, config->ws_pin, alt_val));
    gpio_enable_pullup(config->gpio_port, config->ws_pin, true);
    gpio_set_speed_mode(config->gpio_port, config->ws_pin, GPIO_MEDIUM_SPEED);
    gpio_set_output_type(config->gpio_port, config->ws_pin, GPIO_PUSH_PULL);

    // SCK pin
    TRY(gpio_set_alternate_function(config->gpio_port, config->sck_pin, alt_val));
    gpio_enable_pullup(config->gpio_port, config->sck_pin, true);
    gpio_set_speed_mode(config->gpio_port, config->sck_pin, GPIO_MEDIUM_SPEED);
    gpio_set_output_type(config->gpio_port, config->sck_pin, GPIO_PUSH_PULL);

    // Disable the SPI and I2S peripheral before modifying its registers
    handle->CR1 &= ~SPI_CR1_SPE;
    handle->I2SCFGR &= ~SPI_I2SCFGR_I2SE;

    // I2S mode
    handle->I2SCFGR |= SPI_I2SCFGR_I2SMOD;

    // Set the clock prescaler
    const uint32_t prescaler = (config->use_mck) ? prescaler_table[config->freq].prescaler_with_mck : prescaler_table[config->freq].prescaler;
    handle->I2SPR &= ~(SPI_I2SPR_I2SDIV | SPI_I2SPR_ODD | SPI_I2SPR_MCKOE);
    handle->I2SPR |= ((prescaler & 0x3FFUL) << SPI_I2SPR_I2SDIV_Pos);

    handle->I2SCFGR &= ~(SPI_I2SCFGR_I2SCFG | SPI_I2SCFGR_CKPOL | SPI_I2SCFGR_CHLEN | SPI_I2SCFGR_I2SSTD | SPI_I2SCFGR_DATLEN);

    // Get frame size: It can only be 16 bits when the data is 16 bits
    const uint32_t frame_size_mask = (config->frame == I2S_DATA_16_BITS_FRAME_16_BITS) ? 0 : SPI_I2SCFGR_CHLEN;
    const uint32_t cpol_mask       = (config->cpol) ? SPI_I2SCFGR_CKPOL : 0;

    // Apply user settings
    handle->I2SCFGR |= (((uint32_t)config->dir << SPI_I2SCFGR_I2SCFG_Pos) |   // Direction: TX or RX in master mode
                        ((uint32_t)config->mode << SPI_I2SCFGR_I2SSTD_Pos) |  // I2S standard: Phillips, left or right justified
                        ((uint32_t)config->frame << SPI_I2SCFGR_DATLEN_Pos) | // Data length: 16, 24 or 32 bits
                        frame_size_mask |                                     // Frame size: 16 or 32 bits
                        cpol_mask);                                           // Clock polarity

    return HAL_OK;
}

hal_err_t i2s_master_enable(I2S_TypeDef* handle, bool enable) {
    if (handle == NULL) {
        return HAL_ERR_INVALID_ARG;
    }
    if (enable) {
        handle->I2SCFGR |= SPI_I2SCFGR_I2SE;
    } else {
        handle->I2SCFGR &= ~SPI_I2SCFGR_I2SE;
    }
    return HAL_OK;
}

hal_err_t i2s_master_dma_init(I2S_TypeDef* handle, dma_priority_t priority) {
    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU) {
        return HAL_ERR_INVALID_ARG;
    }

    // Get the DMA stream mapping to the corresponding I2S handle
    dma_stream_map_t dma_map;
    TRY(spi_master_get_dma_stream_map(&dma_map, idx));

    // TX mapping
    DMA_Stream_TypeDef* tx_stream  = dma_map.tx.stream;
    const uint8_t       tx_channel = dma_map.tx.channel;

    // RX mapping
    DMA_Stream_TypeDef* rx_stream  = dma_map.rx.stream;
    const uint8_t       rx_channel = dma_map.rx.channel;

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
        .channel           = tx_channel,
        .nvic_irq_priority = UART_DMA_NVIC_IRQ_PRIORITY,

        .per_addr  = NULL,
        .mem_buf_0 = NULL,
        .mem_buf_1 = NULL,

    };
    TRY(dma_configure_stream(tx_stream, &tx_stream_config));

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
        .channel           = rx_channel,
        .nvic_irq_priority = UART_DMA_NVIC_IRQ_PRIORITY,

        .per_addr  = NULL,
        .mem_buf_0 = NULL,
        .mem_buf_1 = NULL,

    };
    TRY(dma_configure_stream(rx_stream, &rx_stream_config));

    // TX stream configuration
    TRY(dma_disable_stream(tx_stream));
    dma_set_channel(tx_stream, tx_channel);
    dma_set_direct_mode(tx_stream, true);
    dma_set_direction(tx_stream, DMA_DIR_M_P);
    dma_set_flow_controller(tx_stream, true);
    dma_set_stream_priority(tx_stream, priority);
    dma_enable_circm_dbm(tx_stream, false, false);
    dma_set_increment(tx_stream, false, true);
    dma_enable_irqs(tx_stream, true, true, false, true);
    dma_set_per_mem_size(tx_stream, DMA_SIZE_HWORD, DMA_SIZE_HWORD);

    // RX stream configuration
    TRY(dma_disable_stream(rx_stream));
    dma_set_channel(rx_stream, rx_channel);
    dma_set_direct_mode(rx_stream, true);
    dma_set_direction(rx_stream, DMA_DIR_P_M);
    dma_set_flow_controller(rx_stream, true);
    dma_set_stream_priority(rx_stream, priority);
    dma_enable_circm_dbm(rx_stream, false, false);
    dma_set_increment(rx_stream, false, true);
    dma_enable_irqs(rx_stream, true, true, false, true);
    dma_set_per_mem_size(rx_stream, DMA_SIZE_HWORD, DMA_SIZE_HWORD);

    // Enable I2S requests to DMA
    handle->CR2 |= (SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);

    return HAL_OK;
}


// DMA transfers API
hal_err_t i2s_master_transmit(I2S_TypeDef* handle, const void* buf, uint16_t size, dma_done_cb_t callback, void* arg) {
    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU) {
        return HAL_ERR_INVALID_ARG;
    }

    if (buf == NULL || size == 0) {
        return HAL_ERR_INVALID_ARG;
    }

    // Get the DMA stream mapping to the corresponding I2S handle
    dma_stream_map_t dma_map;
    TRY(spi_master_get_dma_stream_map(&dma_map, idx));

    DMA_Stream_TypeDef* stream = dma_map.tx.stream;
    if (stream == NULL) {
        return HAL_ERR_NOT_SUPPORTED;
    }

    // If the data is 24 or 32 bits, we need two DMA transfers
    // Note that if the DATLEN bits are 0b00, that means 16 bit
    // data; 0b01 for 24 and 0b10 for 32. So a non zero value
    // from the DATLEN bits implies a transfer greater than 16 bits
    size = (handle->I2SCFGR & SPI_I2SCFGR_DATLEN) ? (size * 2) : size;

    // Set the memory addresses and length
    dma_set_addresses(stream, &handle->DR, buf, NULL);
    dma_set_trans_length(stream, size);

    // Save the user passed callback
    if (callback) {
        TRY(spi_master_register_callback(callback, arg, idx, true));
    }

    // Enable DMA TX stream
    return dma_enable_stream(stream);
}

hal_err_t i2s_master_receive(I2S_TypeDef* handle, void* buf, uint16_t size, dma_done_cb_t callback, void* arg) {
    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU) {
        return HAL_ERR_INVALID_ARG;
    }

    if (buf == NULL || size == 0) {
        return HAL_ERR_INVALID_ARG;
    }

    // Get the DMA stream mapping to the corresponding I2S handle
    dma_stream_map_t dma_map;
    TRY(spi_master_get_dma_stream_map(&dma_map, idx));

    DMA_Stream_TypeDef* stream = dma_map.rx.stream;
    if (stream == NULL) {
        return HAL_ERR_NOT_SUPPORTED;
    }

    size = (handle->I2SCFGR & SPI_I2SCFGR_DATLEN) ? (size * 2) : size;

    // Set the memory addresses and length
    dma_set_addresses(stream, &handle->DR, buf, NULL);
    dma_set_trans_length(stream, size);

    // Save the user passed callback
    if (callback) {
        TRY(spi_master_register_callback(callback, arg, idx, false));
    }

    // Enable DMA RX stream
    return dma_enable_stream(stream);
}


// Double buffering API
hal_err_t i2s_master_dbm_init(I2S_TypeDef* handle, void* buf_a, void* buf_b, uint16_t size, dma_done_cb_t callback, void* arg) {
    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU) {
        return HAL_ERR_INVALID_ARG;
    }

    if (buf_a == NULL || buf_b == NULL || size == 0) {
        return HAL_ERR_INVALID_ARG;
    }

    // Get the DMA stream mapping to the corresponding I2S handle
    dma_stream_map_t dma_map;
    TRY(spi_master_get_dma_stream_map(&dma_map, idx));

    DMA_Stream_TypeDef* stream = dma_map.rx.stream;
    if (stream == NULL) {
        return HAL_ERR_NOT_SUPPORTED;
    }

    // Get the size
    size = (handle->I2SCFGR & SPI_I2SCFGR_DATLEN) ? (size * 2) : size;

    TRY(dma_disable_stream(stream));
    dma_enable_circm_dbm(stream, true, true);
    dma_set_addresses(stream, &handle->DR, buf_a, buf_b);
    dma_set_trans_length(stream, size);

    // Save the user passed callback
    if (callback) {
        TRY(spi_master_register_callback(callback, arg, idx, false));
    }

    return HAL_OK;
}

hal_err_t i2s_master_dbm_deinit(I2S_TypeDef* handle) {
    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU) {
        return HAL_ERR_INVALID_ARG;
    }

    // Get the DMA stream mapping to the corresponding I2S handle
    dma_stream_map_t dma_map;
    TRY(spi_master_get_dma_stream_map(&dma_map, idx));

    DMA_Stream_TypeDef* stream = dma_map.rx.stream;
    if (stream == NULL) {
        return HAL_ERR_NOT_SUPPORTED;
    }

    TRY(dma_disable_stream(stream));
    dma_enable_circm_dbm(stream, false, false);
    dma_set_trans_length(stream, 0);

    return HAL_OK;
}

hal_err_t i2s_master_dbm_start(I2S_TypeDef* handle) {
    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU) {
        return HAL_ERR_INVALID_ARG;
    }
    dma_stream_map_t dma_map;
    TRY(spi_master_get_dma_stream_map(&dma_map, idx));
    return dma_enable_stream(dma_map.rx.stream);
}

hal_err_t i2s_master_dbm_stop(I2S_TypeDef* handle) {
    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU) {
        return HAL_ERR_INVALID_ARG;
    }
    dma_stream_map_t dma_map;
    TRY(spi_master_get_dma_stream_map(&dma_map, idx));
    return dma_disable_stream(dma_map.rx.stream);
}

uint8_t i2s_master_dbm_get_filled_buffer(I2S_TypeDef* handle) {
    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU) {
        return 0xFFU;
    }


    // Get the DMA stream mapping to the corresponding I2S handle
    dma_stream_map_t dma_map;
    TRY(spi_master_get_dma_stream_map(&dma_map, idx));

    DMA_Stream_TypeDef* stream = dma_map.rx.stream;
    if (stream == NULL) {
        return HAL_ERR_NOT_SUPPORTED;
    }

    // CT represents the active buffer, that is, the
    // buffer currently in use by the dma controller.
    // 0 represents buf_a, and 1 buf_b. If the bit
    // is 0, that means buf_a is currently being used
    // by the DMA controller buf_b is filled and free
    return (stream->CR & DMA_SxCR_CT) ? 0 : 1;
}
