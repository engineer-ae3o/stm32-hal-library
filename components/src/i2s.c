#include "stm32f411xe.h"
#include "spi.h"
#include "i2s.h"
#include "gpio.h"

#include <stddef.h>


// Audio PLL check. To use a different PLL clock speed
// provide a corresponding prescaler table, update the
// #define in "common.h" and update this check
#if AUDIO_PLL_HZ == 76'800'000UL
    #define prescaler_table s_prescaler_table_76_8mhz
#else 
    #error "No pll table defined for used frequency"
#endif

// Clock prescaler table
typedef struct {
    uint16_t prescaler;
    uint16_t prescaler_with_mck;
} prescaler_mck_t;

// The table assumes an audio input PLL of 76.8MHz
// Modify that and everything breaks. It also encodes
// the bit for ODD and the SPI_I2SPR_MCKOE bit
static const prescaler_mck_t s_prescaler_table_76_8mhz[] = {
    [(uint8_t)I2S_FREQ_8kHz]   = { .prescaler = 0U, .prescaler_with_mck = 0U },
    [(uint8_t)I2S_FREQ_16kHz]  = { .prescaler = 0U, .prescaler_with_mck = 0U },
    [(uint8_t)I2S_FREQ_22kHz]  = { .prescaler = 0U, .prescaler_with_mck = 0U },
    [(uint8_t)I2S_FREQ_32kHz]  = { .prescaler = 0U, .prescaler_with_mck = 0U },
    [(uint8_t)I2S_FREQ_44kHz]  = { .prescaler = 0U, .prescaler_with_mck = 0U },
    [(uint8_t)I2S_FREQ_48kHz]  = { .prescaler = 0U, .prescaler_with_mck = 0U },
    [(uint8_t)I2S_FREQ_96kHz]  = { .prescaler = 0U, .prescaler_with_mck = 0U },
    [(uint8_t)I2S_FREQ_192kHz] = { .prescaler = 0U, .prescaler_with_mck = 0U }
};

// Mapping for the DMA channels for the 5 I2S channels
static const dma_stream_map_t s_i2s_dma_map[5] = {
    // I2S1: DMA not supported: Not enough streams to go round other peripherals
    {
        .tx = { .controller = NULL, .stream = NULL, .stream_no = 0, .irq_type = 0, .channel = 0 },
        .rx = { .controller = NULL, .stream = NULL, .stream_no = 0, .irq_type = 0, .channel = 0 }
    },
    // I2S2
    {
        .tx = { .controller = DMA1, .stream = DMA1_Stream4, .stream_no = 4, .irq_type = DMA1_Stream4_IRQn, .channel = 0 },
        .rx = { .controller = DMA1, .stream = DMA1_Stream3, .stream_no = 3, .irq_type = DMA1_Stream3_IRQn, .channel = 0 }
    },
    // I2S3
    {
        .tx = { .controller = DMA1, .stream = DMA1_Stream7, .stream_no = 7, .irq_type = DMA1_Stream7_IRQn, .channel = 0 },
        .rx = { .controller = DMA1, .stream = DMA1_Stream2, .stream_no = 2, .irq_type = DMA1_Stream2_IRQn, .channel = 0 }
    },
    // I2S4
    {
        .tx = { .controller = DMA2, .stream = DMA2_Stream1, .stream_no = 1, .irq_type = DMA2_Stream1_IRQn, .channel = 4 },
        .rx = { .controller = DMA2, .stream = DMA2_Stream4, .stream_no = 4, .irq_type = DMA2_Stream4_IRQn, .channel = 4 }
    },
    // I2S5
    {
        .tx = { .controller = DMA2, .stream = DMA2_Stream6, .stream_no = 6, .irq_type = DMA2_Stream6_IRQn, .channel = 7 },
        .rx = { .controller = DMA2, .stream = DMA2_Stream3, .stream_no = 3, .irq_type = DMA2_Stream3_IRQn, .channel = 2 }
    }
};

// Helpers
static inline uint8_t get_index(const I2S_TypeDef* handle) {
    if      (handle == I2S1) return 0U;
    else if (handle == I2S2) return 1U;
    else if (handle == I2S3) return 2U;
    else if (handle == I2S4) return 3U;
    else if (handle == I2S5) return 4U;
    else                     return 0xFFU;
}


// Public API
void i2s_pll_init(void) {
    
    // Disable the audio PLL before setup
    RCC->CR &= ~RCC_CR_PLLI2SON;
    
#ifdef USE_HSE
    const uint8_t clock_mhz = HSE_VALUE_MHZ;
#else
    const uint8_t clock_mhz = HSI_VALUE_MHZ;
#endif
    
    // Divide the HSE or HSI clock by its value in MHz to get a Vco of 1MHz regardless of its value
    RCC->PLLI2SCFGR &= ~(RCC_PLLI2SCFGR_PLLI2SM | RCC_PLLI2SCFGR_PLLI2SN | RCC_PLLI2SCFGR_PLLI2SR);
    RCC->PLLI2SCFGR |= (clock_mhz << RCC_PLLI2SCFGR_PLLI2SM_Pos)  | // PLLI2SM of val: Divides HSE or HSI by val to get a 1MHz Vco
                       (384UL << RCC_PLLI2SCFGR_PLLI2SN_Pos)      | // PLLI2SN of 384: Multiplies Vco by 384 to get 384MHz
                       (5UL << RCC_PLLI2SCFGR_PLLI2SR_Pos);         // PLLI2SR of 5: Divides the 384MHz Vco by 5 to get us 76.8MHz
    
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
            return HAL_INVALID_ARG;
        }
        goto done;
    }
    
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
        return HAL_INVALID_ARG;
    }
    
done:
    __DSB();
    return HAL_OK;
}

hal_err_t i2s_master_init(I2S_TypeDef* handle, const i2s_master_config_t* config) {
    // Configure GPIO pins
    hal_err_t ret = gpiox_clk_enable(config->gpio_port, true);
    if (ret != HAL_OK) return ret;

    // Alternate function value selection for the GPIOs
    uint8_t alt_val = 0;
    if ((handle == I2S1) || (handle == I2S2)) alt_val = 5U;
    else if (handle == I2S3) alt_val = (config->gpio_port == GPIOD) ? 5U : 6U;
    else if (handle == I2S4) alt_val = (config->gpio_port == GPIOE) ? 5U : 6U;
    else if (handle == I2S5) alt_val = 6U;
    else return HAL_INVALID_ARG;
    
    // MCK
    if (config->use_mck) {
        ret = gpio_set_alternate_function(config->gpio_port, config->mck, alt_val);
        if (ret != HAL_OK) return ret;
        gpio_enable_pullup(config->gpio_port, config->mck, true);
        gpio_set_speed_mode(config->gpio_port, config->mck, GPIO_MEDIUM_SPEED);
        gpio_set_output_type(config->gpio_port, config->mck, GPIO_PUSH_PULL);
    }

    // SD pin: Can be input or output
    ret = gpio_set_alternate_function(config->gpio_port, config->sd, alt_val);
    if (ret != HAL_OK) return ret;
    gpio_enable_pullup(config->gpio_port, config->sd, true);
    gpio_set_speed_mode(config->gpio_port, config->sd, GPIO_MEDIUM_SPEED);
    // Only set output type as push pull when we are driving, that is, in TX mode
    if (config->dir == I2S_DIR_HALF_DUPLEX_TX) gpio_set_output_type(config->gpio_port, config->sd, GPIO_PUSH_PULL);
    
    // WS pin
    ret = gpio_set_alternate_function(config->gpio_port, config->ws, alt_val);
    if (ret != HAL_OK) return ret;
    gpio_enable_pullup(config->gpio_port, config->ws, true);
    gpio_set_speed_mode(config->gpio_port, config->ws, GPIO_MEDIUM_SPEED);
    gpio_set_output_type(config->gpio_port, config->ws, GPIO_PUSH_PULL);
    
    // SCK pin
    ret = gpio_set_alternate_function(config->gpio_port, config->sck, alt_val);
    if (ret != HAL_OK) return ret;
    gpio_enable_pullup(config->gpio_port, config->sck, true);
    gpio_set_speed_mode(config->gpio_port, config->sck, GPIO_MEDIUM_SPEED);
    gpio_set_output_type(config->gpio_port, config->sck, GPIO_PUSH_PULL);
    
    // Disable the SPI and I2S peripheral before modifying its registers
    handle->CR1 &= ~SPI_CR1_SPE;
    handle->I2SCFGR &= ~SPI_I2SCFGR_I2SE;

    // I2S mode
    handle->I2SCFGR |= SPI_I2SCFGR_I2SMOD;
    
    // Set the clock prescaler
    const uint32_t prescaler = (config->use_mck) ? prescaler_table[config->freq].prescaler_with_mck 
                                                 : prescaler_table[config->freq].prescaler;
    handle->I2SPR &= ~(SPI_I2SPR_I2SDIV | SPI_I2SPR_ODD | SPI_I2SPR_MCKOE);
    handle->I2SPR |= ((prescaler & 0x3FFUL) << SPI_I2SPR_I2SDIV_Pos);

    // Clear used bits once in one read-modify-write op
    handle->I2SCFGR &= ~(SPI_I2SCFGR_I2SCFG |
                         SPI_I2SCFGR_CKPOL  |
                         SPI_I2SCFGR_CHLEN  |
                         SPI_I2SCFGR_I2SSTD |
                         SPI_I2SCFGR_DATLEN);
    
    // Get frame size: It can only be 16 bits when the data is 16 bits
    const uint32_t frame_size_mask = (config->data_frame == I2S_DATA_16_BITS_FRAME_16_BITS) ? 0 : SPI_I2SCFGR_CHLEN;
    const uint32_t cpol_mask = (config->cpol) ? SPI_I2SCFGR_CKPOL : 0;
    
    // Apply user settings
    handle->I2SCFGR |= (((uint32_t)config->dir << SPI_I2SCFGR_I2SCFG_Pos)        | // Direction: TX or RX in master mode
                        ((uint32_t)config->mode << SPI_I2SCFGR_I2SSTD_Pos)       | // I2S standard: Phillips, left or right justified
                        ((uint32_t)config->data_frame << SPI_I2SCFGR_DATLEN_Pos) | // Data length: 16, 24 or 32 bits
                        frame_size_mask                                          | // Frame size: 16 or 32 bits
                        cpol_mask);                                                // Clock polarity
    
    return HAL_OK;
}

void i2s_master_enable(I2S_TypeDef* handle, bool enable) {
    if (enable) handle->I2SCFGR |= SPI_I2SCFGR_I2SE;
    else        handle->I2SCFGR &= ~SPI_I2SCFGR_I2SE;
}

hal_err_t i2s_master_dma_init(I2S_TypeDef* handle) {
    
    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU) return HAL_INVALID_ARG;
    
    // TX mapping
    DMA_TypeDef* tx_controller = s_i2s_dma_map[idx].tx.controller;
    DMA_Stream_TypeDef* tx_stream = s_i2s_dma_map[idx].tx.stream;
    uint8_t tx_channel = s_i2s_dma_map[idx].tx.channel;

    // RX mapping
    DMA_TypeDef* rx_controller = s_i2s_dma_map[idx].rx.controller;
    DMA_Stream_TypeDef* rx_stream = s_i2s_dma_map[idx].rx.stream;
    uint8_t rx_channel = s_i2s_dma_map[idx].rx.channel;
    
    hal_err_t ret = dmax_clk_enable(tx_controller, true);
    if (ret != HAL_OK) return ret;
    ret = dmax_clk_enable(rx_controller, true);
    if (ret != HAL_OK) return ret;
    
    // Disable DMA stream before configuring
    ret = dma_disable_stream(tx_stream);
    if (ret != HAL_OK) return ret;
    ret = dma_disable_stream(rx_stream);
    if (ret != HAL_OK) return ret;

    // TX stream configuration
    dma_clear_flags(tx_controller, s_i2s_dma_map[idx].tx.stream_no);
    dma_set_channel(tx_stream, tx_channel);
    dma_set_direct_mode(tx_stream, true);
    dma_set_direction(tx_stream, DMA_DIR_M_P);
    dma_set_flow_controller(tx_stream, true);
    dma_set_stream_priority(tx_stream, DMA_PRIORITY_LOW);
    dma_enable_circm_dbm(tx_stream, false, false);
    dma_set_increment(tx_stream, false, true);
    dma_enable_irqs(tx_stream, true, true, false, true);
    dma_set_addresses(tx_stream, &handle->DR, NULL, NULL);
    dma_set_per_mem_size(tx_stream, DMA_SIZE_HWORD, DMA_SIZE_HWORD);
    
    // RX stream configuration
    dma_clear_flags(rx_controller, s_i2s_dma_map[idx].rx.stream_no);
    dma_set_channel(rx_stream, rx_channel);
    dma_set_direct_mode(rx_stream, true);
    dma_set_direction(rx_stream, DMA_DIR_P_M);
    dma_set_flow_controller(rx_stream, true);
    dma_set_stream_priority(rx_stream, DMA_PRIORITY_LOW);
    dma_enable_circm_dbm(rx_stream, false, false);
    dma_set_increment(rx_stream, false, true);
    dma_enable_irqs(rx_stream, true, true, false, true);
    dma_set_addresses(rx_stream, &handle->DR, NULL, NULL);
    dma_set_per_mem_size(rx_stream, DMA_SIZE_HWORD, DMA_SIZE_HWORD);
    
    // Enable I2S requests to DMA
    handle->CR2 |= (SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);
    
    // Enable DMA stream interrupts
    NVIC_EnableIRQ(s_i2s_dma_map[idx].tx.irq_type);
    NVIC_SetPriority(s_i2s_dma_map[idx].tx.irq_type, I2S_DMA_NVIC_IRQ_PRIORITY);

    NVIC_EnableIRQ(s_i2s_dma_map[idx].rx.irq_type);
    NVIC_SetPriority(s_i2s_dma_map[idx].rx.irq_type, I2S_DMA_NVIC_IRQ_PRIORITY);

    return HAL_OK;
}

hal_err_t i2s_master_get_dma_stream(I2S_TypeDef* handle, DMA_Stream_TypeDef** tx, DMA_Stream_TypeDef** rx) {
    
    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU) return HAL_INVALID_ARG;

    *tx = s_i2s_dma_map[idx].tx.stream;
    *rx = s_i2s_dma_map[idx].rx.stream;
    
    return HAL_OK;
}

hal_err_t i2s_master_transmit(I2S_TypeDef* handle, const void* buf, uint16_t len, dma_trans_done_cb_t callback, void* arg) {
    
    // Get index for DMA stream mapping
    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU) return HAL_INVALID_ARG;
    
    // Save user passed callback
    if (callback) spi_master_register_callback(callback, arg, idx, true);
    
    // TX mapping
    DMA_Stream_TypeDef* stream = s_i2s_dma_map[idx].tx.stream;

    // If the data is 24 or 32 bits, we need two DMA transfers
    // Note that if the DATLEN bits are 0b00, that means 16 bit
    // data; 0b01 for 24 and 0b10 for 32. So a non zero value
    // from the DATLEN bits implies a transfer greater than 16 bits
    len = (handle->I2SCFGR & SPI_I2SCFGR_DATLEN) ? (len * 2) : len;
    
    // Set memory address and length
    dma_set_addresses(stream, NULL, buf, NULL);
    dma_set_trans_length(stream, len);
    
    // Enable DMA TX stream
    return dma_enable_stream(stream);
}

hal_err_t i2s_master_receive(I2S_TypeDef* handle, void* buf, uint16_t len, dma_trans_done_cb_t callback, void* arg) {
    
    // Get index for DMA stream mapping
    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU) return HAL_INVALID_ARG;
    
    // Save user passed callback
    if (callback) spi_master_register_callback(callback, arg, idx, false);
    
    // RX mapping
    DMA_Stream_TypeDef* stream = s_i2s_dma_map[idx].rx.stream;
    len = (handle->I2SCFGR & SPI_I2SCFGR_DATLEN) ? (len * 2) : len;
    
    // Set memory address and length
    dma_set_addresses(stream, NULL, buf, NULL);
    dma_set_trans_length(stream, len);
    
    // Enable DMA RX stream
    return dma_enable_stream(stream);
}

hal_err_t i2s_master_dbm_init(I2S_TypeDef* handle, void* buf_a, void* buf_b, uint16_t len, dma_trans_done_cb_t callback, void* arg) {
    
    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU) return HAL_INVALID_ARG;

    DMA_Stream_TypeDef* stream = s_i2s_dma_map[idx].rx.stream;
    len = (handle->I2SCFGR & SPI_I2SCFGR_DATLEN) ? (len * 2) : len;
    
    hal_err_t ret = dma_disable_stream(stream);
    if (ret != HAL_OK) return ret;

    dma_enable_circm_dbm(stream, true, true);
    dma_set_addresses(stream, &handle->DR, buf_a, buf_b);
    dma_set_trans_length(stream, len);
    
    // Write 0 to CT to ensure the DMA controller starts at `buf_a`
    stream->CR &= ~DMA_SxCR_CT;
    
    // Save user passed callback
    if (callback) spi_master_register_callback(callback, arg, idx, false);
    
    return HAL_OK;
}

hal_err_t i2s_master_dbm_deinit(I2S_TypeDef* handle) {

    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU) return HAL_INVALID_ARG;

    DMA_Stream_TypeDef* stream = s_i2s_dma_map[idx].rx.stream;

    hal_err_t ret = dma_disable_stream(stream);
    if (ret != HAL_OK) return ret;

    dma_enable_circm_dbm(stream, false, false);
    dma_set_trans_length(stream, 0);
    
    return HAL_OK;
}

hal_err_t i2s_master_dbm_start(I2S_TypeDef* handle) {
    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU) return HAL_INVALID_ARG;
    return dma_enable_stream(s_i2s_dma_map[idx].rx.stream);
}

hal_err_t i2s_master_dbm_stop(I2S_TypeDef* handle) {
    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU) return HAL_INVALID_ARG;
    return dma_disable_stream(s_i2s_dma_map[idx].rx.stream);
}

uint32_t i2s_master_dbm_get_filled_buffer(I2S_TypeDef* handle) {

    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU) return 0xFFU;

    DMA_Stream_TypeDef* stream = s_i2s_dma_map[idx].rx.stream;

    // CT represents the active buffer, that is, the
    // buffer currently in use by the dma controller.
    // 0 represents buf_a, and 1 buf_b. If the bit
    // is 0, that means buf_a is currently being used
    // by the DMA controller buf_b is filled and free
    return (stream->CR & DMA_SxCR_CT) ? 0x0U : 0x1U;
}
