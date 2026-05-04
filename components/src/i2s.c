#include "stm32f411xe.h"
#include "i2s.h"
#include "spi.h"



// Mapping for the DMA channels for the 5 I2S channels
static const dma_stream_map_t s_i2s_dma_map[5] = {
    // I2S1
    {
        .tx = { .controller = DMA2, .stream = DMA2_Stream5, .stream_no = 5, .irq_type = DMA2_Stream5_IRQn, .channel = 3 },
        .rx = { .controller = DMA2, .stream = DMA2_Stream2, .stream_no = 2, .irq_type = DMA2_Stream2_IRQn, .channel = 3 }
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
        .rx = { .controller = DMA2, .stream = DMA2_Stream0, .stream_no = 0, .irq_type = DMA2_Stream0_IRQn, .channel = 4 }
    },
    // I2S5
    {
        .tx = { .controller = DMA2, .stream = DMA2_Stream4, .stream_no = 4, .irq_type = DMA2_Stream4_IRQn, .channel = 2 },
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
hal_err_t i2s_master_init(I2S_TypeDef* handle, const i2s_master_config_t* config) {

}

hal_err_t i2s_master_dma_init(I2S_TypeDef* handle) {

}

hal_err_t i2s_master_transmit(I2S_TypeDef* handle, const void* buf, uint16_t len,
                              dma_trans_done_cb_t callback, void* arg) {
    
}

hal_err_t i2s_master_receive(I2S_TypeDef* handle, void* buf, uint16_t len,
                             dma_trans_done_cb_t callback, void* arg) {
        
}

hal_err_t i2s_master_transceive(I2S_TypeDef* handle, const void* tx_data, void* rx_data,
                                uint16_t len, dma_trans_done_cb_t callback, void* arg) {

}

hal_err_t i2s_master_dbm_init(I2S_TypeDef* handle, dma_stream_dir_t dir, void* buf_a,
                              void* buf_b, uint16_t len, dma_dbm_done_cb_t cb, void* arg) {
    
}

hal_err_t i2s_master_dbm_deinit(I2S_TypeDef* handle) {

}

hal_err_t i2s_master_dbm_start(I2S_TypeDef* handle) {
    const uint8_t idx = get_index(handle);
    return dma_enable_stream(I2S_dma_stream_from_handle((I2S_TypeDef*)handle));
}

hal_err_t i2s_master_dbm_stop(I2S_TypeDef* handle) {
    return dma_enable_stream(I2S_dma_stream_from_handle((I2S_TypeDef*)handle));
}

hal_err_t i2s_master_dbm_pause(I2S_TypeDef* handle) {
    return dma_disable_stream(I2S_dma_stream_from_handle((I2S_TypeDef*)handle));
}
