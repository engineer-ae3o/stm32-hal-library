#include "adc.h"


hal_err_t adc_clk_enable(bool enable) {

}

hal_err_t adc_init(const adc_config_t* config) {

}

hal_err_t adc_dma_init(void) {

}

DMA_Stream_TypeDef* adc_get_dma_stream(void) {

}

uint16_t adc_get_sample_oneshot(void) {

}

hal_err_t adc_get_sample_continuous(void* buf, uint16_t len, dma_trans_done_cb_t callback, void* arg) {
    
}

hal_err_t adc_dbm_init(void* buf_a, void* buf_b, uint16_t len, dma_trans_done_cb_t callback, void* arg) {
    
}

hal_err_t adc_dbm_deinit(void) {

}

hal_err_t adc_dbm_start(void) {

}

hal_err_t adc_dbm_stop(void) {

}

uint32_t adc_dbm_get_filled_buffer(void) {

}
