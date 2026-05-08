#include "adc.h"
#include <stddef.h>


// ADC DMA stream settings
#define ADC_DMA_CONTROLLER       DMA2
#define ADC_DMA_STREAM           DMA2_Stream0
#define ADC_DMA_STREAM_NO        0U
#define ADC_DMA_CHANNEL          0U
#define ADC_DMA_IRQ_TYPE         DMA2_Stream0_IRQn

// To save user passed callback
static dma_trans_done_cb_t s_dma_trans_done_cb = NULL;
static void* s_arg = NULL;


// Public API
void adc_clk_enable(bool enable) {
    if (enable) RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    else        RCC->APB2ENR &= ~RCC_APB2ENR_ADC1EN;
    __DSB();
}

void adc_power_on(bool on) {
    ADC1->CR2 &= ~ADC_CR2_ADON;
    if (on) ADC1->CR2 |= ADC_CR2_ADON;
}

void adc_init(const adc_config_t* config) {
    (void)config;
}

hal_err_t adc_dma_init(void) {
    
    hal_err_t ret = dmax_clk_enable(ADC_DMA_CONTROLLER, true);
    if (ret != HAL_OK) return ret;
    
    // Disable DMA stream before configuring
    ret = dma_disable_stream(ADC_DMA_STREAM);
    if (ret != HAL_OK) return ret;
    
    // Stream configuration
    dma_clear_flags(ADC_DMA_CONTROLLER, ADC_DMA_STREAM_NO);
    dma_set_channel(ADC_DMA_STREAM, ADC_DMA_CHANNEL);
    dma_set_direct_mode(ADC_DMA_STREAM, true);
    dma_set_direction(ADC_DMA_STREAM, DMA_DIR_P_M);
    dma_set_flow_controller(ADC_DMA_STREAM, true);
    dma_set_stream_priority(ADC_DMA_STREAM, DMA_PRIORITY_LOW);
    dma_enable_circm_dbm(ADC_DMA_STREAM, false, false);
    dma_set_increment(ADC_DMA_STREAM, false, true);
    dma_enable_irqs(ADC_DMA_STREAM, true, true, false, true);
    dma_set_addresses(ADC_DMA_STREAM, &ADC1->DR, NULL, NULL);
    dma_set_per_mem_size(ADC_DMA_STREAM, DMA_SIZE_HWORD, DMA_SIZE_HWORD);
    
    // Enable DMA stream interrupts
    NVIC_EnableIRQ(ADC_DMA_IRQ_TYPE);
    NVIC_SetPriority(ADC_DMA_IRQ_TYPE, ADC_DMA_NVIC_IRQ_PRIORITY);

    return HAL_OK;
}

DMA_Stream_TypeDef* adc_get_dma_stream(void) {
    return ADC_DMA_STREAM;
}

uint16_t adc_get_sample_oneshot(void) {
    // Disable continuous sampling
    ADC1->CR2 &= ~ADC_CR2_CONT;

    // Start a conversion
    ADC1->CR2 |= ADC_CR2_SWSTART;

    // Poll till the end of conversion is reached
    while (!(ADC1->SR & ADC_SR_EOC));

    return (uint16_t)ADC1->DR;
}

hal_err_t adc_get_sample_continuous(void* buf, uint16_t len, dma_trans_done_cb_t callback, void* arg) {
    
    // Set memory address and length
    dma_set_addresses(ADC_DMA_STREAM, NULL, buf, NULL);
    dma_set_trans_length(ADC_DMA_STREAM, len);
    
    // Save user passed callback
    if (callback) {
        s_dma_trans_done_cb = callback;
        s_arg = arg;
    }
    
    // Enable stream
    return dma_enable_stream(ADC_DMA_STREAM);
}

hal_err_t adc_dbm_init(void* buf_a, void* buf_b, uint16_t len, dma_trans_done_cb_t callback, void* arg) {

    hal_err_t ret = dma_disable_stream(ADC_DMA_STREAM);
    if (ret != HAL_OK) return ret;

    // Configure stream
    dma_enable_circm_dbm(ADC_DMA_STREAM, true, true);
    dma_set_addresses(ADC_DMA_STREAM, &ADC1->DR, buf_a, buf_b);
    dma_set_trans_length(ADC_DMA_STREAM, len);
    
    // Write 0 to CT to ensure the DMA controller starts at buf_a
    ADC_DMA_STREAM->CR &= ~DMA_SxCR_CT;
    
    // Save user passed callback
    if (callback) {
        s_dma_trans_done_cb = callback;
        s_arg = arg;
    }
    
    return HAL_OK;
}

hal_err_t adc_dbm_deinit(void) {

    hal_err_t ret = dma_disable_stream(ADC_DMA_STREAM);
    if (ret != HAL_OK) return ret;

    dma_enable_circm_dbm(ADC_DMA_STREAM, false, false);
    dma_set_trans_length(ADC_DMA_STREAM, 0);
    
    return HAL_OK;
}

hal_err_t adc_dbm_start(void) {
    return dma_enable_stream(ADC_DMA_STREAM);
}

hal_err_t adc_dbm_stop(void) {
    return dma_disable_stream(ADC_DMA_STREAM);
}

void DMA2_Stream0_IRQHandler(void) {
    hal_err_t ret = dma_isr_helper(DMA2_Stream0, &DMA2->LIFCR, &DMA2->LISR, DMA_LISR_TCIF0, DMA_LISR_TEIF0, DMA_LISR_DMEIF0, DMA_LISR_HTIF0);

    // Invoke user callback
    if (s_dma_trans_done_cb) s_dma_trans_done_cb(s_arg, ret);

    // If in circular mode or dbm, return so as not to clear user callback
    if (ADC_DMA_STREAM->CR & (DMA_SxCR_CIRC| DMA_SxCR_DBM)) return;

    s_dma_trans_done_cb = NULL;
    s_arg = NULL;
}

void ADC_IRQHandler(void) {

}
