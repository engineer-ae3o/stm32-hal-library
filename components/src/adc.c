#include "adc.h"


hal_err_t adc_init(ADC_TypeDef* handle, const adc_config_t* config) {

    (void)handle;
    (void)config;

    return HAL_OK;
}

hal_err_t adc_dma_init(ADC_TypeDef* handle) {

    (void)handle;

    return HAL_OK;
}

uint16_t adc_get_sample_oneshot(ADC_TypeDef* handle) {

    (void)handle;

    return 0U;
}

hal_err_t adc_start_sampling(ADC_TypeDef* handle, void* buf, uint16_t len) {

    (void)handle;
    (void)buf;
    (void)len;

    return HAL_OK;
}

hal_err_t adc_stop_sampling(ADC_TypeDef* handle) {

    (void)handle;

    return HAL_OK;
}
