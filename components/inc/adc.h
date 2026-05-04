#ifndef _ADC_H_
#define _ADC_H_


#ifdef __cplusplus
extern "C" {
#endif


#include "stm32f411xe.h"
#include "common.h"

#include <stdint.h>
#include <stdbool.h>


typedef struct {

} adc_config_t;

hal_err_t adc_init(ADC_TypeDef* handle, const adc_config_t* config);
hal_err_t adc_dma_init(ADC_TypeDef* handle);

// Reads a sample and returns it
uint16_t adc_get_sample_oneshot(ADC_TypeDef* handle);

hal_err_t adc_start_sampling(ADC_TypeDef* handle, void* buffer, uint16_t len);
hal_err_t adc_stop_sampling(ADC_TypeDef* handle);


#ifdef __cplusplus
}
#endif


#endif // _ADC_H_