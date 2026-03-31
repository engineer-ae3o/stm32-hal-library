#ifndef _ADC_H_
#define _ADC_H_


#ifdef __cplusplus
extern "C" {
#endif


#include "stm32f411xe.h"

#include <stdint.h>


void adc_init();
void adc_get_oneshot_reading();
void adc_start_continuous();
void adc_stop_continuous();


#ifdef __cplusplus
}
#endif


#endif // _ADC_H_