#ifndef TIMER_INTERNALS_H_
#define TIMER_INTERNALS_H_


#ifdef __cplusplus
extern "C" {
#endif


#include "drivers/timer.h"
#include "utils/common.h"
#include "utils/err.h"

#include <stdint.h>


bool      is_timer_on_apb1(TIM_TypeDef* handle);
bool      is_timer_32_bits(TIM_TypeDef* handle);
hal_err_t timer_set_arr_and_psc(TIM_TypeDef* handle, uint32_t timeout_us);
hal_err_t timer_get_frequency_hz(TIM_TypeDef* handle, uint32_t* timer_freq_hz);
hal_err_t timer_register_callback(timer_cb_t callback, void* arg, uint8_t idx);


#ifdef __cplusplus
}
#endif


#endif // TIMER_INTERNALS_H_