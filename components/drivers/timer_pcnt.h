#ifndef TIMER_PCNT_H_
#define TIMER_PCNT_H_


#ifdef __cplusplus
extern "C" {
#endif


#include "stm32f411xe.h"
#include "drivers/timer_pcnt_types.h"
#include "utils/err.h"

#include <stdint.h>


// Pulse counting API.
// Supports plain pulse counting and quadrature decoding.
// Note that all the functions here are safe to call from either mode
// after the configuration of the timer in the required mode is complete.
// Also, calling either *_init(...) function will overwrite the other mode
// if it were previously initialized.


hal_err_t pcnt_init(TIM_TypeDef* handle, const pcnt_config_t* config);
hal_err_t qdec_init(TIM_TypeDef* handle, const qdec_config_t* config);

hal_err_t pcnt_deinit(TIM_TypeDef* handle);

hal_err_t pcnt_start(TIM_TypeDef* handle);
hal_err_t pcnt_stop(TIM_TypeDef* handle);

hal_err_t pcnt_set_count(TIM_TypeDef* handle, uint32_t count);
hal_err_t pcnt_get_count(TIM_TypeDef* handle, uint32_t* count);


#ifdef __cplusplus
}
#endif


#endif // TIMER_PCNT_H_