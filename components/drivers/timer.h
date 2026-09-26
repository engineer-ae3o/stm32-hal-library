#ifndef TIMER_H_
#define TIMER_H_


#ifdef __cplusplus
extern "C" {
#endif


#include "stm32f411xe.h"
#include "utils/err.h"

#include <stdint.h>


typedef struct {

} timer_config_t;

typedef void (*timer_cb_t)(void* arg);

hal_err_t timer_clock_enable(TIM_TypeDef* handle, bool enable);

hal_err_t timer_init(TIM_TypeDef* handle, const timer_config_t* config, timer_cb_t cb, void* arg);
hal_err_t timer_deinit(TIM_TypeDef* handle);

hal_err_t timer_start_oneshot(TIM_TypeDef* handle, uint32_t timeout_us);
hal_err_t timer_start_periodic(TIM_TypeDef* handle, uint32_t timeout_us);
hal_err_t timer_stop(TIM_TypeDef* handle);
hal_err_t timer_restart(TIM_TypeDef* handle, uint32_t timeout_us);


#ifdef __cplusplus
}
#endif


#endif // TIMER_H_