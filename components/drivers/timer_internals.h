#ifndef TIMER_INTERNALS_H_
#define TIMER_INTERNALS_H_


#ifdef __cplusplus
extern "C" {
#endif


#include "stm32f411xe.h"
#include "drivers/timer.h"
#include "utils/err.h"

#include <stdint.h>


bool is_timer_on_apb1(TIM_TypeDef* handle);
bool is_timer_32_bits(TIM_TypeDef* handle);
bool is_timer_advanced(TIM_TypeDef* handle);

hal_err_t timer_set_arr_and_psc(TIM_TypeDef* handle, uint32_t timeout_us);
hal_err_t timer_get_frequency_hz(TIM_TypeDef* handle, uint32_t* timer_freq_hz);

// Only used when the timer is an advanced timer, ignored otherwise
typedef enum : uint8_t {
    UPDATE_EVENT,
    BREAK_EVENT,
    CAPTURE_COMPARE,
    TRG_COM_EVENT,
} advanced_timer_irq_type_t;

hal_err_t timer_register_callback(TIM_TypeDef* handle, timer_cb_t callback, void* arg, advanced_timer_irq_type_t type);


#ifdef __cplusplus
}
#endif


#endif // TIMER_INTERNALS_H_