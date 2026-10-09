#ifndef TIMER_INTERNALS_H_
#define TIMER_INTERNALS_H_


#ifdef __cplusplus
extern "C" {
#endif


#include "stm32f411xe.h"
#include "drivers/timer_types.h"
#include "utils/err.h"

#include <stdint.h>


typedef enum : uint8_t {
    TIMER_IRQ_UPDATE_EVENT,
    TIMER_IRQ_BREAK_EVENT,
    TIMER_IRQ_CAPTURE_COMPARE,
    TIMER_IRQ_TRIGGER_EVENT,
    TIMER_IRQ_COMMUTATION_EVENT,
} advanced_timer_irq_type_t;

bool is_timer_lite(TIM_TypeDef* handle);
bool is_timer_advanced(TIM_TypeDef* handle);

bool is_timer_on_apb1(TIM_TypeDef* handle);
bool is_timer_32_bits(TIM_TypeDef* handle);

uint8_t  timer_get_num_channels(TIM_TypeDef* handle);
uint32_t timer_get_frequency_hz(TIM_TypeDef* handle);

hal_err_t timer_set_arr_and_psc(TIM_TypeDef* handle, uint32_t timeout_us);
hal_err_t timer_filter_ns_to_ic_code(TIM_TypeDef* handle, uint32_t filter_ns, uint8_t* code);
hal_err_t timer_register_callback(TIM_TypeDef* handle, timer_cb_t callback, advanced_timer_irq_type_t type);


#ifdef __cplusplus
}
#endif


#endif // TIMER_INTERNALS_H_