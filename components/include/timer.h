#ifndef _TIMER_H_
#define _TIMER_H_

#ifdef __cpluplus
extern "C" {
#endif

#include "stdint.h"
#include "stm32f411xe.h"

void timer_init(TIM_TypeDef* timer, uint16_t prescaler);
void timer_set_reload_value(TIM_TypeDef* timer, uint32_t value);
void timer_start(TIM_TypeDef* timer);
void timer_end(TIM_TypeDef* timer);

#ifdef __cpluplus
}
#endif

#endif // _TIMER_H_