#ifndef _TIMER_H_
#define _TIMER_H_

#ifdef __cpluplus
extern "C" {
#endif

#include "stm32f411xe.h"
#include "stdint.h"

void timer_init(void);
void timer_set_reload_value(uint32_t value);
void timer_start(void);
void timer_end(void);

#ifdef __cpluplus
}
#endif

#endif // _TIMER_H_