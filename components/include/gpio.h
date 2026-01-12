#ifndef _GPIO_H_
#define _GPIO_H_

#ifdef __cpluplus
extern "C" {
#endif

#include "stm32f411xe.h"

void gpio_init(GPIO_TypeDef* port, uint32_t pin);
void gpio_set(GPIO_TypeDef* port, uint32_t pin);
void gpio_reset(GPIO_TypeDef* port, uint32_t pin);
void gpio_toggle(GPIO_TypeDef* port, uint32_t pin);
bool gpio_get_level(GPIO_TypeDef* port, uint32_t pin);


#ifdef __cpluplus
}
#endif

#endif // _GPIO_H_