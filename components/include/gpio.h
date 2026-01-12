#ifndef _GPIO_H_
#define _GPIO_H_

#ifdef __cpluplus
extern "C" {
#endif

#include "stm32f411xe.h"


typedef enum  {
    GPIO_A,
    GPIO_B,
    GPIO_C,
    GPIO_D
} gpio_port_t;

void enable_gpiox_clk(gpio_port_t port);
void gpio_set_output(GPIO_TypeDef* port, uint32_t pin);
void gpio_set_input(GPIO_TypeDef* port, uint32_t pin);
void gpio_set(GPIO_TypeDef* port, uint32_t pin);
void gpio_reset(GPIO_TypeDef* port, uint32_t pin);
void gpio_toggle(GPIO_TypeDef* port, uint32_t pin);
bool gpio_get_level(GPIO_TypeDef* port, uint32_t pin);


#ifdef __cpluplus
}
#endif

#endif // _GPIO_H_