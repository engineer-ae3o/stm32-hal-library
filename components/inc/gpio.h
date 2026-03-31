#ifndef _GPIO_H_
#define _GPIO_H_


#ifdef __cplusplus
extern "C" {
#endif


#include "stm32f411xe.h"

void gpiox_clk_enable(GPIO_TypeDef* port);
void gpio_set_output(GPIO_TypeDef* port, uint8_t pin);
void gpio_set_input(GPIO_TypeDef* port, uint8_t pin);
void gpio_set(GPIO_TypeDef* port, uint8_t pin);
void gpio_reset(GPIO_TypeDef* port, uint8_t pin);
void gpio_toggle(GPIO_TypeDef* port, uint8_t pin);
bool gpio_get_level(GPIO_TypeDef* port, uint8_t pin);
void gpio_enable_sys_clk(void);
void gpio_set_interupt(GPIO_TypeDef* port, uint8_t pin, uint8_t edge);
void gpio_clear_interrupt(uint8_t pin);


#ifdef __cplusplus
}
#endif


#endif // _GPIO_H_