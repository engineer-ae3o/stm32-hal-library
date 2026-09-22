#ifndef GPIO_H_
#define GPIO_H_


#ifdef __cplusplus
extern "C" {
#endif


#include "stm32f411xe.h"
#include "drivers/gpio_types.h"
#include "utils/err.h"

#include <stdint.h>


hal_err_t gpiox_clk_enable(GPIO_TypeDef* port, bool enable);

void gpio_set_output(GPIO_TypeDef* port, gpio_pin_t pin);
void gpio_set_input(GPIO_TypeDef* port, gpio_pin_t pin);
void gpio_set_analog(GPIO_TypeDef* port, gpio_pin_t pin);
void gpio_set_alternate_function(GPIO_TypeDef* port, gpio_pin_t pin, uint8_t alt_val);

void gpio_enable_pullups(GPIO_TypeDef* port, gpio_pin_t pin, bool enable);
void gpio_enable_pulldowns(GPIO_TypeDef* port, gpio_pin_t pin, bool enable);

void gpio_set_speed_mode(GPIO_TypeDef* port, gpio_pin_t pin, gpio_speed_mode_t mode);
void gpio_set_output_type(GPIO_TypeDef* port, gpio_pin_t pin, gpio_output_type_t type);

void gpio_set_level(GPIO_TypeDef* port, gpio_pin_t pin, bool level);
void gpio_level_toggle(GPIO_TypeDef* port, gpio_pin_t pin);
bool gpio_get_level(GPIO_TypeDef* port, gpio_pin_t pin);

hal_err_t gpio_set_interrupt(GPIO_TypeDef* port, gpio_pin_t pin, gpio_edge_trigger_t edge, gpio_exti_cb_t callback, void* arg);
void      gpio_clear_interrupt(gpio_pin_t pin);
void      gpio_generate_software_interrupt(gpio_pin_t pin);
IRQn_Type gpio_get_nvic_irq_type(gpio_pin_t pin);


#ifdef __cplusplus
}
#endif


#endif // GPIO_H_