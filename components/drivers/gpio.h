#ifndef GPIO_H_
#define GPIO_H_


#ifdef __cplusplus
extern "C" {
#endif


#include "stm32f411xe.h"
#include "utils/err.h"

#include <stdint.h>


typedef enum : uint8_t {
    GPIO_LOW_SPEED = 0,
    GPIO_MEDIUM_SPEED,
    GPIO_FAST_SPEED,
    GPIO_HIGH_SPEED,
} gpio_speed_mode_t;

typedef enum : uint8_t {
    GPIO_NO_EDGE = 0,
    GPIO_RISING_EDGE_ONLY,
    GPIO_FALLING_EDGE_ONLY,
    GPIO_RISING_AND_FALLING_EDGE,
} gpio_edge_trigger_t;

typedef enum : uint8_t {
    GPIO_PUSH_PULL = 0,
    GPIO_OPEN_DRAIN,
} gpio_output_type_t;

hal_err_t gpiox_clk_enable(GPIO_TypeDef* port, bool enable);

void      gpio_set_output(GPIO_TypeDef* port, uint8_t pin);
void      gpio_set_input(GPIO_TypeDef* port, uint8_t pin);
void      gpio_set_analog(GPIO_TypeDef* port, uint8_t pin);
hal_err_t gpio_set_alternate_function(GPIO_TypeDef* port, uint8_t pin, uint8_t alt_val);

void gpio_enable_pullup(GPIO_TypeDef* port, uint8_t pin, bool enable);
void gpio_enable_pulldown(GPIO_TypeDef* port, uint8_t pin, bool enable);

void gpio_set_speed_mode(GPIO_TypeDef* port, uint8_t pin, gpio_speed_mode_t mode);
void gpio_set_output_type(GPIO_TypeDef* port, uint8_t pin, gpio_output_type_t type);

void gpio_level_set(GPIO_TypeDef* port, uint8_t pin, bool level);
void gpio_level_toggle(GPIO_TypeDef* port, uint8_t pin);
bool gpio_get_level(GPIO_TypeDef* port, uint8_t pin);

hal_err_t gpio_set_interrupt(GPIO_TypeDef* port, uint8_t pin, gpio_edge_trigger_t edge);
void      gpio_clear_interrupt(GPIO_TypeDef* port, uint8_t pin);


#ifdef __cplusplus
}
#endif


#endif // GPIO_H_