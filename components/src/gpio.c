#include "gpio.h"
#include <stdint.h>


void gpio_init(GPIO_TypeDef* port, uint32_t pin) {

    // Enable GPIO clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN_Pos;

    port->MODER |= (1UL << (pin * 2));
    port->OSPEEDR |= (0UL << pin);
    port->OSPEEDR |= (2UL << pin);
    port->BSRR |= (1UL << pin);
}

void gpio_set(GPIO_TypeDef* port, uint32_t pin) {
    port->BSRR |= (1UL << pin);
}

void gpio_reset(GPIO_TypeDef* port, uint32_t pin) {
    port->BSRR |= (1UL << (pin + 16));
}

void gpio_toggle(GPIO_TypeDef* port, uint32_t pin) {
    port->ODR ^= (1UL << pin);
}

bool gpio_get_level(GPIO_TypeDef* port, uint32_t pin) {
    return ((port->IDR >> pin) & 0x01);
}
