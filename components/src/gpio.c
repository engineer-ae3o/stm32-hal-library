#include "gpio.h"
#include <stdint.h>


void enable_gpiox_clk(gpio_port_t port) {
    switch (port) {
    case GPIO_A:
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
        break;
    case GPIO_B:
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
        break;
    case GPIO_C:
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
        break;
    case GPIO_D:
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
        break;
    default:
        break;
    }
}

void gpio_set_output(GPIO_TypeDef* port, uint32_t pin) {
    port->MODER |= (1UL << (pin * 2));
    port->OTYPER &= ~(1UL << pin);
    port->OSPEEDR |= (3UL << (pin * 2));
    port->BSRR = (1UL << pin);
}

void gpio_set_input(GPIO_TypeDef* port, uint32_t pin) {
    port->MODER &= ~(3UL << (pin * 2));
    port->PUPDR |=(1UL << (pin * 2));
}

void gpio_set(GPIO_TypeDef* port, uint32_t pin) {
    port->BSRR = (1UL << pin);
}

void gpio_reset(GPIO_TypeDef* port, uint32_t pin) {
    port->BSRR = (1UL << (pin + 16));
}

void gpio_toggle(GPIO_TypeDef* port, uint32_t pin) {
    port->ODR ^= (1UL << pin);
}

bool gpio_get_level(GPIO_TypeDef* port, uint32_t pin) {
    return ((port->IDR >> pin) & 0x01);
}
