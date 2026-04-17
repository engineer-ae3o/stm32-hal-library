#include "gpio.h"
#include <stdint.h>


void gpiox_clk_enable(GPIO_TypeDef* port) {
    if (port == GPIOA) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    } else if (port == GPIOB) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    } else if (port == GPIOC) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    } else if (port == GPIOD) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    }
}

void gpio_set_output(GPIO_TypeDef* port, uint8_t pin) {
    port->MODER |= (1UL << (pin * 2));
    port->OTYPER &= ~(1UL << pin);
    port->OSPEEDR |= (3UL << (pin * 2));
    port->BSRR = (1UL << pin);
}

void gpio_set_input(GPIO_TypeDef* port, uint8_t pin) {
    port->MODER &= ~(3UL << (pin * 2));
    port->PUPDR |= (1UL << (pin * 2));
}

void gpio_set(GPIO_TypeDef* port, uint8_t pin) {
    port->BSRR = (1UL << pin);
}

void gpio_reset(GPIO_TypeDef* port, uint8_t pin) {
    port->BSRR = (1UL << (pin + 16));
}

void gpio_toggle(GPIO_TypeDef* port, uint8_t pin) {
    port->ODR ^= (1UL << pin);
}

bool gpio_get_level(GPIO_TypeDef* port, uint8_t pin) {
    return ((port->IDR >> pin) & 0x01);
}

void gpio_enable_sys_clk(void) {
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
}

void gpio_set_interupt(GPIO_TypeDef* port, uint8_t pin, uint8_t edge) {
    // Extract register index and bit poition
    const uint8_t reg_idx = pin / 4;
    const uint8_t bit_pos = (pin % 4) * 4;

    // Get port code from gpio port
    uint8_t port_code = 0;
    if (port == GPIOA) {
        port_code = 0;
    } else if (port == GPIOB) {
        port_code = 1;
    } else if (port == GPIOC) {
        port_code = 2;
    } else if (port == GPIOD) {
        port_code = 3;
    }

    // Set external interrupt configuration register
    SYSCFG->EXTICR[reg_idx] &= ~(0xFUL << bit_pos);
    SYSCFG->EXTICR[reg_idx] |= (port_code << bit_pos);

    // Extract rising and falling bits from edge variable
    const uint8_t rising_bit = edge & 0x01UL;
    const uint8_t falling_bit = (edge >> 1UL) & 0x01UL;

    // Clear interrupt edge registers
    EXTI->RTSR &= ~(1UL << pin);
    EXTI->FTSR &= ~(1UL << pin);

    // Set interrupts edge registers if enabled
    if (rising_bit) EXTI->RTSR |= (1UL << pin);
    if (falling_bit) EXTI->FTSR |= (1UL << pin);

    // Unmask interrupts for pin
    EXTI->IMR |= (1UL << pin);
}

void gpio_clear_interrupt(uint8_t pin) {
    // Extract register index and bit poition
    const uint8_t reg_idx = pin / 4;
    const uint8_t bit_pos = (pin % 4) * 4;

    // Clear external interrupt configuration register
    SYSCFG->EXTICR[reg_idx] &= ~(0xFUL << bit_pos);

    // Clear interrupt edge registers
    EXTI->RTSR &= ~(1UL << pin);
    EXTI->FTSR &= ~(1UL << pin);
    
    // Mask interrupts for pin
    EXTI->IMR &= ~(1UL << pin);
}
