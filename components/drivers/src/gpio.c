#include "drivers/gpio.h"
#include "utils/err.h"

#include <stdint.h>
#include <stddef.h>


hal_err_t gpiox_clk_enable(GPIO_TypeDef* port, bool enable) {
    if (enable) {
        if (port == GPIOA) {
            RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
        } else if (port == GPIOB) {
            RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
        } else if (port == GPIOC) {
            RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
        } else if (port == GPIOD) {
            RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
        } else if (port == GPIOE) {
            RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
        } else if (port == GPIOH) {
            RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN;
        } else {
            return HAL_ERR_INVALID_ARG;
        }

    } else {
        if (port == GPIOA) {
            RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOAEN;
        } else if (port == GPIOB) {
            RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOBEN;
        } else if (port == GPIOC) {
            RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOCEN;
        } else if (port == GPIOD) {
            RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIODEN;
        } else if (port == GPIOE) {
            RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOEEN;
        } else if (port == GPIOH) {
            RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOHEN;
        } else {
            return HAL_ERR_INVALID_ARG;
        }
    }

    __DSB();
    return HAL_OK;
}

void gpio_set_output(GPIO_TypeDef* port, gpio_pin_t pin) {
    if (port) {
        port->MODER = (port->MODER & ~(0b11UL << (pin * 2))) | (0b1UL << (pin * 2));
    }
}

void gpio_set_input(GPIO_TypeDef* port, gpio_pin_t pin) {
    if (port) {
        port->MODER &= ~(0b11UL << (pin * 2));
    }
}

void gpio_set_analog(GPIO_TypeDef* port, gpio_pin_t pin) {
    if (port) {
        port->MODER |= (0b11UL << (pin * 2));
    }
}

hal_err_t gpio_set_alternate_function(GPIO_TypeDef* port, gpio_pin_t pin, uint8_t alt_val) {
    if (port == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    // Set the MODER for alternate mode
    port->MODER = (port->MODER & ~(0b11UL << (pin * 2))) | (0b10UL << (pin * 2));

    // Set the specified alternate function
    if (pin <= GPIO_PIN_7) {
        port->AFR[0] &= ~(0xFUL << (pin * 4UL));
        port->AFR[0] |= ((alt_val & 0xFUL) << (pin * 4UL));
    } else if (pin <= GPIO_PIN_15) {
        port->AFR[1] &= ~(0xFUL << ((pin - 8) * 4UL));
        port->AFR[1] |= ((alt_val & 0xFUL) << ((pin - 8) * 4UL));
    } else {
        return HAL_ERR_INVALID_ARG;
    }

    return HAL_OK;
}

void gpio_enable_pullups(GPIO_TypeDef* port, gpio_pin_t pin, bool enable) {
    if (port) {
        if (enable) {
            port->PUPDR = (port->PUPDR & ~(0b11UL << (pin * 2))) | (0b1UL << (pin * 2));
        } else {
            port->PUPDR &= ~(0b11UL << (pin * 2));
        }
    }
}

void gpio_enable_pulldowns(GPIO_TypeDef* port, gpio_pin_t pin, bool enable) {
    if (port) {
        if (enable) {
            port->PUPDR = (port->PUPDR & ~(0b11UL << (pin * 2))) | (0b10UL << (pin * 2));
        } else {
            port->PUPDR &= ~(0b11UL << (pin * 2));
        }
    }
}

void gpio_set_output_type(GPIO_TypeDef* port, gpio_pin_t pin, gpio_output_type_t type) {
    if (port) {
        port->OTYPER = (port->OTYPER & ~(1UL << pin)) | ((uint32_t)type << pin);
    }
}

void gpio_set_speed_mode(GPIO_TypeDef* port, gpio_pin_t pin, gpio_speed_mode_t mode) {
    if (port) {
        port->OSPEEDR = (port->OSPEEDR & ~(0b11UL << (pin * 2))) | ((uint32_t)mode << (pin * 2));
    }
}

void gpio_set_level(GPIO_TypeDef* port, gpio_pin_t pin, bool level) {
    if (port) {
        level ? (port->BSRR = (0b1UL << pin)) : (port->BSRR = (0b1UL << (pin + 16)));
    }
}

void gpio_level_toggle(GPIO_TypeDef* port, gpio_pin_t pin) {
    if (port) {
        // Read the level of the pin and then toggle it
        gpio_set_level(port, pin, !gpio_get_level(port, pin));
    }
}

bool gpio_get_level(GPIO_TypeDef* port, gpio_pin_t pin) {
    if (!port) {
        return false;
    }
    return ((port->IDR >> pin) & 0x01U);
}

hal_err_t gpio_set_interrupt(GPIO_TypeDef* port, gpio_pin_t pin, gpio_edge_trigger_t edge) {
    // Extract rising and falling bits from edge variable
    const bool rising  = (uint8_t)edge & 0x1U;
    const bool falling = ((uint8_t)edge >> 0b1U) & 0x1U;

    if (!rising && !falling) {
        return HAL_ERR_INVALID_ARG;
    }

    // Extract register index and bit position
    const uint8_t reg_idx = pin / 4;
    const uint8_t bit_pos = (pin % 4) * 4;

    // Get the port code from the gpio port
    uint8_t port_code = 0;
    if (port == GPIOA) {
        port_code = 0b000U;
    } else if (port == GPIOB) {
        port_code = 0b001U;
    } else if (port == GPIOC) {
        port_code = 0b010U;
    } else if (port == GPIOD) {
        port_code = 0b011U;
    } else if (port == GPIOE) {
        port_code = 0b100U;
    } else if (port == GPIOH) {
        port_code = 0b111U;
    } else {
        return HAL_ERR_INVALID_ARG;
    }

    // Enable the SYSCFG clock
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    // Set external interrupt configuration register
    SYSCFG->EXTICR[reg_idx] &= ~(0xFUL << bit_pos);
    SYSCFG->EXTICR[reg_idx] |= (uint32_t)(port_code << bit_pos);

    // Clear interrupt edge registers
    EXTI->RTSR &= ~(0b1UL << pin);
    EXTI->FTSR &= ~(0b1UL << pin);

    // Set interrupts edge registers if enabled
    if (rising) {
        EXTI->RTSR |= (0b1UL << pin);
    }
    if (falling) {
        EXTI->FTSR |= (0b1UL << pin);
    }

    // Unmask interrupts for the pin
    EXTI->IMR |= (0b1UL << pin);

    // Clear the interrupt flag
    EXTI->PR = (0b1UL << pin);

    return HAL_OK;
}

void gpio_clear_interrupt(GPIO_TypeDef* port, gpio_pin_t pin) {
    if (port) {
        // Extract register index and bit position
        const uint8_t reg_idx = pin / 4;
        const uint8_t bit_pos = (pin % 4) * 4;

        // Clear external interrupt configuration register
        SYSCFG->EXTICR[reg_idx] &= ~(0xFUL << bit_pos);

        // Clear both interrupt edge registers
        EXTI->RTSR &= ~(0b1UL << pin);
        EXTI->FTSR &= ~(0b1UL << pin);

        // Clear interrupt flag
        EXTI->PR = (0b1UL << pin);

        // Mask interrupts for the pin
        EXTI->IMR &= ~(0b1UL << pin);
    }
}
