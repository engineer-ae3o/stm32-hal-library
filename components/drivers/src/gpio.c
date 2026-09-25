#include "drivers/gpio_types.h"
#include "stm32f411xe.h"
#include "drivers/gpio.h"
#include "utils/common.h"
#include "utils/err.h"

#include <stdint.h>
#include <stddef.h>


static const IRQn_Type s_exti_irq_lut[] = {
    [GPIO_PIN_0]  = EXTI0_IRQn,
    [GPIO_PIN_1]  = EXTI1_IRQn,
    [GPIO_PIN_2]  = EXTI2_IRQn,
    [GPIO_PIN_3]  = EXTI3_IRQn,
    [GPIO_PIN_4]  = EXTI4_IRQn,
    [GPIO_PIN_5]  = EXTI9_5_IRQn,
    [GPIO_PIN_6]  = EXTI9_5_IRQn,
    [GPIO_PIN_7]  = EXTI9_5_IRQn,
    [GPIO_PIN_8]  = EXTI9_5_IRQn,
    [GPIO_PIN_9]  = EXTI9_5_IRQn,
    [GPIO_PIN_10] = EXTI15_10_IRQn,
    [GPIO_PIN_11] = EXTI15_10_IRQn,
    [GPIO_PIN_12] = EXTI15_10_IRQn,
    [GPIO_PIN_13] = EXTI15_10_IRQn,
    [GPIO_PIN_14] = EXTI15_10_IRQn,
    [GPIO_PIN_15] = EXTI15_10_IRQn,
};

typedef struct {
    gpio_exti_cb_t callback;
    void*          arg;
} gpio_irq_ctx_t;

static gpio_irq_ctx_t s_gpio_irq_ctx[ARRAY_SIZE(s_exti_irq_lut)] = {};

// Helper
[[__gnu__::__always_inline__]] static inline void gpio_isr_helper(gpio_pin_t pin) {
    if (EXTI->PR & (1UL << pin)) {
        EXTI->PR = (1UL << pin);
        if (s_gpio_irq_ctx[pin].callback) {
            s_gpio_irq_ctx[pin].callback(s_gpio_irq_ctx[pin].arg);
        }
    }
}


// Public API
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

void gpio_set_alternate_function(GPIO_TypeDef* port, gpio_pin_t pin, uint8_t alt_val) {
    if (port) {
        // Set the MODER for alternate mode
        port->MODER = (port->MODER & ~(0b11UL << (pin * 2))) | (0b10UL << (pin * 2));

        // Set the specified alternate function
        if (pin <= GPIO_PIN_7) {
            port->AFR[0] &= ~(0xFUL << (pin * 4UL));
            port->AFR[0] |= ((alt_val & 0xFUL) << (pin * 4UL));
        } else {
            port->AFR[1] &= ~(0xFUL << ((pin - 8) * 4UL));
            port->AFR[1] |= ((alt_val & 0xFUL) << ((pin - 8) * 4UL));
        }
    }
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
    if (port) {
        return ((port->IDR >> pin) & 0x1U);
    }
    return false;
}

hal_err_t gpio_set_interrupt(GPIO_TypeDef* port, gpio_pin_t pin, gpio_edge_trigger_t edge, gpio_exti_cb_t callback, void* arg) {
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
    SYSCFG->EXTICR[reg_idx] = (SYSCFG->EXTICR[reg_idx] & ~(0xFUL << bit_pos)) | (uint32_t)(port_code << bit_pos);

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

    // Register the pin's interrupt handler
    s_gpio_irq_ctx[pin].callback = callback;
    s_gpio_irq_ctx[pin].arg      = arg;

    // Enable the pin's corresponding NVIC irq line
    NVIC_SetPriority(s_exti_irq_lut[pin], EXTI_LINE_NVIC_IRQ_PRIORITY);
    NVIC_ClearPendingIRQ(s_exti_irq_lut[pin]);
    NVIC_EnableIRQ(s_exti_irq_lut[pin]);

    // Clear the EXTI interrupt flag and unmask the EXTI interrupt for the pin
    EXTI->PR = (0b1UL << pin);
    EXTI->IMR |= (0b1UL << pin);

    return HAL_OK;
}

void gpio_clear_interrupt(gpio_pin_t pin) {
    // Extract register index and bit position
    const uint8_t reg_idx = pin / 4;
    const uint8_t bit_pos = (pin % 4) * 4;

    // Clear the pin's bit field in the EXTI configuration register
    SYSCFG->EXTICR[reg_idx] &= ~(0xFUL << bit_pos);

    // Clear both interrupt edge registers
    EXTI->RTSR &= ~(0b1UL << pin);
    EXTI->FTSR &= ~(0b1UL << pin);

    // Clear the interrupt flag
    EXTI->PR = (0b1UL << pin);

    // Remask the interrupts for the pin
    EXTI->IMR &= ~(0b1UL << pin);

    // Clear the pin's interrupt handler
    s_gpio_irq_ctx[pin].callback = NULL;
    s_gpio_irq_ctx[pin].arg      = NULL;
}

void gpio_generate_software_interrupt(gpio_pin_t pin) {
    EXTI->SWIER |= (1UL << pin);
}

IRQn_Type gpio_get_nvic_irq_type(gpio_pin_t pin) {
    return s_exti_irq_lut[pin];
}


// Handle the GPIO EXTI irqs
void EXTI0_IRQHandler(void) {
    gpio_isr_helper(GPIO_PIN_0);
}

void EXTI1_IRQHandler(void) {
    gpio_isr_helper(GPIO_PIN_1);
}

void EXTI2_IRQHandler(void) {
    gpio_isr_helper(GPIO_PIN_2);
}

void EXTI3_IRQHandler(void) {
    gpio_isr_helper(GPIO_PIN_3);
}

void EXTI4_IRQHandler(void) {
    gpio_isr_helper(GPIO_PIN_4);
}

void EXTI9_5_IRQHandler(void) {
    gpio_isr_helper(GPIO_PIN_5);
    gpio_isr_helper(GPIO_PIN_6);
    gpio_isr_helper(GPIO_PIN_7);
    gpio_isr_helper(GPIO_PIN_8);
    gpio_isr_helper(GPIO_PIN_9);
}

void EXTI15_10_IRQHandler(void) {
    gpio_isr_helper(GPIO_PIN_10);
    gpio_isr_helper(GPIO_PIN_11);
    gpio_isr_helper(GPIO_PIN_12);
    gpio_isr_helper(GPIO_PIN_13);
    gpio_isr_helper(GPIO_PIN_14);
    gpio_isr_helper(GPIO_PIN_15);
}
