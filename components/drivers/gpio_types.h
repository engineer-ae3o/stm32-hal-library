#ifndef GPIO_TYPES_H_
#define GPIO_TYPES_H_


#include <stdint.h>


typedef void (*gpio_exti_cb_t)(void* arg);

typedef enum : uint8_t {
    GPIO_PIN_0 = 0,
    GPIO_PIN_1,
    GPIO_PIN_2,
    GPIO_PIN_3,
    GPIO_PIN_4,
    GPIO_PIN_5,
    GPIO_PIN_6,
    GPIO_PIN_7,
    GPIO_PIN_8,
    GPIO_PIN_9,
    GPIO_PIN_10,
    GPIO_PIN_11,
    GPIO_PIN_12,
    GPIO_PIN_13,
    GPIO_PIN_14,
    GPIO_PIN_15,
} gpio_pin_t;

typedef enum : uint8_t {
    GPIO_LOW_SPEED = 0,
    GPIO_MEDIUM_SPEED,
    GPIO_HIGH_SPEED,
    GPIO_FULL_SPEED,
} gpio_speed_mode_t;

typedef enum : uint8_t {
    GPIO_RISING_EDGE = 1,
    GPIO_FALLING_EDGE,
    GPIO_RISING_FALLING_EDGE,
} gpio_edge_trigger_t;

typedef enum : uint8_t {
    GPIO_PUSH_PULL = 0,
    GPIO_OPEN_DRAIN,
} gpio_output_type_t;


#endif // GPIO_TYPES_H_