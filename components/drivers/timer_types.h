#ifndef TIMER_TYPES_H_
#define TIMER_TYPES_H_


#ifdef __cplusplus
extern "C" {
#endif


#include "stm32f411xe.h"
#include "drivers/pwm_types.h"
#include "drivers/timer.h"
#include "utils/board.h"

#include <stdint.h>
#include <stddef.h>


// The extended timer module (types file)
// Implements extra functionality from the timer(s)


// Output Compare
typedef enum : uint8_t {
    TIMER_OC_CHANNEL_1,
    TIMER_OC_CHANNEL_2,
    TIMER_OC_CHANNEL_3,
    TIMER_OC_CHANNEL_4,
} timer_oc_channel_t;

typedef enum : uint8_t {
    // Edge aligned Modes (CMS = 00)
    TIMER_OC_EDGE_LEFT_ALIGNED  = (0b00U << TIM_CR1_CMS_Pos) | (0b0U << TIM_CR1_DIR_Pos), // Upcounting
    TIMER_OC_EDGE_RIGHT_ALIGNED = (0b00U << TIM_CR1_CMS_Pos) | (0b1U << TIM_CR1_DIR_Pos), // Downcounting

    // Center aligned Modes (CMS != 00; DIR is controlled by the timer hardware)
    TIMER_OC_CENTER_ALIGNED_MODE_1 = (0b01U << TIM_CR1_CMS_Pos), // Center aligned (Interrupt compare flag set only on downcount)
    TIMER_OC_CENTER_ALIGNED_MODE_2 = (0b10U << TIM_CR1_CMS_Pos), // Center aligned (Interrupt compare flag set only on upcount)
    TIMER_OC_CENTER_ALIGNED_MODE_3 = (0b11U << TIM_CR1_CMS_Pos), // Center aligned (Interrupt compare flag set both on up and downcount)
} timer_oc_count_mode_t;

typedef enum : uint8_t {
    TIMER_OC_FROZEN          = 0b000, // The pin is ignored on a match
    TIMER_OC_HIGH_ON_MATCH   = 0b001, // The pin is forced high on a match
    TIMER_OC_LOW_ON_MATCH    = 0b010, // The pin is forced low on a match
    TIMER_OC_TOGGLE_ON_MATCH = 0b011, // The pin is toggled on a match
    TIMER_OC_FORCE_LOW       = 0b100, // The pin is forced low always, regardless of a match
    TIMER_OC_FORCE_HIGH      = 0b101, // The pin is forced high always, regardless of a match
} timer_oc_mode_t;

typedef struct {
    bool output_polarity; // Timer output is active HIGH when this is false, active LOW otherwise
    // The actual channel
    timer_oc_channel_t channel;
    board_pin_t        gpio_pin;
} timer_oc_channel_config_t;

typedef struct {
    bool buffer_compare_reload;

    timer_oc_mode_t       mode;
    timer_oc_count_mode_t count_mode;

    timer_oc_channel_config_t channels[MAX_TIM2_CHANNELS];
    size_t                    num_channels;

    timer_cb_t callback;
    void*      arg;
} timer_oc_config_t;


// Input Capture


// Pulse Counter


// Quadrature decoder (encoder mode)


#ifdef __cplusplus
}
#endif


#endif // TIMER_TYPES_H_