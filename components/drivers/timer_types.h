#ifndef TIMER_TYPES_H_
#define TIMER_TYPES_H_


#ifdef __cplusplus
extern "C" {
#endif


#include "stm32f411xe.h"
#include "utils/board.h"

#include <stdint.h>
#include <stddef.h>


// TIM1 channels
#define MAX_TIM1_CHANNELS (4U)
#define MAX_TIM1_COMPLEMENTARY_CHANNELS (3U)

// General purpose timers channels
#define MAX_TIM2_CHANNELS (4U)
#define MAX_TIM3_CHANNELS (4U)
#define MAX_TIM4_CHANNELS (4U)
#define MAX_TIM5_CHANNELS (4U)

// Lite timers channels
#define MAX_TIM9_CHANNELS (2U)
#define MAX_TIM10_CHANNELS (1U)
#define MAX_TIM11_CHANNELS (1U)


typedef enum : uint8_t {
    TIMER_COUNTER_UP   = 0b0 << TIM_CR1_DIR_Pos,
    TIMER_COUNTER_DOWN = 0b1 << TIM_CR1_DIR_Pos,
} timer_count_dir_t;

typedef enum : uint8_t {
    TIMER_CHANNEL_1 = 0,
    TIMER_CHANNEL_2,
    TIMER_CHANNEL_3,
    TIMER_CHANNEL_4,
} timer_channel_t;

// The channel parameter only has meaning when the interrupt is a capture compare interrupt
typedef struct {
    void (*cb)(void* arg, timer_channel_t channel);
    void* arg;
} timer_cb_t;


// Output Compare types
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
    TIMER_OC_FROZEN          = 0b000, // The channel is ignored on a match
    TIMER_OC_HIGH_ON_MATCH   = 0b001, // The channel is forced high on a match
    TIMER_OC_LOW_ON_MATCH    = 0b010, // The channel is forced low on a match
    TIMER_OC_TOGGLE_ON_MATCH = 0b011, // The channel is toggled on a match
    TIMER_OC_FORCE_LOW       = 0b100, // The channel is forced low always, regardless of a match
    TIMER_OC_FORCE_HIGH      = 0b101, // The channel is forced high always, regardless of a match
    TIMER_OC_PWM_MODE_1      = 0b110, // The channel is high when the timer's counter value is less than the compare value
    TIMER_OC_PWM_MODE_2      = 0b111, // The channel is low when the timer's counter value is less than the compare value
} timer_oc_mode_t;


typedef struct {
    // Timer output is active HIGH when this is false, active LOW otherwise
    bool output_polarity;

    // State of the gpio pin when its channel is disabled (only relevant for the advanced timers)
    bool output_idle_state;

    // The actual channel
    timer_channel_t channel;
    board_pin_t     gpio_pin;
} timer_channel_config_t;


typedef struct {
    bool buffer_compare_reload; // Buffer writes to the CCxR registers

    // Only relevant for the advanced timers
    bool ossr; // State of a channel when the timer is active, but that specific channel's output is inactive
    bool ossi; // State of all channels when the timer is inactive

    timer_oc_mode_t       mode;
    timer_oc_count_mode_t count_mode;

    timer_channel_config_t channels[MAX_TIM2_CHANNELS];
    size_t                 num_channels;

    timer_cb_t callback;
} timer_oc_config_t;


// Input Capture types


// Pulse Counter types
typedef struct {
} pcnt_config_t;


// Quadrature decoder (encoder mode) types


#ifdef __cplusplus
}
#endif


#endif // TIMER_TYPES_H_