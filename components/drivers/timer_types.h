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


// General timer types
typedef enum : uint8_t {
    TIMER_COUNTER_UP   = (0b0 << TIM_CR1_DIR_Pos),
    TIMER_COUNTER_DOWN = (0b1 << TIM_CR1_DIR_Pos),
} timer_count_dir_t;

typedef enum : uint8_t {
    TIMER_CHANNEL_1  = 0,
    TIMER_CHANNEL_2  = 1,
    TIMER_CHANNEL_3  = 2,
    TIMER_CHANNEL_4  = 3,
    TIMER_CHANNEL_NC = 0xFF,
} timer_channel_t;

typedef enum : uint8_t {
    TIM_CLK_DIV_1 = 0b00,
    TIM_CLK_DIV_2 = 0b01,
    TIM_CLK_DIV_4 = 0b10,
} tim_clk_div_t;

// The channel parameter only has meaning when the interrupt is a capture compare interrupt
typedef struct {
    void (*cb)(void* arg, timer_channel_t channel);
    void* arg;
} timer_cb_t;

typedef struct {
    // Timer output is active HIGH when this is false, active LOW otherwise
    bool output_polarity;

    // State of the gpio pin when its channel is disabled (only relevant for the advanced timers)
    bool output_idle_state;

    // The actual channel
    timer_channel_t channel;
    board_pin_t     gpio_pin;
} timer_channel_config_t;


// Output Compare types
typedef enum : uint8_t {
    // Edge aligned Modes (CMS = 00)
    OUTPUT_COMPARE_EDGE_LEFT_ALIGNED  = (0b00U << TIM_CR1_CMS_Pos) | (0b0U << TIM_CR1_DIR_Pos), // Upcounting
    OUTPUT_COMPARE_EDGE_RIGHT_ALIGNED = (0b00U << TIM_CR1_CMS_Pos) | (0b1U << TIM_CR1_DIR_Pos), // Downcounting
    // Center aligned Modes (CMS != 00; DIR is controlled by the timer hardware)
    OUTPUT_COMPARE_CENTER_ALIGNED_MODE_1 = (0b01U << TIM_CR1_CMS_Pos), // Center aligned (Interrupt compare flag set only on downcount)
    OUTPUT_COMPARE_CENTER_ALIGNED_MODE_2 = (0b10U << TIM_CR1_CMS_Pos), // Center aligned (Interrupt compare flag set only on upcount)
    OUTPUT_COMPARE_CENTER_ALIGNED_MODE_3 = (0b11U << TIM_CR1_CMS_Pos), // Center aligned (Interrupt compare flag set both on up and downcount)
} output_compare_count_mode_t;

typedef enum : uint8_t {
    OUTPUT_COMPARE_FROZEN          = 0b000, // The channel is ignored on a match
    OUTPUT_COMPARE_HIGH_ON_MATCH   = 0b001, // The channel is forced high on a match
    OUTPUT_COMPARE_LOW_ON_MATCH    = 0b010, // The channel is forced low on a match
    OUTPUT_COMPARE_TOGGLE_ON_MATCH = 0b011, // The channel is toggled on a match
    OUTPUT_COMPARE_FORCE_LOW       = 0b100, // The channel is forced low always, regardless of a match
    OUTPUT_COMPARE_FORCE_HIGH      = 0b101, // The channel is forced high always, regardless of a match
    OUTPUT_COMPARE_PWM_MODE_1      = 0b110, // The channel is forced high when the timer's counter value is less than the compare value
    OUTPUT_COMPARE_PWM_MODE_2      = 0b111, // The channel is forced low when the timer's counter value is less than the compare value
    // NOTE: A match occurs when the timer's counter value is equal to the compare value
} output_compare_mode_t;

typedef struct {
    bool buffer_compare_reload; // Buffer writes to the CCxR registers

    // Only relevant for the advanced timers
    bool ossr; // State of a channel when the timer is active, but that specific channel's output is inactive
    bool ossi; // State of all channels when the timer is inactive

    output_compare_mode_t       mode;
    output_compare_count_mode_t count_mode;

    timer_channel_config_t channels[MAX_TIM2_CHANNELS];
    size_t                 num_channels;

    timer_cb_t callback;
} output_compare_config_t;

// Input Capture types
typedef struct {
    // TODO: Fill in the input cpature struct
} input_capture_config_t;


// Pulse Counter types
typedef struct {
    // TODO: Fill in the pulse counter struct
    board_pin_t pulse_gpio;
} pcnt_config_t;


// Quadrature decoder (encoder mode) types
typedef enum : uint8_t {
    ENCODER_MODE_1 = (0b01 << TIM_SMCR_SMS_Pos), // The timer counts on the rising and falling edges of only the first channel (2x resolution)
    ENCODER_MODE_2 = (0b10 << TIM_SMCR_SMS_Pos), // The timer counts on the rising and falling edges of only the second channel (2x resolution)
    ENCODER_MODE_3 = (0b11 << TIM_SMCR_SMS_Pos), // The timer counts on the rising and falling edges of the first and second channels (4x resolution)
} encoder_mode_t;

typedef enum : uint8_t {
    ENCODER_NONE = 0,
    ENCODER_USE_PULLUP,
    ENCODER_USE_PULLDOWN,
} encoder_pull_t;

typedef struct {
    encoder_mode_t mode;    // The decoder resolution
    encoder_pull_t pull;    // Use a pullup or down resistor
    tim_clk_div_t  clk_div; // The timer clock divider

    bool     invert_direction; // Invert the rotational polarity/direction
    uint32_t filter_ns;        // Any pulse shorter than this is ignored

    board_pin_t channel_a; // Maps to TIMER_CHANNEL_1
    board_pin_t channel_b; // Maps to TIMER_CHANNEL_2
} encoder_config_t;


#ifdef __cplusplus
}
#endif


#endif // TIMER_TYPES_H_