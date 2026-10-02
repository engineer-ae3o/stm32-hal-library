#ifndef PWM_TYPES_H_
#define PWM_TYPES_H_


#include "stm32f411xe.h"
#include "drivers/timer.h"
#include "utils/board.h"

#include <stdint.h>
#include <stddef.h>


// TIM1 PWM channels
#define MAX_TIM1_CHANNELS (4U)
#define MAX_TIM1_COMPLEMENTARY_CHANNELS (3U)

// General purpose timers PWM channels
#define MAX_TIM2_CHANNELS (4U)
#define MAX_TIM3_CHANNELS (4U)
#define MAX_TIM4_CHANNELS (4U)
#define MAX_TIM5_CHANNELS (4U)

// Lite timers PWM channels
#define MAX_TIM9_CHANNELS (2U)
#define MAX_TIM10_CHANNELS (1U)
#define MAX_TIM11_CHANNELS (1U)


typedef enum : uint8_t {
    PWM_CHANNEL_0 = 0,
    PWM_CHANNEL_1,
    PWM_CHANNEL_2,
    PWM_CHANNEL_3,
} pwm_channel_t;


typedef enum : uint8_t {
    // Edge aligned Modes (CMS = 00)
    PWM_EDGE_ALIGNED_UPCOUNTING   = (0b00U << TIM_CR1_CMS_Pos) | (0b0U << TIM_CR1_DIR_Pos), // Left aligned PWM
    PWM_EDGE_ALIGNED_DOWNCOUNTING = (0b00U << TIM_CR1_CMS_Pos) | (0b1U << TIM_CR1_DIR_Pos), // Right aligned PWM

    // Center aligned Modes (CMS != 00; DIR is controlled by the timer hardware)
    PWM_CENTER_ALIGNED_MODE_1 = (0b01U << TIM_CR1_CMS_Pos), // Center aligned PWM (Interrupt compare flag set only on downcount)
    PWM_CENTER_ALIGNED_MODE_2 = (0b10U << TIM_CR1_CMS_Pos), // Center aligned PWM (Interrupt compare flag set only on upcount)
    PWM_CENTER_ALIGNED_MODE_3 = (0b11U << TIM_CR1_CMS_Pos), // Center aligned PWM (Interrupt compare flag set both on up and downcount)
} pwm_count_mode_t;


typedef enum : uint8_t {
    TIM_CLK_DIV_1 = 0b00,
    TIM_CLK_DIV_2 = 0b01,
    TIM_CLK_DIV_4 = 0b10,
} pwm_clk_div_t;


typedef enum : uint8_t {
    PWM_MODE_1 = 0b110, // The PWM channel is high when the timer's counter value is less than the duty cycle, but low otherwise
    PWM_MODE_2 = 0b111, // The PWM channel is low when the timer's counter value is less than the duty cycle, but high otherwise
} pwm_mode_t;


typedef enum : uint8_t {
    WP_OFF          = 0b00,
    WP_LOCK_LEVEL_1 = 0b01,
    WP_LOCK_LEVEL_2 = 0b10,
    WP_LOCK_LEVEL_3 = 0b11,
} pwm_write_protection_t;


typedef struct {
    bool invert_output;     // Main channel output active low (when this is true)
    bool output_idle_state; // State of the gpio pin when its channel is disabled (only relevant for the advanced timers)
    // The actual PWM channel and its physical gpio pin
    pwm_channel_t channel;
    board_pin_t   gpio_pin;
} pwm_channel_config_t;


// Advanced Timers Configuration
typedef struct {
    pwm_mode_t       pwm_mode;       // The state of the channels when a capture compare event occurs
    uint8_t          repetition_cnt; // The repetition counter
    pwm_count_mode_t pwm_count_mode; // How the timer counts

    bool ossr; // State of a channel when the timer is active, but that specific channel's output is inactive
    bool ossi; // State of all channels when the timer is inactive

    pwm_clk_div_t clk_div; // The timer clock divider to use when configuring the dead time

    pwm_write_protection_t wp_level; // Write protection on the timer's registers to prevent accidental software writes
    // NOTE: Once set, the timer cannot be reconfigured again till a reset (for the most part). Refer to the TRM for more details.

    // Complementary channels. Depends on num_channels
    bool use_complementary_channels; // Whether or not to use the complementary channels
    struct {
        bool        invert_output;     // Complementary channel output active low (when this is true)
        bool        output_idle_state; // State of the channel when MOE is disabled
        board_pin_t gpio_pin;          // The physical complementary channel's GPIO pin
    } complementary_channels[MAX_TIM1_COMPLEMENTARY_CHANNELS];

    // Dead time insertion
    uint32_t dead_time_ns; // The dead time in nanoseconds

    // Main channels
    pwm_channel_config_t channels[MAX_TIM1_CHANNELS];
    size_t               num_channels;

    // Break protection
    struct {
        bool use_break_input;
        bool active_low; // The break input pin is active when low
        bool auto_rearm; // Restart the PWM automatically

        // The break input gpio pin
        board_pin_t gpio_pin;

        // Callback fired on a break event
        timer_cb_t callback;
        void*      user;
    } break_input;
} pwm_advanced_timer_config_t;


// General Purpose and Lite Timers configuration (can still be used with the advanced timers for minimal configuration)
typedef struct {
    pwm_mode_t       pwm_mode;
    pwm_count_mode_t pwm_count_mode;

    pwm_channel_config_t channels[MAX_TIM2_CHANNELS];
    size_t               num_channels;
} pwm_timer_config_t;


#endif // PWM_TYPES_H_