#ifndef PWM_TYPES_H_
#define PWM_TYPES_H_


#include "stm32f411xe.h"
#include "drivers/timer_types.h"
#include "utils/board.h"

#include <stdint.h>
#include <stddef.h>


typedef enum : uint8_t {
    PWM_MODE_1 = OUTPUT_COMPARE_PWM_MODE_1, // The PWM channel is high when the timer's counter value is less than the duty cycle, but low otherwise
    PWM_MODE_2 = OUTPUT_COMPARE_PWM_MODE_2, // The PWM channel is low when the timer's counter value is less than the duty cycle, but high otherwise
} pwm_mode_t;


typedef enum : uint8_t {
    WP_OFF          = 0b00,
    WP_LOCK_LEVEL_1 = 0b01,
    WP_LOCK_LEVEL_2 = 0b10,
    WP_LOCK_LEVEL_3 = 0b11,
} pwm_write_protection_t;


// General Purpose and Lite Timers configuration (can still be used with the advanced timers for minimal configuration)
typedef struct {
    // How the timer hardware reacts to a compare event
    pwm_mode_t pwm_mode;
    // How the timer counts and in what direction
    output_compare_count_mode_t pwm_count_mode;

    timer_channel_config_t channels[MAX_TIM2_CHANNELS];
    size_t                 num_channels;
} pwm_gp_timer_config_t;


// Advanced Timers Configuration
typedef struct {
    // The state of the channels when a capture compare event occurs
    pwm_mode_t pwm_mode;
    // The repetition counter
    uint8_t repetition_cnt;
    // How the timer counts and in what direction
    output_compare_count_mode_t pwm_count_mode;

    bool ossr; // State of a channel when the timer is active, but that specific channel's output is inactive
    bool ossi; // State of all channels when the timer is inactive

    tim_clk_div_t clk_div; // The timer clock divider to use when configuring the dead time

    pwm_write_protection_t wp_level; // Write protection on the timer's registers to prevent accidental software writes
    // NOTE: Once set, the timer cannot be reconfigured again till a reset (for the most part). Refer to the TRM for more details.

    // Complementary channels. Depends on num_channels
    bool use_complementary_channels; // Whether or not to use the complementary channels
    struct {
        bool        output_polarity;   // Complementary channel output active low (when this is true)
        bool        output_idle_state; // State of the channel when MOE is disabled
        board_pin_t gpio_pin;          // The physical complementary channel's GPIO pin
    } complementary_channels[MAX_TIM1_COMPLEMENTARY_CHANNELS];

    // Dead time insertion
    uint32_t dead_time_ns; // The dead time in nanoseconds

    // Main channels
    timer_channel_config_t channels[MAX_TIM1_CHANNELS];
    size_t                 num_channels;

    // Break protection
    struct {
        bool use_break_input;
        bool active_low; // The break input pin is active when low
        bool auto_rearm; // Restart the PWM automatically

        // The break input gpio pin
        board_pin_t gpio_pin;

        // Callback fired on a break event
        timer_cb_t callback;
    } break_input;
} pwm_advanced_timer_config_t;


#endif // PWM_TYPES_H_