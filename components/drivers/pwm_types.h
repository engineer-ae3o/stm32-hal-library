#ifndef PWM_TYPES_H_
#define PWM_TYPES_H_


#include "stm32f411xe.h"
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
    PWM_CHANNEL_0,
    PWM_CHANNEL_1,
    PWM_CHANNEL_2,
    PWM_CHANNEL_3,
} pwm_channel_t;


typedef enum : uint32_t {
    // Edge aligned Modes (CMS = 00)
    PWM_EDGE_ALIGNED_UPCOUNTING   = (0b00U << TIM_CR1_CMS_Pos) | (0b0U << TIM_CR1_DIR_Pos), // Left aligned PWM
    PWM_EDGE_ALIGNED_DOWNCOUNTING = (0b00U << TIM_CR1_CMS_Pos) | (0b1U << TIM_CR1_DIR_Pos), // Right aligned PWM

    // Center aligned Modes (CMS != 00; DIR is controlled by the timer hardware)
    PWM_CENTER_ALIGNED_MODE_1 = (0b01U << TIM_CR1_CMS_Pos), // Center aligned PWM (Interrupt compare flag set only on downcount)
    PWM_CENTER_ALIGNED_MODE_2 = (0b10U << TIM_CR1_CMS_Pos), // Center aligned PWM (Interrupt compare flag set only on upcount)
    PWM_CENTER_ALIGNED_MODE_3 = (0b11U << TIM_CR1_CMS_Pos), // Center aligned PWM (Interrupt compare flag set both on up and downcount)
} pwm_count_mode_t;


typedef enum : uint8_t {
    PWM_MODE_1 = 0b110, // The PWM channel is high when the timer's counter value is less than the duty cycle, but low otherwise
    PWM_MODE_2 = 0b111, // The PWM channel is low when the timer's counter value is less than the duty cycle, but high otherwise
} pwm_mode_t;


typedef struct {
    pwm_channel_t channel;
    board_pin_t   gpio_pin;
    bool          invert_output; // The PWM output becomes active low when this is true
} pwm_channel_config_t;


// Advanced Timers Configuration
typedef struct {
    pwm_mode_t       pwm_mode;
    pwm_count_mode_t pwm_count_mode;
    uint8_t          repetition_cnt;

    // Main channels
    pwm_channel_config_t channels[MAX_TIM1_CHANNELS];
    size_t               num_channels;

    // Complementary channels. Depends on num_channels
    bool use_complementary_channels; // Whether or not to even use the complementary channels
    struct {
        board_pin_t gpio_pin;        // The complementary pin
        bool        invert_output;   // Complementary pin active low
        bool        idle_state_high; // Idle state when Break/MOE occurs
    } complementary_channels[MAX_TIM1_COMPLEMENTARY_CHANNELS];

    // Dead time Insertion
    uint32_t dead_time_ns; // The dead time in nanoseconds

    // Break protection
    struct {
        bool        use_break_input;
        bool        active_low;
        bool        auto_rearm; // Restart the PWM automatically
        board_pin_t gpio_pin;   // The break input gpio pin
    } break_input;
} pwm_advanced_timer_config_t;


// General Purpose & Lite Timers Configuration
typedef struct {
    pwm_mode_t       pwm_mode;
    pwm_count_mode_t pwm_count_mode;

    pwm_channel_config_t channels[MAX_TIM2_CHANNELS];
    size_t               num_channels;
} pwm_timer_config_t;


#endif // PWM_TYPES_H_