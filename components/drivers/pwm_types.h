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
} pwm_mode_t;


typedef struct {
    board_pin_t   channel_pins[MAX_TIM1_CHANNELS];
    pwm_channel_t channels[MAX_TIM1_CHANNELS];
    size_t        num_of_channels;

    bool       invert_output[MAX_TIM1_CHANNELS];
    pwm_mode_t mode;

    bool        use_inverted_channels;
    board_pin_t inverted_output_pins[MAX_TIM1_COMPLEMENTARY_CHANNELS];
} pwm_advanced_timers_config_t;


typedef struct {
    board_pin_t   channel_pins[MAX_TIM1_CHANNELS];
    pwm_channel_t channels[MAX_TIM1_CHANNELS];
    size_t        num_of_channels;

    bool       invert_output[MAX_TIM1_CHANNELS];
    pwm_mode_t mode;
} pwm_general_timers_config_t;


typedef struct {
    board_pin_t   channel_pins[MAX_TIM1_CHANNELS];
    pwm_channel_t channels[MAX_TIM1_CHANNELS];
    size_t        num_of_channels;

    bool       invert_output[MAX_TIM1_CHANNELS];
    pwm_mode_t mode;
} pwm_lite_timers_config_t;


#endif // PWM_TYPES_H_