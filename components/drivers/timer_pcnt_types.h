#ifndef TIMER_PCNT_TYPES_H_
#define TIMER_PCNT_TYPES_H_


#ifdef __cplusplus
extern "C" {
#endif


#include "stm32f411xe.h"
#include "drivers/timer_types.h"
#include "utils/board.h"

#include <stdint.h>
#include <stddef.h>


// Pulse Counter types
typedef enum : uint8_t {
    // The timer is clocked from an input capture pin (can be channel 1 or 2) through the slave mod controller
    PCNT_MODE_1 = 0,
    // The timer is clocked directly by external pulses applied to the ETR pin
    PCNT_MODE_2,
} pcnt_mode_t;


typedef enum : uint8_t {
    PCNT_RISING_EDGE = 0,
    PCNT_FALLING_EDGE,
} pcnt_edge_t;


typedef enum : uint8_t {
    // Equivalent to TIMER_CHANNEL_1 and TIMER_CHANNEL_2
    PCNT_CHANNEL_1 = TIM_CCMR1_CC1S_Pos,
    PCNT_CHANNEL_2 = TIM_CCMR1_CC2S_Pos,
} pcnt_channel_t;


typedef enum : uint8_t {
    PCNT_ETR_DIV_1 = 0b00,
    PCNT_ETR_DIV_2 = 0b01,
    PCNT_ETR_DIV_4 = 0b10,
    PCNT_ETR_DIV_8 = 0b11,
} pcnt_prescaler_t;


typedef struct {
    pcnt_mode_t   mode; // How the timer, channel and the gpio gets configured
    pcnt_edge_t   edge; // What edge to count as a valid pulse
    timer_pull_t  pull; // Use a pullup or down resistor; or none
    tim_clk_div_t clk_div; // The timer clock divider

    // Mode specific settings
    union {
        // Fill in respect to the PCNT mode chosen
        struct {
            pcnt_channel_t channel;
        } mode_1;

        struct {
            pcnt_prescaler_t prescaler;
        } mode_2;
    } settings;

    uint32_t    filter_ns;  // Any pulse shorter than this is ignored
    board_pin_t pulse_gpio; // The physical gpio pin
    // In mode 1, this pin can only be either a BOARD_TIMx_CH1_Pxx or BOARD_TIMx_CH2_Pxx (refer to utils/board.h)
    // In mode 2, this pin can only be a BOARD_TIMx_ETR_Pxx (refer to utils/board.h)

    timer_cb_t callback; // Called on an overflow of the timer's counter
} pcnt_config_t;


// Quadrature decoder (encoder mode) types
typedef enum : uint8_t {
    // The timer counts on the rising and falling edges of only the first channel (2x resolution)
    ENCODER_MODE_1 = (0b01 << TIM_SMCR_SMS_Pos),
    // The timer counts on the rising and falling edges of only the second channel (2x resolution)
    ENCODER_MODE_2 = (0b10 << TIM_SMCR_SMS_Pos),
    // The timer counts on the rising and falling edges of the first and second channels (4x resolution)
    ENCODER_MODE_3 = (0b11 << TIM_SMCR_SMS_Pos),
} encoder_mode_t;


typedef struct {
    encoder_mode_t mode;    // The decoder resolution
    timer_pull_t   pull;    // Use a pullup or down resistor; or none
    tim_clk_div_t  clk_div; // The timer clock divider

    bool     invert_direction; // Invert the rotational polarity/direction
    uint32_t filter_ns;        // Any pulse shorter than this is ignored

    board_pin_t channel_a; // Maps to TIMER_CHANNEL_1
    board_pin_t channel_b; // Maps to TIMER_CHANNEL_2

    timer_cb_t
        callback; // Called on an update event (overflow or underflow of the timer's counter)
} qdec_config_t;


#ifdef __cplusplus
}
#endif


#endif // TIMER_PCNT_TYPES_H_