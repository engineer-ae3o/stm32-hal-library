#pragma once


#include "stm32f411xe.h"
#include "Unity/unity.h"

#include "drivers/src/timer_internals.h"
#include "drivers/timer_extended.h"
#include "drivers/timer_types.h"
#include "drivers/pwm_types.h"
#include "drivers/timer.h"
#include "drivers/gpio.h"
#include "drivers/pwm.h"
#include "utils/common.h"
#include "utils/board.h"
#include "utils/clock.h"
#include "utils/tick.h"
#include "utils/err.h"

#include <array>
#include <cstdint>
#include <utility>


// Shared wiring and helpers for the hardware loopback timer tests
// (timer_extended.cpp, timer_pcnt.cpp and the PWM input tests in pwm.cpp).
//
// WIRING:
//         GEN_A -------> IN_A and IN_ETR
//         GEN_B -------> IN_B
//
// TIM3 (16 bit) is the signal source. Either its output compare / PWM output drives the pins (generated clocks
// and PWM), or the CPU takes the pins over as plain GPIO outputs and bit bangs exact pulse trains and
// quadrature sequences, so the expected counts are exact rather than statistical.
// TIM2 (32 bit) is the instrument under test.


namespace test::loopback {

    inline TIM_TypeDef* const GEN   = TIM3;
    inline const board_pin_t  GEN_A = BOARD_TIM3_CH1_PB4;
    inline const board_pin_t  GEN_B = BOARD_TIM3_CH2_PB5;

    inline TIM_TypeDef* const MEAS   = TIM2;
    inline const board_pin_t  IN_A   = BOARD_TIM2_CH1_PA5;
    inline const board_pin_t  IN_B   = BOARD_TIM2_CH2_PB3;
    inline const board_pin_t  IN_ETR = BOARD_TIM2_ETR_PA15;

    // Pin access
    inline uint32_t moder(const board_pin_t& pin) {
        return (pin.port->MODER >> (std::to_underlying(pin.pin) * 2U)) & 0b11U;
    }

    inline uint32_t pupdr(const board_pin_t& pin) {
        return (pin.port->PUPDR >> (std::to_underlying(pin.pin) * 2U)) & 0b11U;
    }

    inline uint32_t afr(const board_pin_t& pin) {
        const uint32_t n = std::to_underlying(pin.pin);
        return (n < 8U) ? ((pin.port->AFR[0] >> (n * 4U)) & 0xFU)
                        : ((pin.port->AFR[1] >> ((n - 8U) * 4U)) & 0xFU);
    }

    // Turn the pin into a plain push pull output driving the given level. The next
    // driver init that touches the pin switches it back to its alternate function
    inline void take_over(const board_pin_t& pin, bool level) {
        TEST_ASSERT_EQUAL(HAL_OK, gpiox_clk_enable(pin.port, true));
        gpio_set_output(pin.port, pin.pin);
        gpio_set_output_type(pin.port, pin.pin, GPIO_PUSH_PULL);
        gpio_set_speed_mode(pin.port, pin.pin, GPIO_FULL_SPEED);
        gpio_set_level(pin.port, pin.pin, level);
    }

    // Fresh start for a test: both timers clocked and reset, both stimulus pins driven low
    inline void reset() {
        for (auto* timer : {GEN, MEAS}) {
            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(timer, true));
            TEST_ASSERT_EQUAL(HAL_OK, timer_deinit(timer));
            TEST_ASSERT_EQUAL(HAL_OK, timer_reset(timer));
        }
        take_over(GEN_A, false);
        take_over(GEN_B, false);
    }

    // Timer prescaler that gives the timer under test 1us ticks
    inline uint16_t ticks_1us() {
        return static_cast<uint16_t>(timer_get_frequency_hz(MEAS) / 1'000'000U);
    }

    // Generate a given number of pulses on the given pin
    inline void
    pulses(const board_pin_t& pin, uint64_t count, uint32_t width_us) {
        for (uint32_t i = 0; i < count; i++) {
            gpio_set_level(pin.port, pin.pin, true);
            delay_us(width_us);
            gpio_set_level(pin.port, pin.pin, false);
            delay_us(width_us);
        }
    }

    // 'count' pulses as short as two back to back writes can make them (tens of ns)
    inline void glitches(const board_pin_t& pin, uint32_t count) {
        for (uint32_t i = 0; i < count; i++) {
            gpio_set_level(pin.port, pin.pin, false);
            gpio_set_level(pin.port, pin.pin, true);
            delay_us(10);
        }
    }

    // 'cycles' full quadrature cycles on GEN_A (channel A) and GEN_B (channel B), starting from A = B = 0.
    // A leading B: 00 > 10 > 11 > 01 > 00. B leading A: 00 > 01 > 11 > 10 > 00
    inline void quad_walk(uint32_t cycles, bool a_leads_b) {
        constexpr auto lead = std::array{true, true, false, false};
        constexpr auto lag  = std::array{false, true, true, false};

        for (uint32_t cycle = 0; cycle < cycles; cycle++) {
            for (uint32_t s = 0; s < 4; s++) {
                gpio_set_level(
                    GEN_A.port, GEN_A.pin, a_leads_b ? lead[s] : lag[s]);
                gpio_set_level(
                    GEN_B.port, GEN_B.pin, a_leads_b ? lag[s] : lead[s]);
                delay_us(5);
            }
        }
    }

    // Transitions seen on a pin over a window
    inline uint32_t count_edges(const board_pin_t& pin, uint32_t duration_us) {
        uint32_t       edges = 0;
        bool           last  = gpio_get_level(pin.port, pin.pin);
        const uint32_t start = DWT->CYCCNT;

        const uint32_t clock_mhz = get_system_core_clock() / 1'000'000U;
        const uint32_t span      = duration_us * clock_mhz;

        while ((DWT->CYCCNT - start) < span) {
            const bool now = gpio_get_level(pin.port, pin.pin);
            edges += (now != last) ? 1U : 0U;
            last = now;
        }
        return edges;
    }

    // Generate a clock of given frequency on channel 1 of TIM3
    inline void toggle_clock(uint32_t frequency_hz) {
        output_compare_config_t cfg{};
        cfg.mode        = OUTPUT_COMPARE_TOGGLE_ON_MATCH;
        cfg.count_mode  = OUTPUT_COMPARE_EDGE_LEFT_ALIGNED;
        cfg.channels[0] = {
            .output_polarity   = false,
            .output_idle_state = false,
            .channel           = TIMER_CHANNEL_1,
            .gpio_pin          = GEN_A,
        };
        cfg.num_channels = 1;

        uint32_t max = 0;
        TEST_ASSERT_EQUAL(HAL_OK, output_compare_init(GEN, &cfg));
        TEST_ASSERT_EQUAL(
            HAL_OK,
            output_compare_start(GEN,
                                 2U * frequency_hz,
                                 &max)); // The output toggles once per period
        TEST_ASSERT_EQUAL(
            HAL_OK, output_compare_set_compare(GEN, TIMER_CHANNEL_1, max / 2U));
    }

    inline void generate_pwm(uint32_t hz, uint32_t duty_percent) {
        pwm_gp_timer_config_t cfg{};
        cfg.pwm_mode       = PWM_MODE_1;
        cfg.pwm_count_mode = OUTPUT_COMPARE_EDGE_LEFT_ALIGNED;
        cfg.channels[0]    = {
            .output_polarity   = false,
            .output_idle_state = false,
            .channel           = TIMER_CHANNEL_1,
            .gpio_pin          = GEN_A,
        };
        cfg.num_channels = 1;

        uint32_t max = 0;
        TEST_ASSERT_EQUAL(HAL_OK, pwm_gp_timer_init(GEN, &cfg));
        TEST_ASSERT_EQUAL(HAL_OK, pwm_start(GEN, hz, &max));
        TEST_ASSERT_EQUAL(HAL_OK,
                          pwm_set_duty_cycle(GEN,
                                             TIMER_CHANNEL_1,
                                             (max * duty_percent) / 100U));
    }

} // namespace test::loopback
