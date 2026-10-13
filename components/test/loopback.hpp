#pragma once


#include "stm32f411xe.h"
#include "Unity/unity.h"

#include "drivers/src/timer_internals.h"
#include "drivers/timer_extended.h"
#include "drivers/timer_types.h"
#include "drivers/pwm_types.h"
#include "drivers/timer.h"
#include "utils/common.h"
#include "drivers/gpio.h"
#include "drivers/pwm.h"
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
// WIRING: one jumper wire each. Change the pins below if they clash with something on your board.
//
//   PA0 (TIM5_CH1, "generator A") ---> PB4 (TIM3_CH1, measurer channel 1)
//                                  \-> PD2 (TIM3_ETR, measurer external trigger input)
//   PA1 (TIM5_CH2, "generator B") ---> PB5 (TIM3_CH2, measurer channel 2)
//
// TIM5 (32 bit) is the signal source. Either its output compare / PWM output drives the pins (generated clocks
// and PWM), or the CPU takes the pins over as plain GPIO outputs and bit bangs exact pulse trains and
// quadrature sequences, so the expected counts are exact rather than statistical.
// TIM3 (16 bit) is the instrument under test.
//
// NOTE: These tests assume every clock in the tree is a multiple of 1MHz, so that a prescaler of
// (timer clock / 1MHz) gives exact 1us ticks.

namespace test::loopback {

    inline TIM_TypeDef* const GEN  = TIM5;
    inline TIM_TypeDef* const MEAS = TIM3;

    inline const board_pin_t GEN_A  = BOARD_TIM5_CH1_PA0;
    inline const board_pin_t GEN_B  = BOARD_TIM5_CH2_PA1;
    inline const board_pin_t IN_A   = BOARD_TIM3_CH1_PB4;
    inline const board_pin_t IN_B   = BOARD_TIM3_CH2_PB5;
    inline const board_pin_t IN_ETR = BOARD_TIM3_ETR_PD2;

    // How long each quadrature state is held. Comfortably above any filter the tests ask for
    inline constexpr uint32_t STEP_US = 4;


    // GPIO register helpers
    inline uint32_t pin_number(const board_pin_t& pin) {
        return std::to_underlying(pin.pin);
    }

    inline uint32_t moder(const board_pin_t& pin) {
        return (pin.port->MODER >> (pin_number(pin) * 2U)) & 0b11U;
    }

    inline uint32_t pupdr(const board_pin_t& pin) {
        return (pin.port->PUPDR >> (pin_number(pin) * 2U)) & 0b11U;
    }

    inline uint32_t afr(const board_pin_t& pin) {
        const uint32_t n = pin_number(pin);
        return (n < 8U) ? ((pin.port->AFR[0] >> (n * 4U)) & 0xFU) : ((pin.port->AFR[1] >> ((n - 8U) * 4U)) & 0xFU);
    }

    inline void drive(const board_pin_t& pin, bool level) {
        pin.port->BSRR = 1UL << (pin_number(pin) + (level ? 0U : 16U));
    }

    inline bool sample(const board_pin_t& pin) {
        return ((pin.port->IDR >> pin_number(pin)) & 1U) != 0;
    }

    // Turn the pin into a plain push pull output driving the given level. The next driver init that
    // touches the pin switches it back to its alternate function
    inline void take_over_pin(const board_pin_t& pin, bool level) {
        TEST_ASSERT_EQUAL(HAL_OK, gpiox_clk_enable(pin.port, true));
        const uint32_t n = pin_number(pin);
        drive(pin, level);
        pin.port->OTYPER &= ~(1UL << n);
        pin.port->OSPEEDR |= (0b11UL << (n * 2U));
        pin.port->PUPDR &= ~(0b11UL << (n * 2U));
        pin.port->MODER = (pin.port->MODER & ~(0b11UL << (n * 2U))) | (0b01UL << (n * 2U));
    }

    // Park the pin as an input with a pulldown, so a wire hanging off a previous test's output idles low
    inline hal_err_t park_pin(const board_pin_t& pin) {
        TRY(gpiox_clk_enable(pin.port, true));
        const uint32_t n = pin_number(pin);
        pin.port->MODER &= ~(0b11UL << (n * 2U));
        pin.port->PUPDR = (pin.port->PUPDR & ~(0b11UL << (n * 2U))) | (0b10UL << (n * 2U));
        return HAL_OK;
    }


    // Bit banged stimulus
    // 'count' full pulses: high for high_us, then low for low_us
    inline void pulses(const board_pin_t& pin, uint32_t count, uint32_t high_us, uint32_t low_us) {
        for (uint32_t i = 0; i < count; i++) {
            drive(pin, true);
            delay_us(high_us);
            drive(pin, false);
            delay_us(low_us);
        }
    }

    // 'count' pulses as short as two back to back BSRR writes allow (tens of ns): below any filter these tests use
    inline void glitches(const board_pin_t& pin, uint32_t count) {
        for (uint32_t i = 0; i < count; i++) {
            drive(pin, true);
            drive(pin, false);
            delay_us(10);
        }
    }

    // Walk GEN_A (channel A) and GEN_B (channel B) through 'cycles' full quadrature cycles starting from (A=0, B=0).
    // a_leads_b: A rises first (the sequence 00 > 10 > 11 > 01 > 00), otherwise B leads (00 > 01 > 11 > 10 > 00)
    inline void quad_walk(uint32_t cycles, bool a_leads_b) {
        static constexpr bool lead[4] = {true, true, false, false};
        static constexpr bool lag[4]  = {false, true, true, false};
        for (uint32_t c = 0; c < cycles; c++) {
            for (uint32_t s = 0; s < 4; s++) {
                drive(GEN_A, a_leads_b ? lead[s] : lag[s]);
                drive(GEN_B, a_leads_b ? lag[s] : lead[s]);
                delay_us(STEP_US);
            }
        }
    }

    // Count the transitions seen on a pin over a window, sampling in a tight loop
    inline uint32_t count_edges(const board_pin_t& pin, uint32_t duration_us) {
        uint32_t       edges = 0;
        bool           last  = sample(pin);
        const uint32_t t0    = DWT->CYCCNT;
        const uint32_t span  = duration_us * (get_system_core_clock() / 1'000'000U);
        while ((DWT->CYCCNT - t0) < span) {
            const bool now = sample(pin);
            if (now != last) {
                edges += 1;
                last = now;
            }
        }
        return edges;
    }


    // Signal source (TIM5)
    // A square wave of the given frequency on 'pin': the output toggles on a compare match once per timer period
    inline hal_err_t generator_toggle_start(timer_channel_t channel, const board_pin_t& pin, uint32_t pin_hz) {
        output_compare_config_t cfg{};
        cfg.buffer_compare_reload = false;
        cfg.mode                  = OUTPUT_COMPARE_TOGGLE_ON_MATCH;
        cfg.count_mode            = OUTPUT_COMPARE_EDGE_LEFT_ALIGNED;
        cfg.channels[0]           = {.output_polarity = false, .output_idle_state = false, .channel = channel, .gpio_pin = pin};
        cfg.num_channels          = 1;

        TRY(output_compare_init(GEN, &cfg));

        // The pin toggles once per period, so the period frequency has to be twice the pin frequency
        uint32_t max_compare = 0;
        TRY(output_compare_start(GEN, 2U * pin_hz, &max_compare));
        return output_compare_set_compare(GEN, channel, max_compare / 2U);
    }

    // Frequency actually coming out of the generator, derived from its registers
    inline uint32_t generator_pin_hz() {
        const uint64_t divisor = (uint64_t)(GEN->PSC + 1U) * ((uint64_t)GEN->ARR + 1U) * 2U;
        return (uint32_t)(((uint64_t)timer_get_frequency_hz(GEN) + (divisor / 2U)) / divisor);
    }

    // A PWM wave on GEN_A. 'compare' and 'max_duty' are the duty cycle numbers the PWM driver ended up using
    inline hal_err_t generator_pwm_start(uint32_t pwm_hz, uint32_t duty_percent, uint32_t* compare, uint32_t* max_duty) {
        pwm_gp_timer_config_t cfg{};
        cfg.pwm_mode       = PWM_MODE_1;
        cfg.pwm_count_mode = OUTPUT_COMPARE_EDGE_LEFT_ALIGNED;
        cfg.channels[0]    = {.output_polarity = false, .output_idle_state = false, .channel = TIMER_CHANNEL_1, .gpio_pin = GEN_A};
        cfg.num_channels   = 1;

        TRY(pwm_gp_timer_init(GEN, &cfg));
        TRY(pwm_start(GEN, pwm_hz, max_duty));
        *compare = (uint32_t)((uint64_t)(*max_duty) * duty_percent / 100U);
        return pwm_set_duty_cycle(GEN, TIMER_CHANNEL_1, *compare);
    }

    inline void generator_stop() {
        (void)output_compare_deinit(GEN);
    }


    // Instrument helpers
    // Timer prescaler that makes the measurer count in exact 1us ticks
    inline uint32_t prescaler_for_1us(TIM_TypeDef* handle) {
        return timer_get_frequency_hz(handle) / 1'000'000U;
    }

    inline uint32_t cycles_to_us32(uint32_t cycles, uint32_t sysclk_hz) {
        return (uint32_t)((uint64_t)cycles * 1'000'000ULL / sysclk_hz);
    }


    // Fixture
    // Clocks on, both timers back to their reset state, loopback pins parked low.
    // The _hal flavor never asserts, so it is safe to use while the system clock is switched
    inline hal_err_t reset_fixture_hal() {
        TRY(timx_clk_enable(GEN, true));
        TRY(timx_clk_enable(MEAS, true));
        TRY(timer_deinit(GEN));
        TRY(timer_reset(GEN));
        TRY(timer_deinit(MEAS));
        TRY(timer_reset(MEAS));
        TRY(park_pin(GEN_A));
        TRY(park_pin(GEN_B));
        return HAL_OK;
    }

    inline void reset_fixture() {
        TEST_ASSERT_EQUAL(HAL_OK, reset_fixture_hal());
    }

    // Poll until the predicate holds or the timeout (in ms) runs out
    template<typename predicate>
    bool wait_for(predicate pred, uint32_t timeout_ms) {
        const uint64_t t0 = ms_since_boot();
        while (!pred() && ((ms_since_boot() - t0) < timeout_ms)) {
        }
        return pred();
    }


    // System clock sweep. Changing the system clock moves the timer clock, and the SysTick/DWT based time
    // bases used to cross check the timers are independent of the timers' own clock arithmetic.
    // NOTE: TEST_ASSERT_* longjmps out of the test on failure and C++ destructors would not run, so there is
    // deliberately no RAII guard here. Measure with the non asserting _hal helpers while switched, restore the
    // clock, and only then assert on the recorded results.
    struct clock_preset_t {
        system_clock_t hse;
        system_clock_t hsi;
        uint32_t       mhz;
    };

    inline constexpr std::array<clock_preset_t, 3> CLOCK_SWEEP = {{
        {HSE_PLL_48MHz, HSI_PLL_48MHz, 48},
        {HSE_PLL_84MHz, HSI_PLL_84MHz, 84},
        {HSE_PLL_100MHz, HSI_PLL_100MHz, 100},
    }};

    // Pick the preset from the same oscillator family the board is currently running on
    inline system_clock_t pick_preset(const clock_preset_t& preset, system_clock_t original) {
        return (original <= HSE_PLL_DIRECT) ? preset.hse : preset.hsi;
    }

    inline void switch_clock(system_clock_t clock) {
        system_core_clock_config(clock);
        systick_init();
    }

} // namespace test::loopback
