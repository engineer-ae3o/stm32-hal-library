#include "stm32f411xe.h"
#include "Unity/unity.h"

#include "drivers/src/timer_internals.h"
#include "drivers/timer_extended.h"
#include "test/timer_extended.hpp"
#include "drivers/timer_types.h"
#include "test/loopback.hpp"
#include "drivers/timer.h"
#include "utils/board.h"
#include "utils/tick.h"
#include "utils/err.h"
#include "utils/log.h"

#include <array>
#include <cstdint>
#include <utility>


// Input capture, output compare (toggle mode) and the filter code helper.
// Wiring and the signal sources are in test/loopback.hpp.
//
// ASSUMPTION: input_capture_config_t has the new 'timer_prescaler' field (PSC = timer_prescaler - 1, 0 and
// TIMER_COUNTER_DIR_NONE are HAL_ERR_INVALID_ARG) and input_capture_get_timer_frequency(...) returns the tick rate.

namespace test::timer_ext {

    namespace {

        constexpr const char* TAG = "Timer_ext_Test";

        namespace lb = test::loopback;

        TIM_TypeDef* const MEAS = lb::MEAS;
        TIM_TypeDef* const GEN  = lb::GEN;

        // Callbacks count into whatever the test hands them, so no test shares anything with another
        struct counter_t {
            volatile uint32_t n = 0;
        };

        void count_cb(void* arg, timer_channel_t, timer_count_dir_t) {
            auto* counter = static_cast<counter_t*>(arg);
            counter->n    = counter->n + 1;
        }

        // Channel 1 on PB4, 1us ticks, no filter, no callback
        input_capture_config_t ic_config(ic_edge_t edge = IC_EDGE_RISING) {
            input_capture_config_t cfg{};
            cfg.clk_div         = TIM_CLK_DIV_1;
            cfg.count_mode      = TIMER_COUNTER_DIR_UP;
            cfg.timer_prescaler = lb::ticks_1us();
            cfg.channels[0]     = {
                .edge      = edge,
                .pull      = TIMER_PULL_NONE,
                .prescaler = IC_PRESCALER_DIV_1,
                .channel   = TIMER_CHANNEL_1,
                .gpio_pin  = lb::IN_A,
                .filter_ns = 0,
            };
            cfg.num_channels = 1;
            return cfg;
        }

        void start_capture(const input_capture_config_t& cfg) {
            TEST_ASSERT_EQUAL(HAL_OK, input_capture_init(MEAS, &cfg));
            TEST_ASSERT_EQUAL(HAL_OK, input_capture_start(MEAS));
        }

        // Wait for the next channel 1 capture (polling, no callback needed) and return its timestamp
        uint32_t next_capture() {
            const uint64_t t0 = ms_since_boot();
            while (!(MEAS->SR & TIM_SR_CC1IF)) {
                TEST_ASSERT_TRUE_MESSAGE(
                    (ms_since_boot() - t0) < 50,
                    "No capture within 50ms. Is the loopback wire connected?");
            }
            uint32_t value = 0;
            TEST_ASSERT_EQUAL(
                HAL_OK,
                input_capture_get_sample(MEAS, TIMER_CHANNEL_1, &value));
            return value;
        }

        // Ticks between two timestamps of the 16 bit counter
        uint32_t ticks_between(uint32_t first, uint32_t second) {
            return (second - first) & UINT16_MAX;
        }

        // Input capture: init and configuration
        void ic_init_rejects_invalid_arguments() {
            lb::reset();
            const auto good = ic_config();
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG,
                              input_capture_init(nullptr, &good));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG,
                              input_capture_init(MEAS, nullptr));

            const auto invalid = [&](auto mutate) {
                auto cfg = good;
                mutate(cfg);
                TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG,
                                  input_capture_init(MEAS, &cfg));
            };
            invalid([](auto& cfg) {
                cfg.num_channels = 0;
            });
            invalid([](auto& cfg) {
                cfg.num_channels = 5;
            });
            invalid([](auto& cfg) {
                cfg.timer_prescaler = 0;
            });
            invalid([](auto& cfg) {
                cfg.count_mode = TIMER_COUNTER_DIR_NONE;
            });
            invalid([](auto& cfg) {
                cfg.channels[0].channel = TIMER_CHANNEL_NC;
            });

            // TIM9 only has two channels
            auto lite                = good;
            lite.channels[0].channel = TIMER_CHANNEL_3;
            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM9, true));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG,
                              input_capture_init(TIM9, &lite));
            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM9, false));
        }

        void ic_init_fails_on_a_locked_advanced_timer() {
            const auto cfg = ic_config();
            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM1, true));
            TEST_ASSERT_EQUAL(HAL_OK, timer_reset(TIM1));
            TIM1->BDTR |= TIM_BDTR_LOCK;
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_STATE,
                              input_capture_init(TIM1, &cfg));
            TEST_ASSERT_EQUAL(HAL_OK, timer_reset(TIM1));
            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM1, false));
        }

        void ic_timer_prescaler_sets_the_tick_rate() {
            lb::reset();
            const uint32_t clk = timer_get_frequency_hz(MEAS);

            auto cfg            = ic_config();
            cfg.timer_prescaler = 4;
            TEST_ASSERT_EQUAL(HAL_OK, input_capture_init(MEAS, &cfg));
            TEST_ASSERT_EQUAL_UINT32(3U, MEAS->PSC);

            uint32_t hz = 0;
            TEST_ASSERT_EQUAL(HAL_OK,
                              input_capture_get_timer_frequency(MEAS, &hz));
            TEST_ASSERT_EQUAL_UINT32(clk / 4U, hz);

            cfg = ic_config(); // 1us ticks
            TEST_ASSERT_EQUAL(HAL_OK, input_capture_init(MEAS, &cfg));
            TEST_ASSERT_EQUAL(HAL_OK,
                              input_capture_get_timer_frequency(MEAS, &hz));
            TEST_ASSERT_EQUAL_UINT32(1'000'000U, hz);

            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG,
                              input_capture_get_timer_frequency(nullptr, &hz));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG,
                              input_capture_get_timer_frequency(MEAS, nullptr));
        }

        void ic_init_configures_the_gpio_pin() {
            lb::reset();
            auto cfg             = ic_config();
            cfg.channels[0].pull = TIMER_USE_PULLUP;
            TEST_ASSERT_EQUAL(HAL_OK, input_capture_init(MEAS, &cfg));

            TEST_ASSERT_EQUAL_UINT32(0b10U,
                                     lb::moder(lb::IN_A)); // Alternate function
            TEST_ASSERT_EQUAL_UINT32(lb::IN_A.af, lb::afr(lb::IN_A));
            TEST_ASSERT_EQUAL_UINT32(0b01U, lb::pupdr(lb::IN_A));
        }

        void ic_start_and_stop_run_and_freeze_the_counter() {
            lb::reset();
            const auto cfg = ic_config();
            TEST_ASSERT_EQUAL(HAL_OK, input_capture_init(MEAS, &cfg));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG,
                              input_capture_start(nullptr));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, input_capture_stop(nullptr));

            TEST_ASSERT_FALSE(MEAS->CR1 & TIM_CR1_CEN);
            TEST_ASSERT_EQUAL(HAL_OK, input_capture_start(MEAS));
            delay_us(100);
            TEST_ASSERT_TRUE(MEAS->CNT > 0U);

            TEST_ASSERT_EQUAL(HAL_OK, input_capture_stop(MEAS));
            const uint32_t held = MEAS->CNT;
            delay_us(100);
            TEST_ASSERT_EQUAL_UINT32(held, MEAS->CNT);
        }

        // Input capture: measuring a signal
        void ic_measures_the_period_of_a_generated_clock() {
            lb::reset();
            lb::toggle_clock(2'000); // 500us period
            start_capture(ic_config());

            const uint32_t first  = next_capture();
            const uint32_t second = next_capture();
            const uint32_t third  = next_capture();
            TEST_ASSERT_UINT32_WITHIN(1U, 500U, ticks_between(first, second));
            TEST_ASSERT_UINT32_WITHIN(
                1U, 500U, ticks_between(second, third)); // NOLINT
        }

        void ic_captures_only_the_selected_edge() {
            for (const auto edge : {IC_EDGE_RISING, IC_EDGE_FALLING}) {
                lb::reset();
                lb::generate_pwm(1'000, 25); // 250us high, 750us low
                start_capture(ic_config(edge));

                // The pin is read right after the capture, long before the next edge
                (void)next_capture();
                TEST_ASSERT_EQUAL(edge == IC_EDGE_RISING,
                                  gpio_get_level(lb::IN_A.port, lb::IN_A.pin));
                (void)next_capture();
                TEST_ASSERT_EQUAL(edge == IC_EDGE_RISING,
                                  gpio_get_level(lb::IN_A.port, lb::IN_A.pin));
            }
        }

        void ic_both_edges_capture_the_high_and_low_times() {
            lb::reset();
            lb::generate_pwm(1'000, 25);
            start_capture(ic_config(IC_EDGE_RISING_FALLING));

            const uint32_t first  = next_capture();
            const uint32_t second = next_capture();
            const uint32_t third  = next_capture();
            const uint32_t gap_1  = ticks_between(first, second);
            const uint32_t gap_2  = ticks_between(second, third); // NOLINT

            // One gap is the 250us high time, the other the 750us low time
            TEST_ASSERT_UINT32_WITHIN(2U, 1'000U, gap_1 + gap_2);
            TEST_ASSERT_UINT32_WITHIN(
                2U, 500U, (gap_1 > gap_2) ? (gap_1 - gap_2) : (gap_2 - gap_1));
        }

        void ic_digital_filter_rejects_glitches() {
            lb::reset();
            counter_t events;
            auto      cfg             = ic_config();
            cfg.callback              = {count_cb, &events};
            cfg.channels[0].filter_ns = 500;
            start_capture(cfg);

            lb::glitches(lb::GEN_A, 50);
            delay_us(50);
            TEST_ASSERT_EQUAL_UINT32(0U, events.n);

            lb::pulses(lb::GEN_A, 50, 20);
            delay_us(50);
            TEST_ASSERT_EQUAL_UINT32(50U, events.n);
        }

        void ic_input_prescaler_divides_the_event_rate() {
            for (const auto& [prescaler, divisor] :
                 std::array<std::pair<ic_prescaler_t, uint32_t>, 3>{{
                     {IC_PRESCALER_DIV_2, 2},
                     {IC_PRESCALER_DIV_4, 4},
                     {IC_PRESCALER_DIV_8, 8},
                 }}) {
                lb::reset();
                counter_t events;
                auto      cfg             = ic_config();
                cfg.callback              = {count_cb, &events};
                cfg.channels[0].prescaler = prescaler;
                start_capture(cfg);

                lb::pulses(lb::GEN_A, 64, 20);
                delay_us(50);
                TEST_ASSERT_EQUAL_UINT32(64U / divisor, events.n);
            }
        }

        void ic_reports_overcapture_once() {
            lb::reset();
            lb::toggle_clock(2'000);
            start_capture(ic_config());
            delay_ms(
                5); // Several edges with nobody reading the capture register

            uint32_t value = 0;
            TEST_ASSERT_EQUAL(
                HAL_ERR_TIM_OVER_CAPTURE,
                input_capture_get_sample(MEAS, TIMER_CHANNEL_1, &value));
            TEST_ASSERT_EQUAL(
                HAL_OK,
                input_capture_get_sample(MEAS, TIMER_CHANNEL_1, &value));
        }

        void ic_get_sample_rejects_invalid_arguments() {
            uint32_t value = 0;
            TEST_ASSERT_EQUAL(
                HAL_ERR_INVALID_ARG,
                input_capture_get_sample(nullptr, TIMER_CHANNEL_1, &value));
            TEST_ASSERT_EQUAL(
                HAL_ERR_INVALID_ARG,
                input_capture_get_sample(MEAS, TIMER_CHANNEL_1, nullptr));
            TEST_ASSERT_EQUAL(
                HAL_ERR_INVALID_ARG,
                input_capture_get_sample(MEAS, TIMER_CHANNEL_NC, &value));
            TEST_ASSERT_EQUAL(
                HAL_ERR_NOT_SUPPORTED,
                input_capture_get_sample(TIM10, TIMER_CHANNEL_2, &value));
            TEST_ASSERT_EQUAL(
                HAL_ERR_NOT_SUPPORTED,
                input_capture_get_sample(TIM9, TIMER_CHANNEL_3, &value));
        }

        void ic_deinit_stops_the_events() {
            lb::reset();
            counter_t events;
            auto      cfg = ic_config();
            cfg.callback  = {count_cb, &events};
            start_capture(cfg);

            lb::pulses(lb::GEN_A, 10, 20);
            delay_us(50);
            TEST_ASSERT_EQUAL_UINT32(10U, events.n);

            TEST_ASSERT_EQUAL(HAL_OK, input_capture_deinit(MEAS));
            TEST_ASSERT_FALSE(MEAS->CR1 & TIM_CR1_CEN);
            lb::pulses(lb::GEN_A, 10, 20);
            delay_us(50);
            TEST_ASSERT_EQUAL_UINT32(10U, events.n);
        }

        // Filter code helper (shared by input capture, the counters and PWM input)
        void filter_code_grows_with_the_requested_time() {
            lb::reset();
            uint32_t none = 0xFF, small = 0, large = 0;
            TEST_ASSERT_EQUAL(
                HAL_OK,
                timer_filter_ns_to_ic_code(MEAS, TIM_CLK_DIV_1, 0, &none));
            TEST_ASSERT_EQUAL(
                HAL_OK,
                timer_filter_ns_to_ic_code(MEAS, TIM_CLK_DIV_1, 100, &small));
            TEST_ASSERT_EQUAL(
                HAL_OK,
                timer_filter_ns_to_ic_code(MEAS, TIM_CLK_DIV_1, 1'000, &large));
            TEST_ASSERT_EQUAL_UINT32(
                0U, none); // No filter requested, none programmed
            TEST_ASSERT_TRUE(small > 0U);
            TEST_ASSERT_TRUE(large > small);

            uint32_t code = 0;
            TEST_ASSERT_EQUAL(HAL_ERR_NOT_SUPPORTED,
                              timer_filter_ns_to_ic_code(
                                  MEAS, TIM_CLK_DIV_1, 1'000'000'000U, &code));
            TEST_ASSERT_EQUAL(
                HAL_ERR_INVALID_ARG,
                timer_filter_ns_to_ic_code(nullptr, TIM_CLK_DIV_1, 100, &code));
            TEST_ASSERT_EQUAL(
                HAL_ERR_INVALID_ARG,
                timer_filter_ns_to_ic_code(MEAS, TIM_CLK_DIV_1, 100, nullptr));
            TEST_ASSERT_EQUAL(
                HAL_ERR_INVALID_ARG,
                timer_filter_ns_to_ic_code(
                    MEAS, static_cast<tim_clk_div_t>(3), 100, &code));
        }

        // Output compare, toggle mode (the PWM modes are covered by the PWM tests)
        void oc_compare_callback_fires_once_per_period() {
            lb::reset();
            counter_t events;

            output_compare_config_t cfg{};
            cfg.mode         = OUTPUT_COMPARE_TOGGLE_ON_MATCH;
            cfg.count_mode   = OUTPUT_COMPARE_EDGE_LEFT_ALIGNED;
            cfg.channels[0]  = {.output_polarity   = false,
                                .output_idle_state = false,
                                .channel           = TIMER_CHANNEL_1,
                                .gpio_pin          = lb::GEN_A};
            cfg.num_channels = 1;
            cfg.callback     = {count_cb, &events};

            uint32_t max = 0;
            TEST_ASSERT_EQUAL(HAL_OK, output_compare_init(GEN, &cfg));
            TEST_ASSERT_EQUAL(HAL_OK, output_compare_start(GEN, 1'000, &max));
            TEST_ASSERT_EQUAL(
                HAL_OK,
                output_compare_set_compare(GEN, TIMER_CHANNEL_1, max / 2U));

            delay_ms(100);
            TEST_ASSERT_UINT32_WITHIN(
                3U,
                100U,
                events
                    .n); // One compare match per period, 1000 periods per second
        }

        void oc_pause_and_resume_channel_gate_the_output() {
            lb::reset();
            lb::toggle_clock(2'000);
            TEST_ASSERT_TRUE(lb::count_edges(lb::GEN_A, 3'000) >=
                             5U); // 6 edges in 3ms

            TEST_ASSERT_EQUAL(
                HAL_OK, output_compare_pause_channel(GEN, TIMER_CHANNEL_1));
            delay_us(5); // Let the pin settle to its disabled level
            TEST_ASSERT_EQUAL_UINT32(0U, lb::count_edges(lb::GEN_A, 3'000));

            TEST_ASSERT_EQUAL(
                HAL_OK, output_compare_resume_channel(GEN, TIMER_CHANNEL_1));
            TEST_ASSERT_TRUE(lb::count_edges(lb::GEN_A, 3'000) >= 5U);
        }

    } // namespace

    void all() {
        LOGI(TAG, "Starting the tests on the extended timer driver");
        UNITY_BEGIN();

        RUN_TEST(ic_init_rejects_invalid_arguments);
        RUN_TEST(ic_init_fails_on_a_locked_advanced_timer);
        RUN_TEST(ic_timer_prescaler_sets_the_tick_rate);
        RUN_TEST(ic_init_configures_the_gpio_pin);
        RUN_TEST(ic_start_and_stop_run_and_freeze_the_counter);
        RUN_TEST(ic_measures_the_period_of_a_generated_clock);
        RUN_TEST(ic_captures_only_the_selected_edge);
        RUN_TEST(ic_both_edges_capture_the_high_and_low_times);
        RUN_TEST(ic_digital_filter_rejects_glitches);
        RUN_TEST(ic_input_prescaler_divides_the_event_rate);
        RUN_TEST(ic_reports_overcapture_once);
        RUN_TEST(ic_get_sample_rejects_invalid_arguments);
        RUN_TEST(ic_deinit_stops_the_events);
        RUN_TEST(filter_code_grows_with_the_requested_time);
        RUN_TEST(oc_compare_callback_fires_once_per_period);
        RUN_TEST(oc_pause_and_resume_channel_gate_the_output);

        TRY_THEN_FAIL(timx_clk_enable(lb::MEAS, false));
        TRY_THEN_FAIL(timx_clk_enable(lb::GEN, false));

        UNITY_END();
        LOGI(TAG, "Done with all tests on the extended timer driver");
    }

} // namespace test::timer_ext
