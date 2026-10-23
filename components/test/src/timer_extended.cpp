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


namespace test::timer_ext {

    namespace {

        constexpr const char* TAG = "Timer_ext_Test";

        namespace lb = test::loopback;

        TIM_TypeDef* const MEAS = lb::TESTED;
        TIM_TypeDef* const SRC  = lb::SOURCE;

        // Helpers
        void count_cb(void* arg, timer_channel_t, timer_count_dir_t) {
            auto& counter = *static_cast<volatile uint32_t*>(arg);
            counter += 1;
        }

        void channel_cb(void* arg, timer_channel_t channel, timer_count_dir_t) {
            TEST_ASSERT_TRUE(std::to_underlying(channel) <= TIMER_CHANNEL_4);
            auto* counter = static_cast<volatile uint32_t*>(arg);
            counter[std::to_underlying(channel)] += 1;
        }

        // Channel 1 on IN_A, 1us ticks, no filter, no callback
        inline input_capture_config_t
        ic_config(ic_edge_t edge = IC_EDGE_RISING) {
            input_capture_config_t cfg{};
            cfg.clk_div         = TIM_CLK_DIV_1;
            cfg.count_mode      = TIMER_COUNTER_DIR_UP;
            cfg.timer_prescaler = lb::ticks_1us();
            cfg.channels[0]     = {
                .edge      = edge,
                .pull      = TIMER_USE_PULLDOWN,
                .prescaler = IC_PRESCALER_DIV_1,
                .channel   = TIMER_CHANNEL_1,
                .gpio_pin  = lb::INPUT_A,
                .filter_ns = 0,
            };
            cfg.num_channels = 1;
            return cfg;
        }

        inline void start_capture(const input_capture_config_t& cfg) {
            TEST_ASSERT_EQUAL(HAL_OK, input_capture_init(MEAS, &cfg));
            TEST_ASSERT_EQUAL(HAL_OK, input_capture_start(MEAS));
        }

        // Wait for the next channel 1 capture (polling, no callback needed) and return its timestamp
        inline uint32_t next_capture() {
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

        inline uint32_t ticks_between(uint32_t first, uint32_t second) {
            return second - first;
        }

        // TESTS
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
            invalid([](input_capture_config_t& cfg) {
                cfg.num_channels = 0;
            });
            invalid([](input_capture_config_t& cfg) {
                cfg.num_channels = MAX_TIM2_CHANNELS + 1;
            });
            invalid([](input_capture_config_t& cfg) {
                cfg.timer_prescaler = 0;
            });
            invalid([](input_capture_config_t& cfg) {
                cfg.count_mode = TIMER_COUNTER_DIR_NONE;
            });
            invalid([](input_capture_config_t& cfg) {
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
            TEST_ASSERT_EQUAL_UINT32(cfg.timer_prescaler - 1, MEAS->PSC);

            uint32_t hz = 0;
            TEST_ASSERT_EQUAL(HAL_OK,
                              input_capture_get_timer_frequency(MEAS, &hz));
            TEST_ASSERT_EQUAL_UINT32(clk / cfg.timer_prescaler, hz);

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

            TEST_ASSERT_EQUAL_UINT32(0b10U, lb::moder(lb::INPUT_A));
            TEST_ASSERT_EQUAL_UINT32(lb::INPUT_A.af, lb::afr(lb::INPUT_A));
            TEST_ASSERT_EQUAL_UINT32(0b01U, lb::pupdr(lb::INPUT_A));
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
            lb::generate_clock(2'000); // 500us period
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
                TEST_ASSERT_EQUAL(
                    edge == IC_EDGE_RISING,
                    gpio_get_level(lb::INPUT_A.port, lb::INPUT_A.pin));
                (void)next_capture();
                TEST_ASSERT_EQUAL(
                    edge == IC_EDGE_RISING,
                    gpio_get_level(lb::INPUT_A.port, lb::INPUT_A.pin));
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
            uint32_t events           = 0;
            auto     cfg              = ic_config();
            cfg.callback              = {.cb = count_cb, .arg = &events};
            cfg.channels[0].filter_ns = 500;
            start_capture(cfg);

            lb::glitches(lb::OUTPUT_A, 50);
            delay_us(50);
            TEST_ASSERT_EQUAL_UINT32(0U, events);

            lb::pulses(lb::OUTPUT_A, 50, 20);
            delay_us(50);
            TEST_ASSERT_EQUAL_UINT32(50U, events);
        }

        void ic_input_prescaler_divides_the_event_rate() {
            for (const auto& [prescaler, divisor] :
                 std::array<std::pair<ic_prescaler_t, uint32_t>, 3>{{
                     {IC_PRESCALER_DIV_2, 2},
                     {IC_PRESCALER_DIV_4, 4},
                     {IC_PRESCALER_DIV_8, 8},
                 }}) {
                lb::reset();
                uint32_t events           = 0;
                auto     cfg              = ic_config();
                cfg.callback              = {.cb = count_cb, .arg = &events};
                cfg.channels[0].prescaler = prescaler;
                start_capture(cfg);

                lb::pulses(lb::OUTPUT_A, 64, 20);
                delay_us(50);
                TEST_ASSERT_EQUAL_UINT32(64U / divisor, events);
            }
        }

        void ic_reports_overcapture_once() {
            lb::reset();
            lb::generate_clock(2'000);
            start_capture(ic_config());

            // Several edges with nobody reading the capture register
            delay_ms(5);

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
            uint32_t events = 0;
            auto     cfg    = ic_config();
            cfg.callback    = {.cb = count_cb, .arg = &events};
            start_capture(cfg);

            lb::pulses(lb::OUTPUT_A, 10, 20);
            delay_us(50);
            TEST_ASSERT_EQUAL_UINT32(10U, events);

            TEST_ASSERT_EQUAL(HAL_OK, input_capture_deinit(MEAS));
            TEST_ASSERT_FALSE(MEAS->CR1 & TIM_CR1_CEN);
            lb::pulses(lb::OUTPUT_A, 10, 20);
            delay_us(50);
            TEST_ASSERT_EQUAL_UINT32(10U, events);
        }

        // Filter code helper (shared by input capture, the counters and PWM input)
        void filter_code_grows_with_the_requested_time() {
            lb::reset();

            uint32_t none = 0, small = 0, large = 0;
            TEST_ASSERT_EQUAL(
                HAL_OK,
                timer_filter_ns_to_ic_code(MEAS, TIM_CLK_DIV_1, 0, &none));
            TEST_ASSERT_EQUAL(
                HAL_OK,
                timer_filter_ns_to_ic_code(MEAS, TIM_CLK_DIV_1, 100, &small));
            TEST_ASSERT_EQUAL(
                HAL_OK,
                timer_filter_ns_to_ic_code(MEAS, TIM_CLK_DIV_1, 1'000, &large));

            TEST_ASSERT_EQUAL_UINT32(0U, none);
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
            uint32_t events = 0;

            output_compare_config_t cfg{};
            cfg.mode        = OUTPUT_COMPARE_TOGGLE_ON_MATCH;
            cfg.count_mode  = OUTPUT_COMPARE_EDGE_LEFT_ALIGNED;
            cfg.channels[0] = {
                .output_polarity   = false,
                .output_idle_state = false,
                .channel           = TIMER_CHANNEL_1,
                .gpio_pin          = lb::OUTPUT_A,
            };
            cfg.num_channels = 1;
            cfg.callback     = {.cb = count_cb, .arg = &events};

            uint32_t max = 0;
            TEST_ASSERT_EQUAL(HAL_OK, output_compare_init(SRC, &cfg));
            TEST_ASSERT_EQUAL(HAL_OK, output_compare_start(SRC, 1'000, &max));
            TEST_ASSERT_EQUAL(
                HAL_OK,
                output_compare_set_compare(SRC, TIMER_CHANNEL_1, max / 2U));

            delay_ms(100);
            TEST_ASSERT_UINT32_WITHIN(
                3U,
                100U,
                events); // One compare match per period, 1000 periods per second
        }

        void oc_pause_and_resume_channel_gate_the_output() {
            lb::reset();

            constexpr uint32_t frequency_hz = 2'000;
            constexpr uint32_t duration_us  = 3'000;
            constexpr uint32_t edges = (frequency_hz * duration_us) / 1'000'000;
            lb::generate_clock(frequency_hz);

            // 6 edges in 3ms
            TEST_ASSERT_TRUE(lb::count_edges(lb::OUTPUT_A, duration_us) >=
                             edges);

            TEST_ASSERT_EQUAL(
                HAL_OK, output_compare_pause_channel(SRC, TIMER_CHANNEL_1));
            delay_us(1); // Let the pin settle to its disabled level
            TEST_ASSERT_EQUAL_UINT32(
                0U, lb::count_edges(lb::OUTPUT_A, duration_us));

            TEST_ASSERT_EQUAL(
                HAL_OK, output_compare_resume_channel(SRC, TIMER_CHANNEL_1));
            TEST_ASSERT_TRUE(lb::count_edges(lb::OUTPUT_A, duration_us) >=
                             edges);

            lb::reset();
        }

        // Input capture: more coverage
        void ic_channels_capture_on_their_own_pins() {
            lb::reset();
            std::array<uint32_t, 4> counts{};
            auto                    cfg = ic_config();
            cfg.callback    = {.cb = channel_cb, .arg = counts.data()};
            cfg.channels[1] = {
                .edge      = IC_EDGE_RISING,
                .pull      = TIMER_PULL_NONE,
                .prescaler = IC_PRESCALER_DIV_1,
                .channel   = TIMER_CHANNEL_2,
                .gpio_pin  = lb::INPUT_B,
                .filter_ns = 0,
            };
            cfg.num_channels = 2;
            start_capture(cfg);

            lb::pulses(lb::OUTPUT_A, 10, 20);
            delay_us(50);
            TEST_ASSERT_EQUAL_UINT32(10U, counts[0]);
            TEST_ASSERT_EQUAL_UINT32(0U, counts[1]);

            lb::pulses(lb::OUTPUT_B, 7, 20);
            delay_us(50);
            TEST_ASSERT_EQUAL_UINT32(10U, counts[0]);
            TEST_ASSERT_EQUAL_UINT32(7U, counts[1]);
        }

        void ic_edge_selection_sets_the_events_per_pulse() {
            for (const auto& [edge, expected] :
                 std::array<std::pair<ic_edge_t, uint32_t>, 3>{{
                     {IC_EDGE_RISING, 10},
                     {IC_EDGE_FALLING, 10},
                     {IC_EDGE_RISING_FALLING, 20},
                 }}) {
                lb::reset();
                uint32_t events = 0;
                auto     cfg    = ic_config(edge);
                cfg.callback    = {.cb = count_cb, .arg = &events};
                start_capture(cfg);

                lb::pulses(lb::OUTPUT_A, 10, 20);
                delay_us(50);
                TEST_ASSERT_EQUAL_UINT32(expected, events);
            }
        }

        void ic_timer_prescaler_scales_the_captured_ticks() {
            lb::reset();
            lb::generate_clock(2'000); // 500us period

            auto cfg            = ic_config();
            cfg.timer_prescaler = 10 * lb::ticks_1us(); // 10us ticks
            start_capture(cfg);

            const uint32_t first  = next_capture();
            const uint32_t second = next_capture();
            TEST_ASSERT_UINT32_WITHIN(1U, 50U, ticks_between(first, second));
        }

        // Output compare, toggle mode: more coverage
        void oc_toggle_clock_has_the_requested_frequency() {
            struct case_t {
                uint32_t hz;
                uint32_t window_us;
                uint32_t expected_edges; // Two edges per period of the output
            };
            for (const auto& c : std::array<case_t, 2>{{
                     {.hz = 1'000, .window_us = 20'000, .expected_edges = 40},
                     {.hz = 5'000, .window_us = 10'000, .expected_edges = 100},
                 }}) {
                lb::reset();
                lb::generate_clock(c.hz);
                TEST_ASSERT_UINT32_WITHIN(
                    2U,
                    c.expected_edges,
                    lb::count_edges(lb::OUTPUT_A, c.window_us));
            }
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
        RUN_TEST(ic_channels_capture_on_their_own_pins);
        RUN_TEST(ic_edge_selection_sets_the_events_per_pulse);
        RUN_TEST(ic_timer_prescaler_scales_the_captured_ticks);
        RUN_TEST(oc_toggle_clock_has_the_requested_frequency);

        TRY_THEN_FAIL(timx_clk_enable(lb::TESTED, false));
        TRY_THEN_FAIL(timx_clk_enable(lb::SOURCE, false));

        UNITY_END();
        LOGI(TAG, "Done with all tests on the extended timer driver");
    }

} // namespace test::timer_ext
