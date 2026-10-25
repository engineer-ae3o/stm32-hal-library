#include "drivers/gpio.h"
#include "stm32f411xe.h"
#include "Unity/unity.h"

#include "drivers/timer_pcnt_types.h"
#include "drivers/timer_types.h"
#include "drivers/timer_pcnt.h"
#include "test/timer_pcnt.hpp"
#include "test/loopback.hpp"
#include "drivers/timer.h"
#include "utils/common.h"
#include "utils/board.h"
#include "utils/clock.h"
#include "utils/tick.h"
#include "utils/err.h"
#include "utils/log.h"

#include <array>
#include <cstdint>
#include <utility>


// Pulse counter and quadrature decoder tests. Exact counts come from bit banging the loopback pins,
// the generated clock test uses the generator timer's output compare. Wiring is in test/loopback.hpp.

namespace test::timer_pcnt {

    namespace {

        constexpr const char* TAG = "PCNT_Test";

        namespace lb = test::loopback;

        TIM_TypeDef* const MEAS = lb::TESTED;
        TIM_TypeDef* const GEN  = lb::SOURCE;

        struct update_logs_t {
            volatile uint32_t          num_updates = 0;
            volatile timer_count_dir_t dir         = TIMER_COUNTER_DIR_NONE;
        };

        void update_cb(void* arg, timer_channel_t, timer_count_dir_t dir) {
            auto* logs        = static_cast<update_logs_t*>(arg);
            logs->dir         = dir;
            logs->num_updates = logs->num_updates + 1;
        }

        // Mode 1 on the given channel's pin
        inline pcnt_config_t pcnt_1(pcnt_channel_t channel = PCNT_CHANNEL_1,
                                    pcnt_edge_t    edge    = PCNT_RISING_EDGE) {
            pcnt_config_t cfg{};
            cfg.mode                    = PCNT_MODE_1;
            cfg.edge                    = edge;
            cfg.pull                    = TIMER_USE_PULLDOWN;
            cfg.clk_div                 = TIM_CLK_DIV_1;
            cfg.settings.mode_1.channel = channel;
            cfg.filter_ns               = 0;
            cfg.pulse_gpio =
                (channel == PCNT_CHANNEL_1) ? lb::INPUT_A : lb::INPUT_B;
            return cfg;
        }

        // Mode 2 on the external trigger pin
        inline pcnt_config_t pcnt_2(pcnt_prescaler_t prescaler = PCNT_ETR_DIV_1,
                                    pcnt_edge_t      edge = PCNT_RISING_EDGE) {
            pcnt_config_t cfg{};
            cfg.mode                      = PCNT_MODE_2;
            cfg.edge                      = edge;
            cfg.pull                      = TIMER_USE_PULLDOWN;
            cfg.clk_div                   = TIM_CLK_DIV_1;
            cfg.settings.mode_2.prescaler = prescaler;
            cfg.filter_ns                 = 0;
            cfg.pulse_gpio                = lb::INPUT_ETR;
            return cfg;
        }

        inline qdec_config_t qdec(encoder_mode_t mode = ENCODER_MODE_3) {
            qdec_config_t cfg{};
            cfg.mode             = mode;
            cfg.pull             = TIMER_USE_PULLDOWN;
            cfg.clk_div          = TIM_CLK_DIV_1;
            cfg.invert_direction = false;
            cfg.filter_ns        = 0;
            cfg.channel_a        = lb::INPUT_A;
            cfg.channel_b        = lb::INPUT_B;
            return cfg;
        }

        inline void start_counting(const pcnt_config_t& cfg) {
            TEST_ASSERT_EQUAL(HAL_OK, pcnt_init(MEAS, &cfg));
            TEST_ASSERT_EQUAL(HAL_OK, pcnt_start(MEAS));
        }

        inline void start_decoding(const qdec_config_t& cfg) {
            TEST_ASSERT_EQUAL(HAL_OK, qdec_init(MEAS, &cfg));
            TEST_ASSERT_EQUAL(HAL_OK, pcnt_start(MEAS));
        }

        inline uint32_t get_pcnt_count() {
            uint32_t value = 0;
            TEST_ASSERT_EQUAL(HAL_OK, pcnt_get_count(MEAS, &value));
            return value;
        }

        // The decoder counts down by wrapping below zero: read it as signed
        inline int32_t get_pcnt_count_signed() {
            return static_cast<int32_t>(get_pcnt_count());
        }

        // Counts per full quadrature cycle. Encoder mode has 4x resolution, compare to the 2x of the other modes
        constexpr int32_t
        counts_per_cycle(encoder_mode_t mode = ENCODER_MODE_3) {
            return (mode == ENCODER_MODE_3) ? 4 : 2;
        }

        // TESTS
        // Pulse counter: init and configuration
        void pcnt_init_rejects_invalid_arguments() {
            lb::reset();
            const auto good = pcnt_1();
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, pcnt_init(nullptr, &good));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, pcnt_init(MEAS, nullptr));

            // TIM10 and TIM11 have no slave mode controller, TIM5 and TIM9 have no external trigger pin
            TEST_ASSERT_EQUAL(HAL_ERR_NOT_SUPPORTED, pcnt_init(TIM10, &good));
            TEST_ASSERT_EQUAL(HAL_ERR_NOT_SUPPORTED, pcnt_init(TIM11, &good));
            const auto mode_2 = pcnt_2();
            TEST_ASSERT_EQUAL(HAL_ERR_NOT_SUPPORTED, pcnt_init(TIM5, &mode_2));
            TEST_ASSERT_EQUAL(HAL_ERR_NOT_SUPPORTED, pcnt_init(TIM9, &mode_2));

            auto bad_mode = good;
            bad_mode.mode = static_cast<pcnt_mode_t>(7);
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, pcnt_init(MEAS, &bad_mode));

            auto bad_filter      = good;
            bad_filter.filter_ns = 1'000'000'000U;
            TEST_ASSERT_EQUAL(HAL_ERR_NOT_SUPPORTED,
                              pcnt_init(MEAS, &bad_filter));
        }

        void pcnt_init_fails_on_a_locked_advanced_timer() {
            const auto cfg = pcnt_1();
            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM1, true));
            TEST_ASSERT_EQUAL(HAL_OK, timer_reset(TIM1));
            TIM1->BDTR |= TIM_BDTR_LOCK;
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_STATE, pcnt_init(TIM1, &cfg));
            TEST_ASSERT_EQUAL(HAL_OK, timer_reset(TIM1));
            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM1, false));
        }

        void pcnt_init_configures_the_gpio_pin() {
            lb::reset();
            auto cfg = pcnt_1(PCNT_CHANNEL_2);
            cfg.pull = TIMER_USE_PULLDOWN;
            TEST_ASSERT_EQUAL(HAL_OK, pcnt_init(MEAS, &cfg));
            TEST_ASSERT_EQUAL_UINT32(0b10U, lb::moder(lb::INPUT_B));
            TEST_ASSERT_EQUAL_UINT32(lb::INPUT_B.af, lb::afr(lb::INPUT_B));
            TEST_ASSERT_EQUAL_UINT32(0b10U, lb::pupdr(lb::INPUT_B));

            cfg      = pcnt_2();
            cfg.pull = TIMER_USE_PULLUP;
            TEST_ASSERT_EQUAL(HAL_OK, pcnt_init(MEAS, &cfg));
            TEST_ASSERT_EQUAL_UINT32(0b10U, lb::moder(lb::INPUT_ETR));
            TEST_ASSERT_EQUAL_UINT32(lb::INPUT_ETR.af, lb::afr(lb::INPUT_ETR));
            TEST_ASSERT_EQUAL_UINT32(0b01U, lb::pupdr(lb::INPUT_ETR));
        }

        // Pulse counter: counting
        void pcnt_mode_1_counts_pulses_on_either_channel() {
            for (const auto channel : {PCNT_CHANNEL_1, PCNT_CHANNEL_2}) {
                lb::reset();
                start_counting(pcnt_1(channel));

                lb::pulses((channel == PCNT_CHANNEL_1) ? lb::OUTPUT_A
                                                       : lb::OUTPUT_B,
                           100,
                           4);
                delay_us(20);
                TEST_ASSERT_EQUAL_UINT32(100U, get_pcnt_count());
            }
        }

        void pcnt_mode_1_counts_only_the_selected_edge() {
            for (const auto edge : {PCNT_RISING_EDGE, PCNT_FALLING_EDGE}) {
                lb::reset();
                start_counting(pcnt_1(PCNT_CHANNEL_1, edge));

                gpio_set_level(lb::OUTPUT_A.port, lb::OUTPUT_A.pin, true);
                delay_us(20);
                TEST_ASSERT_EQUAL_UINT32((edge == PCNT_RISING_EDGE) ? 1U : 0U,
                                         get_pcnt_count());
                gpio_set_level(lb::OUTPUT_A.port, lb::OUTPUT_A.pin, false);
                delay_us(20);
                TEST_ASSERT_EQUAL_UINT32(1U, get_pcnt_count());
            }
        }

        void pcnt_mode_2_counts_pulses_on_the_selected_edge() {
            for (const auto edge : {PCNT_RISING_EDGE, PCNT_FALLING_EDGE}) {
                lb::reset();
                start_counting(pcnt_2(PCNT_ETR_DIV_1, edge));

                gpio_set_level(lb::OUTPUT_A.port, lb::OUTPUT_A.pin, true);
                delay_us(20);
                TEST_ASSERT_EQUAL_UINT32((edge == PCNT_RISING_EDGE) ? 1U : 0U,
                                         get_pcnt_count());
                gpio_set_level(lb::OUTPUT_A.port, lb::OUTPUT_A.pin, false);
                delay_us(20);
                TEST_ASSERT_EQUAL_UINT32(1U, get_pcnt_count());

                lb::pulses(lb::OUTPUT_A, 100, 4);
                delay_us(20);
                TEST_ASSERT_EQUAL_UINT32(101U, get_pcnt_count());
            }
        }

        void pcnt_mode_2_prescaler_divides_the_count() {
            for (const auto& [prescaler, divisor] :
                 std::array<std::pair<pcnt_prescaler_t, uint32_t>, 3>{{
                     {PCNT_ETR_DIV_2, 2},
                     {PCNT_ETR_DIV_4, 4},
                     {PCNT_ETR_DIV_8, 8},
                 }}) {
                lb::reset();
                start_counting(pcnt_2(prescaler));

                lb::pulses(lb::OUTPUT_A, 800, 4);
                delay_us(20);
                TEST_ASSERT_UINT32_WITHIN(
                    1U,
                    800U / divisor,
                    get_pcnt_count()); // +/- 1 for where in its cycle the prescaler starts
            }
        }

        void pcnt_filter_rejects_glitches() {
            for (auto cfg : {pcnt_1(), pcnt_2()}) {
                lb::reset();
                cfg.filter_ns = 500;
                start_counting(cfg);

                lb::glitches(lb::OUTPUT_A, 100);
                delay_us(50);

                // It should be 0 pulses, but the first transition would trigger an edge
                TEST_ASSERT_UINT32_WITHIN(1, 0, get_pcnt_count());

                lb::pulses(lb::OUTPUT_A, 50, 20);
                delay_us(20);
                TEST_ASSERT_EQUAL_UINT32(50U, get_pcnt_count());
            }
        }

        void pcnt_start_and_stop_freeze_and_resume_the_count() {
            lb::reset();
            const auto cfg = pcnt_1();
            TEST_ASSERT_EQUAL(HAL_OK, pcnt_init(MEAS, &cfg));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, pcnt_start(nullptr));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, pcnt_stop(nullptr));

            lb::pulses(lb::OUTPUT_A, 10, 4); // Not started yet
            TEST_ASSERT_EQUAL_UINT32(0U, get_pcnt_count());

            TEST_ASSERT_EQUAL(HAL_OK, pcnt_start(MEAS));
            lb::pulses(lb::OUTPUT_A, 25, 4);
            delay_us(20);
            TEST_ASSERT_EQUAL_UINT32(25U, get_pcnt_count());

            TEST_ASSERT_EQUAL(HAL_OK, pcnt_stop(MEAS));
            lb::pulses(lb::OUTPUT_A, 10, 4);
            TEST_ASSERT_EQUAL_UINT32(25U, get_pcnt_count());

            TEST_ASSERT_EQUAL(HAL_OK, pcnt_start(MEAS));
            lb::pulses(lb::OUTPUT_A, 5, 4);
            delay_us(20);
            TEST_ASSERT_EQUAL_UINT32(30U, get_pcnt_count());
        }

        void pcnt_set_and_get_count_respect_the_counter_width() {
            lb::reset();
            uint32_t value = 0;
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG,
                              pcnt_get_count(nullptr, &value));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG,
                              pcnt_get_count(MEAS, nullptr));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, pcnt_set_count(nullptr, 0));

            // 16 bit timer
            TEST_ASSERT_EQUAL(HAL_OK, pcnt_set_count(GEN, UINT16_MAX - 5U));
            TEST_ASSERT_EQUAL(HAL_OK, pcnt_get_count(GEN, &value));
            TEST_ASSERT_EQUAL_UINT32(UINT16_MAX - 5U, value);
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG,
                              pcnt_set_count(GEN, UINT16_MAX + 1U));
            TEST_ASSERT_EQUAL(HAL_OK, pcnt_get_count(GEN, &value));
            TEST_ASSERT_EQUAL_UINT32(
                UINT16_MAX - 5U, value); // The rejected write changed nothing

            // 32 bit timer
            TEST_ASSERT_EQUAL(HAL_OK, pcnt_set_count(MEAS, UINT32_MAX - 5U));
            TEST_ASSERT_EQUAL(HAL_OK, pcnt_get_count(MEAS, &value));
            TEST_ASSERT_EQUAL_UINT32(UINT32_MAX - 5U, value);
        }

        void pcnt_counts_a_generated_clock_at_the_right_rate() {
            lb::reset();

            constexpr uint32_t frequency = 100'000U;
            constexpr uint32_t period_ms = 50;

            lb::generate_clock(frequency);
            const auto cfg = pcnt_1();
            TEST_ASSERT_EQUAL(HAL_OK, pcnt_init(MEAS, &cfg));

            // The window is timed with the CPU's cycle counter, independent of the timers' clock arithmetic
            const uint32_t start = DWT->CYCCNT;
            TEST_ASSERT_EQUAL(HAL_OK, pcnt_start(MEAS));
            delay_ms(period_ms);
            TEST_ASSERT_EQUAL(HAL_OK, pcnt_stop(MEAS));
            const uint32_t cycles = DWT->CYCCNT - start;

            const uint32_t expected_pulses = (uint32_t)round_div_u64(
                frequency * (uint64_t)cycles, get_system_core_clock());

            TEST_ASSERT_UINT32_WITHIN(
                round_div_u32(expected_pulses, 200), // 0.5% error
                expected_pulses,
                get_pcnt_count());
        }

        void pcnt_deinit_stops_counting() {
            lb::reset();
            start_counting(pcnt_1());
            lb::pulses(lb::OUTPUT_A, 10, 4);
            delay_us(20);
            TEST_ASSERT_EQUAL_UINT32(10U, get_pcnt_count());

            TEST_ASSERT_EQUAL(HAL_OK, pcnt_deinit(MEAS));
            lb::pulses(lb::OUTPUT_A, 10, 4);
            TEST_ASSERT_EQUAL_UINT32(0U, get_pcnt_count());
        }

        void pcnt_init_replaces_a_previous_decoder_configuration() {
            lb::reset();
            const auto decoder = qdec();
            TEST_ASSERT_EQUAL(HAL_OK, qdec_init(MEAS, &decoder));
            start_counting(
                pcnt_1()); // Must get_pcnt_count pulses now, not quadrature steps

            lb::pulses(lb::OUTPUT_A, 10, 4);
            delay_us(20);
            TEST_ASSERT_EQUAL_UINT32(10U, get_pcnt_count());
        }

        // Quadrature decoding
        void qdec_init_rejects_invalid_arguments() {
            lb::reset();
            const auto good = qdec();
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, qdec_init(nullptr, &good));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, qdec_init(MEAS, nullptr));
            TEST_ASSERT_EQUAL(HAL_ERR_NOT_SUPPORTED, qdec_init(TIM9, &good));
            TEST_ASSERT_EQUAL(HAL_ERR_NOT_SUPPORTED, qdec_init(TIM10, &good));
            TEST_ASSERT_EQUAL(HAL_ERR_NOT_SUPPORTED, qdec_init(TIM11, &good));

            auto bad_filter      = good;
            bad_filter.filter_ns = 1'000'000'000U;
            TEST_ASSERT_EQUAL(HAL_ERR_NOT_SUPPORTED,
                              qdec_init(MEAS, &bad_filter));
        }

        void qdec_init_configures_both_gpio_pins() {
            lb::reset();
            auto cfg = qdec();
            cfg.pull = TIMER_USE_PULLUP;
            TEST_ASSERT_EQUAL(HAL_OK, qdec_init(MEAS, &cfg));

            for (const auto* pin : {&lb::INPUT_A, &lb::INPUT_B}) {
                TEST_ASSERT_EQUAL_UINT32(0b10U, lb::moder(*pin));
                TEST_ASSERT_EQUAL_UINT32(pin->af, lb::afr(*pin));
                TEST_ASSERT_EQUAL_UINT32(0b01U, lb::pupdr(*pin));
            }
        }

        void qdec_counts_up_for_a_leading_and_returns_to_zero_going_back() {
            for (const auto mode : {
                     ENCODER_MODE_1,
                     ENCODER_MODE_2,
                     ENCODER_MODE_3,
                 }) {
                lb::reset();
                start_decoding(qdec(mode));

                lb::quad_walk(50, true); // A leads B
                delay_us(20);
                TEST_ASSERT_EQUAL_INT32(50 * counts_per_cycle(mode),
                                        get_pcnt_count_signed());

                lb::quad_walk(50, false); // B leads A
                delay_us(20);
                TEST_ASSERT_EQUAL_INT32(0, get_pcnt_count_signed());
            }
        }

        void qdec_counts_down_for_b_leading() {
            for (const auto mode : {
                     ENCODER_MODE_1,
                     ENCODER_MODE_2,
                     ENCODER_MODE_3,
                 }) {
                lb::reset();
                start_decoding(qdec(mode));

                lb::quad_walk(50, false);
                delay_us(20);
                TEST_ASSERT_EQUAL_INT32(-50 * counts_per_cycle(mode),
                                        get_pcnt_count_signed());
            }
        }

        void qdec_invert_direction_reverses_the_count() {
            for (const auto mode : {
                     ENCODER_MODE_1,
                     ENCODER_MODE_2,
                     ENCODER_MODE_3,
                 }) {
                lb::reset();
                auto cfg             = qdec(mode);
                cfg.invert_direction = true;
                start_decoding(cfg);

                lb::quad_walk(50, true);
                delay_us(20);
                TEST_ASSERT_EQUAL_INT32(-50 * counts_per_cycle(mode),
                                        get_pcnt_count_signed());
            }
        }

        void qdec_filter_rejects_glitches() {
            lb::reset();
            auto cfg      = qdec();
            cfg.filter_ns = 500;
            start_decoding(cfg);

            lb::glitches(lb::OUTPUT_A, 100);
            lb::glitches(lb::OUTPUT_B, 100);
            delay_us(50);

            // It should be 0 pulses, but the first transition would trigger an edge
            TEST_ASSERT_INT32_WITHIN(
                1 * counts_per_cycle(), 0, get_pcnt_count_signed());

            lb::quad_walk(25, true);
            delay_us(50);
            TEST_ASSERT_EQUAL_UINT32(100, get_pcnt_count_signed());
        }

        void qdec_update_callback_reports_the_direction_of_update() {
            lb::reset();
            update_logs_t logs;
            auto          cfg = qdec();
            cfg.callback      = {.cb = update_cb, .arg = &logs};
            start_decoding(cfg);

            // 80 (20 steps * 4x resolution) counts up from 40 below the top of the 32 bit counter wraps past it
            uint32_t current = UINT32_MAX - 39U;
            TEST_ASSERT_EQUAL(HAL_OK, pcnt_set_count(MEAS, current));

            lb::quad_walk(20, true);
            current += 20 * 4;
            delay_us(20);

            TEST_ASSERT_EQUAL_UINT32(1U, logs.num_updates);
            TEST_ASSERT_EQUAL(TIMER_COUNTER_DIR_UP, logs.dir);
            TEST_ASSERT_EQUAL_UINT32(current, get_pcnt_count());

            // 160 (40 steps * 4x resolution) counts back goes under zero
            lb::quad_walk(40, false);
            current -= 40 * 4;
            delay_us(20);

            TEST_ASSERT_EQUAL_UINT32(2U, logs.num_updates);
            TEST_ASSERT_EQUAL(TIMER_COUNTER_DIR_DOWN, logs.dir);
            TEST_ASSERT_EQUAL_UINT32(current, get_pcnt_count());
        }

        // Pulse counter and decoder: more coverage
        void pcnt_overflow_callback_fires_when_the_counter_wraps() {
            lb::reset();
            update_logs_t logs;
            auto          cfg = pcnt_1();
            cfg.callback      = {.cb = update_cb, .arg = &logs};
            start_counting(cfg);

            // 100 pulses from 50 below the top of the 32 bit counter: one wrap, 50 pulses past it
            TEST_ASSERT_EQUAL(HAL_OK, pcnt_set_count(MEAS, UINT32_MAX - 49U));
            lb::pulses(lb::OUTPUT_A, 100, 4);
            delay_us(20);
            TEST_ASSERT_EQUAL_UINT32(1U, logs.num_updates);
            TEST_ASSERT_EQUAL(TIMER_COUNTER_DIR_UP, logs.dir);
            TEST_ASSERT_EQUAL_UINT32(50U, get_pcnt_count());
        }

        void pcnt_mode_2_prescaler_divides_a_generated_clock() {
            lb::reset();
            lb::generate_clock(100'000);
            start_counting(pcnt_2(PCNT_ETR_DIV_4));

            // The window is timed with the CPU's cycle counter, independent of the timers' clock arithmetic
            const uint32_t start = DWT->CYCCNT;
            delay_ms(50);
            TEST_ASSERT_EQUAL(HAL_OK, pcnt_stop(MEAS));
            const uint32_t cycles = DWT->CYCCNT - start;

            // A quarter of the pulses reach the counter
            const uint32_t expected_pulses = round_div_u32(
                (uint32_t)round_div_u64(100'000ULL * (uint64_t)cycles,
                                        get_system_core_clock()),
                4);

            TEST_ASSERT_UINT32_WITHIN(
                round_div_u32(expected_pulses, 200), // 0.5% error
                expected_pulses,
                get_pcnt_count());
        }

        void qdec_single_channel_modes_ignore_the_other_channel() {
            struct case_t {
                encoder_mode_t mode;
                board_pin_t    ignored;
                board_pin_t    counted;
            };
            // Mode 1 counts channel A's edges only, mode 2 channel B's
            for (const auto& [mode, ignored, counted] : std::array<case_t, 2>{{
                     {
                         .mode    = ENCODER_MODE_1,
                         .ignored = lb::OUTPUT_B,
                         .counted = lb::OUTPUT_A,
                     },
                     {
                         .mode    = ENCODER_MODE_2,
                         .ignored = lb::OUTPUT_A,
                         .counted = lb::OUTPUT_B,
                     },
                 }}) {
                lb::reset();
                start_decoding(qdec(mode));

                lb::pulses(ignored, 20, 5);
                delay_us(20);
                TEST_ASSERT_EQUAL_INT32(0, get_pcnt_count_signed());

                gpio_set_level(counted.port, counted.pin, true);
                delay_us(20);
                TEST_ASSERT_TRUE(get_pcnt_count_signed() != 0);
            }
        }

        void qdec_stop_freezes_the_count_and_start_resumes_it() {
            lb::reset();
            start_decoding(qdec(ENCODER_MODE_3));

            int32_t current = 0;

            constexpr int32_t steps = 10;
            lb::quad_walk(steps, true);
            current += counts_per_cycle(ENCODER_MODE_3) * steps;
            delay_us(20);

            TEST_ASSERT_EQUAL_INT32(current, get_pcnt_count_signed());

            TEST_ASSERT_EQUAL(HAL_OK, pcnt_stop(MEAS));
            lb::quad_walk(steps, true);
            TEST_ASSERT_EQUAL_INT32(current, get_pcnt_count_signed());

            TEST_ASSERT_EQUAL(HAL_OK, pcnt_start(MEAS));
            lb::quad_walk(steps, true);
            current += counts_per_cycle(ENCODER_MODE_3) * steps;
            delay_us(20);

            TEST_ASSERT_EQUAL_INT32(current, get_pcnt_count_signed());
        }

    } // namespace

    void all() {
        LOGI(TAG, "Starting the tests on the PCNT driver");
        UNITY_BEGIN();

        // Pulse counting tests
        RUN_TEST(pcnt_init_rejects_invalid_arguments);
        RUN_TEST(pcnt_init_fails_on_a_locked_advanced_timer);
        RUN_TEST(pcnt_init_configures_the_gpio_pin);
        RUN_TEST(pcnt_mode_1_counts_pulses_on_either_channel);
        RUN_TEST(pcnt_mode_1_counts_only_the_selected_edge);
        RUN_TEST(pcnt_mode_2_counts_pulses_on_the_selected_edge);
        RUN_TEST(pcnt_mode_2_prescaler_divides_the_count);
        RUN_TEST(pcnt_filter_rejects_glitches);
        RUN_TEST(pcnt_start_and_stop_freeze_and_resume_the_count);
        RUN_TEST(pcnt_set_and_get_count_respect_the_counter_width);
        RUN_TEST(pcnt_counts_a_generated_clock_at_the_right_rate);
        RUN_TEST(pcnt_deinit_stops_counting);
        RUN_TEST(pcnt_init_replaces_a_previous_decoder_configuration);
        RUN_TEST(pcnt_overflow_callback_fires_when_the_counter_wraps);
        RUN_TEST(pcnt_mode_2_prescaler_divides_a_generated_clock);

        // Quadrature decoder tests
        RUN_TEST(qdec_init_rejects_invalid_arguments);
        RUN_TEST(qdec_init_configures_both_gpio_pins);
        RUN_TEST(qdec_counts_up_for_a_leading_and_returns_to_zero_going_back);
        RUN_TEST(qdec_counts_down_for_b_leading);
        RUN_TEST(qdec_invert_direction_reverses_the_count);
        RUN_TEST(qdec_filter_rejects_glitches);
        RUN_TEST(qdec_update_callback_reports_the_direction_of_update);
        RUN_TEST(qdec_single_channel_modes_ignore_the_other_channel);
        RUN_TEST(qdec_stop_freezes_the_count_and_start_resumes_it);

        TRY_THEN_FAIL(timx_clk_enable(lb::TESTED, false));
        TRY_THEN_FAIL(timx_clk_enable(lb::SOURCE, false));

        UNITY_END();
        LOGI(TAG, "Done with all tests on the PCNT driver");
    }

} // namespace test::timer_pcnt
