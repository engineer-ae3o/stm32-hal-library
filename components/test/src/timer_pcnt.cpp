#include "stm32f411xe.h"
#include "Unity/unity.h"

#include "drivers/src/timer_internals.h"
#include "drivers/timer_pcnt_types.h"
#include "drivers/timer_pcnt.h"
#include "drivers/timer_types.h"
#include "drivers/timer.h"
#include "drivers/gpio.h"
#include "test/timer_pcnt.hpp"
#include "test/loopback.hpp"
#include "utils/common.h"
#include "utils/board.h"
#include "utils/clock.h"
#include "utils/tick.h"
#include "utils/err.h"
#include "utils/log.h"

#include <array>
#include <cstdint>
#include <utility>


// Pulse counter and quadrature decoder tests.
// Exact counts come from bit banging the loopback pins (see test/loopback.hpp for the wiring). The generated clock
// tests use TIM5's output compare as the source and the CPU's cycle counter as an independent time base.

namespace test::timer_pcnt {

    namespace {

        constexpr const char* TAG = "PCNT_Test";

        namespace lb = test::loopback;

        TIM_TypeDef* const MEAS = lb::MEAS;
        TIM_TypeDef* const GEN  = lb::GEN;

        // Helpers
        // The loopback pin that feeds the given counter input
        const board_pin_t& stimulus_for(pcnt_channel_t channel) {
            return (channel == PCNT_CHANNEL_1) ? lb::GEN_A : lb::GEN_B;
        }

        const board_pin_t& input_for(pcnt_channel_t channel) {
            return (channel == PCNT_CHANNEL_1) ? lb::IN_A : lb::IN_B;
        }

        pcnt_config_t
        make_pcnt_mode_1(pcnt_channel_t channel, pcnt_edge_t edge = PCNT_RISING_EDGE, uint32_t filter_ns = 0, timer_cb_t callback = {}) {
            pcnt_config_t cfg{};
            cfg.mode                    = PCNT_MODE_1;
            cfg.edge                    = edge;
            cfg.pull                    = TIMER_PULL_NONE;
            cfg.clk_div                 = TIM_CLK_DIV_1;
            cfg.settings.mode_1.channel = channel;
            cfg.filter_ns               = filter_ns;
            cfg.pulse_gpio              = input_for(channel);
            cfg.callback                = callback;
            return cfg;
        }

        pcnt_config_t make_pcnt_mode_2(pcnt_prescale_t prescaler = PCNT_ETR_DIV_1, pcnt_edge_t edge = PCNT_RISING_EDGE, uint32_t filter_ns = 0) {
            pcnt_config_t cfg{};
            cfg.mode                      = PCNT_MODE_2;
            cfg.edge                      = edge;
            cfg.pull                      = TIMER_PULL_NONE;
            cfg.clk_div                   = TIM_CLK_DIV_1;
            cfg.settings.mode_2.prescaler = prescaler;
            cfg.filter_ns                 = filter_ns;
            cfg.pulse_gpio                = lb::IN_ETR;
            cfg.callback                  = {};
            return cfg;
        }

        qdec_config_t make_qdec(encoder_mode_t mode, bool invert = false, uint32_t filter_ns = 0, timer_cb_t callback = {}) {
            qdec_config_t cfg{};
            cfg.mode             = mode;
            cfg.pull             = TIMER_PULL_NONE;
            cfg.clk_div          = TIM_CLK_DIV_1;
            cfg.invert_direction = invert;
            cfg.filter_ns        = filter_ns;
            cfg.channel_a        = lb::IN_A;
            cfg.channel_b        = lb::IN_B;
            cfg.callback         = callback;
            return cfg;
        }

        uint32_t read_count() {
            uint32_t count = 0xDEADBEEFU;
            TEST_ASSERT_EQUAL(HAL_OK, pcnt_get_count(MEAS, &count));
            return count;
        }

        // The decoder counts on a 16 bit timer: read it as a signed value so counting down shows up as negative
        int32_t read_signed_count() {
            return static_cast<int16_t>(read_count());
        }

        // Begin counting with the loopback pins sitting low (A = B = 0) and the counter at zero
        void start_counting_from_idle() {
            lb::take_over_pin(lb::GEN_A, false);
            lb::take_over_pin(lb::GEN_B, false);
            delay_us(20);
        }

        constexpr uint32_t ALL_CCXE = TIM_CCER_CC1E | TIM_CCER_CC2E;
        constexpr uint32_t SMCR_ETR = TIM_SMCR_ETF | TIM_SMCR_ETPS | TIM_SMCR_ECE | TIM_SMCR_ETP;

        volatile uint32_t          s_update_events = 0;
        volatile timer_count_dir_t s_update_dir[8] = {};

        void update_cb(void*, timer_channel_t, timer_count_dir_t direction) {
            const uint32_t i = s_update_events;
            if (i < 8) {
                s_update_dir[i] = direction;
            }
            s_update_events = i + 1;
        }

        void reset_update_log() {
            s_update_events = 0;
            for (auto& d : s_update_dir) {
                d = TIMER_COUNTER_DIR_NONE;
            }
        }

        // TESTS
        // Pulse counter: argument validation and register programming
        void pcnt_init_rejects_invalid_arguments() {
            lb::reset_fixture();
            const auto good = make_pcnt_mode_1(PCNT_CHANNEL_1);

            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, pcnt_init(nullptr, &good));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, pcnt_init(MEAS, nullptr));

            // TIM10 and TIM11 have no slave mode controller at all
            TEST_ASSERT_EQUAL(HAL_ERR_NOT_SUPPORTED, pcnt_init(TIM10, &good));
            TEST_ASSERT_EQUAL(HAL_ERR_NOT_SUPPORTED, pcnt_init(TIM11, &good));

            // TIM5 and TIM9 have no external trigger input for mode 2
            const auto mode_2 = make_pcnt_mode_2();
            TEST_ASSERT_EQUAL(HAL_ERR_NOT_SUPPORTED, pcnt_init(TIM5, &mode_2));
            TEST_ASSERT_EQUAL(HAL_ERR_NOT_SUPPORTED, pcnt_init(TIM9, &mode_2));

            auto bad_mode = good;
            bad_mode.mode = static_cast<pcnt_mode_t>(7);
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, pcnt_init(MEAS, &bad_mode));

            // A filter longer than the hardware can do
            auto bad_filter      = good;
            bad_filter.filter_ns = 1'000'000'000U;
            TEST_ASSERT_EQUAL(HAL_ERR_NOT_SUPPORTED, pcnt_init(MEAS, &bad_filter));

            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM1, true));
            TEST_ASSERT_EQUAL(HAL_OK, timer_reset(TIM1));
            TIM1->BDTR |= TIM_BDTR_LOCK;
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_STATE, pcnt_init(TIM1, &good));
            TEST_ASSERT_EQUAL(HAL_OK, timer_reset(TIM1));
            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM1, false));
        }

        void pcnt_mode_1_programs_the_slave_external_clock_mode() {
            lb::reset_fixture();

            struct channel_case_t {
                pcnt_channel_t channel;
                uint32_t       ccxs_pos;
                uint32_t       ccxp;
                uint32_t       trigger_selection; // TI1FP1 or TI2FP2
            };
            const std::array<channel_case_t, 2> channels = {{
                {PCNT_CHANNEL_1, 0, TIM_CCER_CC1P, 0b101},
                {PCNT_CHANNEL_2, 8, TIM_CCER_CC2P, 0b110},
            }};

            for (const auto& c : channels) {
                for (const auto edge : {PCNT_RISING_EDGE, PCNT_FALLING_EDGE}) {
                    const auto cfg = make_pcnt_mode_1(c.channel, edge);
                    TEST_ASSERT_EQUAL(HAL_OK, pcnt_init(MEAS, &cfg));

                    TEST_ASSERT_EQUAL_UINT32(0b01U, (MEAS->CCMR1 >> c.ccxs_pos) & 0b11U);              // The channel's own TIx is the input
                    TEST_ASSERT_EQUAL_UINT32(0b111U, (MEAS->SMCR & TIM_SMCR_SMS) >> TIM_SMCR_SMS_Pos); // External clock mode 1
                    TEST_ASSERT_EQUAL_UINT32(c.trigger_selection, (MEAS->SMCR & TIM_SMCR_TS) >> TIM_SMCR_TS_Pos);
                    TEST_ASSERT_EQUAL_UINT32((edge == PCNT_FALLING_EDGE) ? c.ccxp : 0U, MEAS->CCER & c.ccxp);

                    TEST_ASSERT_EQUAL_UINT32(UINT16_MAX, MEAS->ARR); // Free running
                    TEST_ASSERT_EQUAL_UINT32(0U, MEAS->PSC);
                    TEST_ASSERT_FALSE(MEAS->CR1 & TIM_CR1_CEN);              // Not counting until started
                    TEST_ASSERT_EQUAL_UINT32(0U, MEAS->DIER & TIM_DIER_UIE); // No overflow callback requested
                }
            }
        }

        void pcnt_mode_2_programs_the_external_trigger_path() {
            lb::reset_fixture();

            const std::array<pcnt_prescale_t, 4> prescalers = {PCNT_ETR_DIV_1, PCNT_ETR_DIV_2, PCNT_ETR_DIV_4, PCNT_ETR_DIV_8};
            for (const auto prescaler : prescalers) {
                for (const auto edge : {PCNT_RISING_EDGE, PCNT_FALLING_EDGE}) {
                    const auto cfg = make_pcnt_mode_2(prescaler, edge);
                    TEST_ASSERT_EQUAL(HAL_OK, pcnt_init(MEAS, &cfg));

                    TEST_ASSERT_TRUE(MEAS->SMCR & TIM_SMCR_ECE);             // External clock mode 2
                    TEST_ASSERT_EQUAL_UINT32(0U, MEAS->SMCR & TIM_SMCR_SMS); // Slave mode controller untouched
                    TEST_ASSERT_EQUAL_UINT32(std::to_underlying(prescaler), (MEAS->SMCR & TIM_SMCR_ETPS) >> TIM_SMCR_ETPS_Pos);
                    TEST_ASSERT_EQUAL_UINT32((edge == PCNT_FALLING_EDGE) ? TIM_SMCR_ETP : 0U, MEAS->SMCR & TIM_SMCR_ETP);
                    TEST_ASSERT_EQUAL_UINT32(0U, MEAS->SMCR & TIM_SMCR_ETF); // No filter requested
                    TEST_ASSERT_EQUAL_UINT32(UINT16_MAX, MEAS->ARR);
                }
            }

            // A requested filter lands in ETF
            const auto filtered = make_pcnt_mode_2(PCNT_ETR_DIV_1, PCNT_RISING_EDGE, 500);
            TEST_ASSERT_EQUAL(HAL_OK, pcnt_init(MEAS, &filtered));
            uint32_t expected_code = 0;
            TEST_ASSERT_EQUAL(HAL_OK, timer_filter_ns_to_ic_code(MEAS, TIM_CLK_DIV_1, 500, &expected_code));
            TEST_ASSERT_TRUE(expected_code != 0U);
            TEST_ASSERT_EQUAL_UINT32(expected_code, (MEAS->SMCR & TIM_SMCR_ETF) >> TIM_SMCR_ETF_Pos);
        }

        void pcnt_init_programs_the_clock_divider_and_the_overflow_callback() {
            lb::reset_fixture();

            auto cfg    = make_pcnt_mode_1(PCNT_CHANNEL_1, PCNT_RISING_EDGE, 0, {update_cb, nullptr});
            cfg.clk_div = TIM_CLK_DIV_4;
            TEST_ASSERT_EQUAL(HAL_OK, pcnt_init(MEAS, &cfg));

            TEST_ASSERT_EQUAL_UINT32(std::to_underlying(TIM_CLK_DIV_4), (MEAS->CR1 & TIM_CR1_CKD) >> TIM_CR1_CKD_Pos);
            TEST_ASSERT_TRUE(MEAS->DIER & TIM_DIER_UIE);
            TEST_ASSERT_TRUE(MEAS->CR1 & TIM_CR1_URS); // Only overflows interrupt, not the UG at init
            TEST_ASSERT_TRUE(NVIC->ISER[TIM3_IRQn >> 5] & (1UL << (TIM3_IRQn & 0x1FU)));
        }

        void pcnt_init_configures_the_gpio_pin() {
            lb::reset_fixture();

            struct pull_case_t {
                timer_pull_t pull;
                uint32_t     pupdr;
            };
            const std::array<pull_case_t, 3> pulls = {{{TIMER_PULL_NONE, 0b00}, {TIMER_USE_PULLUP, 0b01}, {TIMER_USE_PULLDOWN, 0b10}}};

            for (const auto& p : pulls) {
                auto mode_1 = make_pcnt_mode_1(PCNT_CHANNEL_2);
                mode_1.pull = p.pull;
                TEST_ASSERT_EQUAL(HAL_OK, pcnt_init(MEAS, &mode_1));
                TEST_ASSERT_EQUAL_UINT32(0b10U, lb::moder(lb::IN_B));
                TEST_ASSERT_EQUAL_UINT32(lb::IN_B.af, lb::afr(lb::IN_B));
                TEST_ASSERT_EQUAL_UINT32(p.pupdr, lb::pupdr(lb::IN_B));

                auto mode_2 = make_pcnt_mode_2();
                mode_2.pull = p.pull;
                TEST_ASSERT_EQUAL(HAL_OK, pcnt_init(MEAS, &mode_2));
                TEST_ASSERT_EQUAL_UINT32(0b10U, lb::moder(lb::IN_ETR));
                TEST_ASSERT_EQUAL_UINT32(lb::IN_ETR.af, lb::afr(lb::IN_ETR));
                TEST_ASSERT_EQUAL_UINT32(p.pupdr, lb::pupdr(lb::IN_ETR));
            }
        }

        // Pulse counter: exact counts
        void pcnt_mode_1_counts_exactly_on_the_selected_edge() {
            for (const auto channel : {PCNT_CHANNEL_1, PCNT_CHANNEL_2}) {
                for (const auto edge : {PCNT_RISING_EDGE, PCNT_FALLING_EDGE}) {
                    lb::reset_fixture();
                    const board_pin_t& stimulus = stimulus_for(channel);
                    lb::take_over_pin(stimulus, false);
                    delay_us(20);

                    const auto cfg = make_pcnt_mode_1(channel, edge);
                    TEST_ASSERT_EQUAL(HAL_OK, pcnt_init(MEAS, &cfg));
                    TEST_ASSERT_EQUAL(HAL_OK, pcnt_set_count(MEAS, 0));
                    TEST_ASSERT_EQUAL(HAL_OK, pcnt_start(MEAS));

                    // A single edge in each direction: only the selected one counts
                    lb::drive(stimulus, true);
                    delay_us(20);
                    TEST_ASSERT_EQUAL_UINT32((edge == PCNT_RISING_EDGE) ? 1U : 0U, read_count());
                    lb::drive(stimulus, false);
                    delay_us(20);
                    TEST_ASSERT_EQUAL_UINT32(1U, read_count());

                    // Then 100 full pulses: exactly one count each
                    lb::pulses(stimulus, 100, 4, 4);
                    delay_us(20);
                    TEST_ASSERT_EQUAL_UINT32(101U, read_count());
                }
            }
        }

        void pcnt_mode_2_counts_exactly_and_the_etr_prescaler_divides() {
            struct div_case_t {
                pcnt_prescale_t prescaler;
                uint32_t        divisor;
            };
            const std::array<div_case_t, 4> cases = {{{PCNT_ETR_DIV_1, 1}, {PCNT_ETR_DIV_2, 2}, {PCNT_ETR_DIV_4, 4}, {PCNT_ETR_DIV_8, 8}}};

            for (const auto& c : cases) {
                lb::reset_fixture();
                lb::take_over_pin(lb::GEN_A, false);
                delay_us(20);

                const auto cfg = make_pcnt_mode_2(c.prescaler);
                TEST_ASSERT_EQUAL(HAL_OK, pcnt_init(MEAS, &cfg));
                TEST_ASSERT_EQUAL(HAL_OK, pcnt_set_count(MEAS, 0));
                TEST_ASSERT_EQUAL(HAL_OK, pcnt_start(MEAS));

                lb::pulses(lb::GEN_A, 800, 4, 4);
                delay_us(20);

                // The external trigger prescaler divides the pulse rate. +/- 1 for where in its cycle it starts
                TEST_ASSERT_UINT32_WITHIN(1U, 800U / c.divisor, read_count());
            }
        }

        void pcnt_mode_2_counts_exactly_on_the_selected_edge() {
            for (const auto edge : {PCNT_RISING_EDGE, PCNT_FALLING_EDGE}) {
                lb::reset_fixture();
                lb::take_over_pin(lb::GEN_A, false);
                delay_us(20);

                const auto cfg = make_pcnt_mode_2(PCNT_ETR_DIV_1, edge);
                TEST_ASSERT_EQUAL(HAL_OK, pcnt_init(MEAS, &cfg));
                TEST_ASSERT_EQUAL(HAL_OK, pcnt_set_count(MEAS, 0));
                TEST_ASSERT_EQUAL(HAL_OK, pcnt_start(MEAS));

                lb::drive(lb::GEN_A, true);
                delay_us(20);
                TEST_ASSERT_EQUAL_UINT32((edge == PCNT_RISING_EDGE) ? 1U : 0U, read_count());
                lb::drive(lb::GEN_A, false);
                delay_us(20);
                TEST_ASSERT_EQUAL_UINT32(1U, read_count());
            }
        }

        void pcnt_filter_rejects_glitches_in_both_modes() {
            for (const bool use_etr : {false, true}) {
                lb::reset_fixture();
                lb::take_over_pin(lb::GEN_A, false);
                delay_us(20);

                const auto cfg =
                    use_etr ? make_pcnt_mode_2(PCNT_ETR_DIV_1, PCNT_RISING_EDGE, 500) : make_pcnt_mode_1(PCNT_CHANNEL_1, PCNT_RISING_EDGE, 500);
                TEST_ASSERT_EQUAL(HAL_OK, pcnt_init(MEAS, &cfg));
                TEST_ASSERT_EQUAL(HAL_OK, pcnt_set_count(MEAS, 0));
                TEST_ASSERT_EQUAL(HAL_OK, pcnt_start(MEAS));

                lb::glitches(lb::GEN_A, 50);
                delay_us(50);
                TEST_ASSERT_EQUAL_UINT32(0U, read_count());

                lb::pulses(lb::GEN_A, 50, 20, 20);
                delay_us(50);
                TEST_ASSERT_EQUAL_UINT32(50U, read_count());
            }
        }

        void pcnt_start_and_stop_freeze_and_resume_the_count() {
            lb::reset_fixture();
            lb::take_over_pin(lb::GEN_A, false);
            delay_us(20);

            const auto cfg = make_pcnt_mode_1(PCNT_CHANNEL_1);
            TEST_ASSERT_EQUAL(HAL_OK, pcnt_init(MEAS, &cfg));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, pcnt_start(nullptr));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, pcnt_stop(nullptr));

            // Pulses before the counter is started are not counted
            lb::pulses(lb::GEN_A, 10, 4, 4);
            TEST_ASSERT_EQUAL_UINT32(0U, read_count());

            TEST_ASSERT_EQUAL(HAL_OK, pcnt_start(MEAS));
            TEST_ASSERT_TRUE(MEAS->CR1 & TIM_CR1_CEN);
            lb::pulses(lb::GEN_A, 25, 4, 4);
            delay_us(20);
            TEST_ASSERT_EQUAL_UINT32(25U, read_count());

            TEST_ASSERT_EQUAL(HAL_OK, pcnt_stop(MEAS));
            TEST_ASSERT_FALSE(MEAS->CR1 & TIM_CR1_CEN);
            lb::pulses(lb::GEN_A, 10, 4, 4);
            TEST_ASSERT_EQUAL_UINT32(25U, read_count()); // Frozen

            TEST_ASSERT_EQUAL(HAL_OK, pcnt_start(MEAS));
            lb::pulses(lb::GEN_A, 5, 4, 4);
            delay_us(20);
            TEST_ASSERT_EQUAL_UINT32(30U, read_count()); // Resumes from where it stopped
        }

        void pcnt_set_and_get_count_respect_the_counter_width() {
            lb::reset_fixture();

            uint32_t count = 0;
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, pcnt_get_count(nullptr, &count));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, pcnt_get_count(MEAS, nullptr));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, pcnt_set_count(nullptr, 0));

            // 16 bit timer
            TEST_ASSERT_EQUAL(HAL_OK, pcnt_set_count(MEAS, UINT16_MAX));
            TEST_ASSERT_EQUAL_UINT32(UINT16_MAX, read_count());
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, pcnt_set_count(MEAS, UINT16_MAX + 1U));
            TEST_ASSERT_EQUAL_UINT32(UINT16_MAX, read_count()); // The rejected write changed nothing

            // 32 bit timer
            TEST_ASSERT_EQUAL(HAL_OK, pcnt_set_count(GEN, UINT32_MAX - 5U));
            TEST_ASSERT_EQUAL(HAL_OK, pcnt_get_count(GEN, &count));
            TEST_ASSERT_EQUAL_UINT32(UINT32_MAX - 5U, count);
        }

        void pcnt_deinit_stops_counting_and_clears_the_slave_configuration() {
            lb::reset_fixture();
            const auto cfg = make_pcnt_mode_1(PCNT_CHANNEL_1, PCNT_FALLING_EDGE, 0, {update_cb, nullptr});
            TEST_ASSERT_EQUAL(HAL_OK, pcnt_init(MEAS, &cfg));
            TEST_ASSERT_EQUAL(HAL_OK, pcnt_start(MEAS));

            TEST_ASSERT_EQUAL(HAL_OK, pcnt_deinit(MEAS));
            TEST_ASSERT_FALSE(MEAS->CR1 & TIM_CR1_CEN);
            TEST_ASSERT_EQUAL_UINT32(0U, MEAS->SMCR & (TIM_SMCR_SMS | TIM_SMCR_TS | SMCR_ETR));
            TEST_ASSERT_EQUAL_UINT32(0U, MEAS->CCMR1 & (TIM_CCMR1_CC1S | TIM_CCMR1_CC2S));
            TEST_ASSERT_EQUAL_UINT32(0U, MEAS->CCER & (TIM_CCER_CC1P | TIM_CCER_CC2P | ALL_CCXE));
            TEST_ASSERT_EQUAL_UINT32(0U, MEAS->DIER & TIM_DIER_UIE);
            TEST_ASSERT_EQUAL_UINT32(0U, MEAS->CNT);
            TEST_ASSERT_FALSE(NVIC->ISER[TIM3_IRQn >> 5] & (1UL << (TIM3_IRQn & 0x1FU)));
        }

        void pcnt_and_qdec_init_overwrite_each_other() {
            lb::reset_fixture();

            // Mode 1 > decoder: the slave mode and trigger selection are replaced, not merged
            const auto mode_1 = make_pcnt_mode_1(PCNT_CHANNEL_2);
            TEST_ASSERT_EQUAL(HAL_OK, pcnt_init(MEAS, &mode_1));
            const auto qdec = make_qdec(ENCODER_MODE_3);
            TEST_ASSERT_EQUAL(HAL_OK, qdec_init(MEAS, &qdec));
            TEST_ASSERT_EQUAL_UINT32(std::to_underlying(ENCODER_MODE_3), MEAS->SMCR & TIM_SMCR_SMS);
            TEST_ASSERT_EQUAL_UINT32(0U, MEAS->SMCR & TIM_SMCR_TS);

            // Decoder > mode 2
            const auto mode_2 = make_pcnt_mode_2();
            TEST_ASSERT_EQUAL(HAL_OK, pcnt_init(MEAS, &mode_2));
            TEST_ASSERT_EQUAL_UINT32(0U, MEAS->SMCR & TIM_SMCR_SMS);
            TEST_ASSERT_TRUE(MEAS->SMCR & TIM_SMCR_ECE);

            // Mode 2 > mode 1: the external clock mode 2 enable must not survive
            TEST_ASSERT_EQUAL(HAL_OK, pcnt_init(MEAS, &mode_1));
            TEST_ASSERT_FALSE(MEAS->SMCR & TIM_SMCR_ECE);
            TEST_ASSERT_EQUAL_UINT32(0b111U, (MEAS->SMCR & TIM_SMCR_SMS) >> TIM_SMCR_SMS_Pos);
        }

        // Pulse counter: generated clock, against the CPU's cycle counter
        struct count_result_t {
            hal_err_t err;
            uint32_t  sysclk_hz;
            uint32_t  pin_hz;    // What the generator's registers say it produces
            uint32_t  window_us; // How long the counter ran for, by the cycle counter
            uint32_t  count;
            uint32_t  overflows;
        };

        // Count the generated clock for 'window_ms' with the given counter configuration
        hal_err_t count_generated_clock(const pcnt_config_t& cfg, uint32_t pin_hz, uint32_t window_ms, count_result_t* out) {
            TRY(lb::reset_fixture_hal());
            TRY(lb::generator_toggle_start(TIMER_CHANNEL_1, lb::GEN_A, pin_hz));
            out->sysclk_hz = get_system_core_clock();
            out->pin_hz    = lb::generator_pin_hz();

            reset_update_log();
            TRY(pcnt_init(MEAS, &cfg));
            TRY(pcnt_set_count(MEAS, 0));

            const uint32_t t0 = DWT->CYCCNT;
            TRY(pcnt_start(MEAS));
            delay_ms(window_ms);
            TRY(pcnt_stop(MEAS));
            const uint32_t t1 = DWT->CYCCNT;

            out->window_us = lb::cycles_to_us32(t1 - t0, out->sysclk_hz);
            out->overflows = s_update_events;
            return pcnt_get_count(MEAS, &out->count);
        }

        void pcnt_frequency_of_the_generated_clock_matches_across_system_clocks() {
            struct variant_t {
                pcnt_config_t cfg;
                uint32_t      divisor; // Counts per generated pulse
            };

            const system_clock_t original = get_system_core_clock_type();

            // Mode 1 on the channel 1 pin, and mode 2 on the external trigger pin with the /4 prescaler
            const std::array<variant_t, 2> variants = {{
                {make_pcnt_mode_1(PCNT_CHANNEL_1), 1},
                {make_pcnt_mode_2(PCNT_ETR_DIV_4), 4},
            }};

            std::array<std::array<count_result_t, lb::CLOCK_SWEEP.size()>, variants.size()> results{};
            for (size_t v = 0; v < variants.size(); v++) {
                for (size_t i = 0; i < lb::CLOCK_SWEEP.size(); i++) {
                    lb::switch_clock(lb::pick_preset(lb::CLOCK_SWEEP[i], original));
                    results[v][i].err = count_generated_clock(variants[v].cfg, 100'000, 50, &results[v][i]);
                    (void)pcnt_deinit(MEAS);
                    lb::generator_stop();
                }
            }
            lb::switch_clock(original); // Back to the original clock before anything gets asserted
            lb::reset_fixture();

            for (size_t v = 0; v < variants.size(); v++) {
                for (size_t i = 0; i < lb::CLOCK_SWEEP.size(); i++) {
                    const auto& r = results[v][i];
                    TEST_ASSERT_EQUAL(HAL_OK, r.err);
                    TEST_ASSERT_EQUAL_UINT32(lb::CLOCK_SWEEP[i].mhz * 1'000'000U, r.sysclk_hz);
                    TEST_ASSERT_EQUAL_UINT32(100'000U, r.pin_hz);

                    // Frequency from the count and the independently timed window, within 0.3%
                    const uint32_t measured_hz = (uint32_t)((uint64_t)r.count * variants[v].divisor * 1'000'000ULL / r.window_us);
                    TEST_ASSERT_UINT32_WITHIN(300U, 100'000U, measured_hz);
                }
            }
        }

        void pcnt_overflow_callback_fires_once_per_wrap_of_the_counter() {
            // 500kHz for 150ms is about 75000 counts: one wrap of a 16 bit counter
            count_result_t result{};
            const auto     cfg = make_pcnt_mode_1(PCNT_CHANNEL_1, PCNT_RISING_EDGE, 0, {update_cb, nullptr});
            TEST_ASSERT_EQUAL(HAL_OK, count_generated_clock(cfg, 500'000, 150, &result));
            TEST_ASSERT_EQUAL_UINT32(500'000U, result.pin_hz);

            const uint32_t total    = result.overflows * 65'536U + result.count;
            const uint32_t expected = (uint32_t)((uint64_t)result.pin_hz * result.window_us / 1'000'000ULL);
            TEST_ASSERT_UINT32_WITHIN(expected / 500U + 5U, expected, total); // 0.2% + a few counts for the start and stop skew
            TEST_ASSERT_UINT32_WITHIN(1U, expected / 65'536U, result.overflows);
            TEST_ASSERT_TRUE(result.overflows >= 1U);
            TEST_ASSERT_EQUAL(TIMER_COUNTER_DIR_UP, s_update_dir[0]); // A pulse counter only ever counts up
        }

        // Quadrature decoder
        void qdec_init_rejects_invalid_arguments() {
            lb::reset_fixture();
            const auto good = make_qdec(ENCODER_MODE_3);

            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, qdec_init(nullptr, &good));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, qdec_init(MEAS, nullptr));

            for (auto* const handle : {TIM9, TIM10, TIM11}) {
                TEST_ASSERT_EQUAL(HAL_ERR_NOT_SUPPORTED, qdec_init(handle, &good));
            }

            auto bad_filter      = good;
            bad_filter.filter_ns = 1'000'000'000U;
            TEST_ASSERT_EQUAL(HAL_ERR_NOT_SUPPORTED, qdec_init(MEAS, &bad_filter));

            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM1, true));
            TEST_ASSERT_EQUAL(HAL_OK, timer_reset(TIM1));
            TIM1->BDTR |= TIM_BDTR_LOCK;
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_STATE, qdec_init(TIM1, &good));
            TEST_ASSERT_EQUAL(HAL_OK, timer_reset(TIM1));
            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM1, false));
        }

        void qdec_programs_the_encoder_mode_and_both_inputs() {
            lb::reset_fixture();

            for (const auto mode : {ENCODER_MODE_1, ENCODER_MODE_2, ENCODER_MODE_3}) {
                const auto cfg = make_qdec(mode);
                TEST_ASSERT_EQUAL(HAL_OK, qdec_init(MEAS, &cfg));

                TEST_ASSERT_EQUAL_UINT32(std::to_underlying(mode), MEAS->SMCR & TIM_SMCR_SMS);
                TEST_ASSERT_EQUAL_UINT32(0b01U, MEAS->CCMR1 & TIM_CCMR1_CC1S); // Both channels read their own pin
                TEST_ASSERT_EQUAL_UINT32(0b01U, (MEAS->CCMR1 & TIM_CCMR1_CC2S) >> TIM_CCMR1_CC2S_Pos);
                TEST_ASSERT_EQUAL_UINT32(0U, MEAS->CCER & (TIM_CCER_CC1P | TIM_CCER_CC2P)); // Not inverted
                TEST_ASSERT_EQUAL_UINT32(UINT16_MAX, MEAS->ARR);
                TEST_ASSERT_FALSE(MEAS->CR1 & TIM_CR1_CEN);

                // Both pins are alternate function inputs
                TEST_ASSERT_EQUAL_UINT32(0b10U, lb::moder(lb::IN_A));
                TEST_ASSERT_EQUAL_UINT32(0b10U, lb::moder(lb::IN_B));
                TEST_ASSERT_EQUAL_UINT32(lb::IN_A.af, lb::afr(lb::IN_A));
                TEST_ASSERT_EQUAL_UINT32(lb::IN_B.af, lb::afr(lb::IN_B));
            }
        }

        void qdec_configures_the_pull_resistors_on_both_pins() {
            lb::reset_fixture();

            struct pull_case_t {
                timer_pull_t pull;
                uint32_t     pupdr;
            };
            const std::array<pull_case_t, 3> pulls = {{{TIMER_PULL_NONE, 0b00}, {TIMER_USE_PULLUP, 0b01}, {TIMER_USE_PULLDOWN, 0b10}}};

            for (const auto& p : pulls) {
                auto cfg = make_qdec(ENCODER_MODE_3);
                cfg.pull = p.pull;
                TEST_ASSERT_EQUAL(HAL_OK, qdec_init(MEAS, &cfg));
                TEST_ASSERT_EQUAL_UINT32(p.pupdr, lb::pupdr(lb::IN_A));
                TEST_ASSERT_EQUAL_UINT32(p.pupdr, lb::pupdr(lb::IN_B));
            }
        }

        // Counts per full quadrature cycle: 4 when both channels count, 2 when only one does
        uint32_t counts_per_cycle(encoder_mode_t mode) {
            return (mode == ENCODER_MODE_3) ? 4U : 2U;
        }

        void qdec_counts_each_mode_in_both_directions() {
            constexpr uint32_t CYCLES = 200;

            for (const auto mode : {ENCODER_MODE_1, ENCODER_MODE_2, ENCODER_MODE_3}) {
                for (const bool a_leads_b : {true, false}) {
                    lb::reset_fixture();
                    start_counting_from_idle();

                    const auto cfg = make_qdec(mode);
                    TEST_ASSERT_EQUAL(HAL_OK, qdec_init(MEAS, &cfg));
                    TEST_ASSERT_EQUAL(HAL_OK, pcnt_set_count(MEAS, 0));
                    TEST_ASSERT_EQUAL(HAL_OK, pcnt_start(MEAS));

                    lb::quad_walk(CYCLES, a_leads_b);
                    delay_us(20);

                    // A leading B counts up, B leading A counts down
                    const int32_t expected = static_cast<int32_t>(CYCLES * counts_per_cycle(mode)) * (a_leads_b ? 1 : -1);
                    TEST_ASSERT_EQUAL_INT32(expected, read_signed_count());
                }
            }
        }

        void qdec_walking_forward_then_back_returns_to_zero() {
            lb::reset_fixture();
            start_counting_from_idle();

            const auto cfg = make_qdec(ENCODER_MODE_3);
            TEST_ASSERT_EQUAL(HAL_OK, qdec_init(MEAS, &cfg));
            TEST_ASSERT_EQUAL(HAL_OK, pcnt_set_count(MEAS, 0));
            TEST_ASSERT_EQUAL(HAL_OK, pcnt_start(MEAS));

            lb::quad_walk(75, true);
            delay_us(20);
            TEST_ASSERT_EQUAL_INT32(300, read_signed_count());
            lb::quad_walk(75, false);
            delay_us(20);
            TEST_ASSERT_EQUAL_INT32(0, read_signed_count());
        }

        void qdec_invert_direction_reverses_the_count() {
            constexpr uint32_t CYCLES = 100;

            for (const auto mode : {ENCODER_MODE_1, ENCODER_MODE_2, ENCODER_MODE_3}) {
                for (const bool a_leads_b : {true, false}) {
                    lb::reset_fixture();
                    start_counting_from_idle();

                    const auto cfg = make_qdec(mode, true);
                    TEST_ASSERT_EQUAL(HAL_OK, qdec_init(MEAS, &cfg));
                    TEST_ASSERT_EQUAL(HAL_OK, pcnt_set_count(MEAS, 0));
                    TEST_ASSERT_EQUAL(HAL_OK, pcnt_start(MEAS));

                    lb::quad_walk(CYCLES, a_leads_b);
                    delay_us(20);

                    // Same magnitude as without the inversion, opposite sign
                    const int32_t expected = static_cast<int32_t>(CYCLES * counts_per_cycle(mode)) * (a_leads_b ? -1 : 1);
                    TEST_ASSERT_EQUAL_INT32(expected, read_signed_count());
                }
            }
        }

        void qdec_filter_rejects_glitches_on_both_channels() {
            lb::reset_fixture();
            start_counting_from_idle();

            const auto cfg = make_qdec(ENCODER_MODE_3, false, 500);
            TEST_ASSERT_EQUAL(HAL_OK, qdec_init(MEAS, &cfg));
            TEST_ASSERT_EQUAL(HAL_OK, pcnt_set_count(MEAS, 0));
            TEST_ASSERT_EQUAL(HAL_OK, pcnt_start(MEAS));

            lb::glitches(lb::GEN_A, 50);
            lb::glitches(lb::GEN_B, 50);
            delay_us(50);
            TEST_ASSERT_EQUAL_INT32(0, read_signed_count());

            lb::quad_walk(25, true); // 4us per state is well above the filter
            delay_us(50);
            TEST_ASSERT_EQUAL_INT32(100, read_signed_count());
        }

        void qdec_update_callback_reports_the_direction_of_overflow_and_underflow() {
            lb::reset_fixture();
            start_counting_from_idle();
            reset_update_log();

            const auto cfg = make_qdec(ENCODER_MODE_3, false, 0, {update_cb, nullptr});
            TEST_ASSERT_EQUAL(HAL_OK, qdec_init(MEAS, &cfg));
            TEST_ASSERT_EQUAL(HAL_OK, pcnt_set_count(MEAS, 0));
            TEST_ASSERT_EQUAL(HAL_OK, pcnt_start(MEAS));

            // 16400 cycles forward is 65600 counts: one wrap of the 16 bit counter, ending 64 counts past zero
            lb::quad_walk(16'400, true);
            delay_us(50);
            TEST_ASSERT_EQUAL_UINT32(1U, s_update_events);
            TEST_ASSERT_EQUAL(TIMER_COUNTER_DIR_UP, s_update_dir[0]);
            TEST_ASSERT_EQUAL_UINT32(64U, read_count());

            // 100 cycles back is 400 counts: straight through zero and out the bottom
            lb::quad_walk(100, false);
            delay_us(50);
            TEST_ASSERT_EQUAL_UINT32(2U, s_update_events);
            TEST_ASSERT_EQUAL(TIMER_COUNTER_DIR_DOWN, s_update_dir[1]);
            TEST_ASSERT_EQUAL_UINT32(65'536U - 336U, read_count());
        }

    } // namespace

    void all() {
        LOGI(TAG, "Starting the tests on the PCNT driver");
        UNITY_BEGIN();

        RUN_TEST(pcnt_init_rejects_invalid_arguments);
        RUN_TEST(pcnt_mode_1_programs_the_slave_external_clock_mode);
        RUN_TEST(pcnt_mode_2_programs_the_external_trigger_path);
        RUN_TEST(pcnt_init_programs_the_clock_divider_and_the_overflow_callback);
        RUN_TEST(pcnt_init_configures_the_gpio_pin);
        RUN_TEST(pcnt_mode_1_counts_exactly_on_the_selected_edge);
        RUN_TEST(pcnt_mode_2_counts_exactly_and_the_etr_prescaler_divides);
        RUN_TEST(pcnt_mode_2_counts_exactly_on_the_selected_edge);
        RUN_TEST(pcnt_filter_rejects_glitches_in_both_modes);
        RUN_TEST(pcnt_start_and_stop_freeze_and_resume_the_count);
        RUN_TEST(pcnt_set_and_get_count_respect_the_counter_width);
        RUN_TEST(pcnt_deinit_stops_counting_and_clears_the_slave_configuration);
        RUN_TEST(pcnt_and_qdec_init_overwrite_each_other);
        RUN_TEST(qdec_init_rejects_invalid_arguments);
        RUN_TEST(qdec_programs_the_encoder_mode_and_both_inputs);
        RUN_TEST(qdec_configures_the_pull_resistors_on_both_pins);
        RUN_TEST(qdec_counts_each_mode_in_both_directions);
        RUN_TEST(qdec_walking_forward_then_back_returns_to_zero);
        RUN_TEST(qdec_invert_direction_reverses_the_count);
        RUN_TEST(qdec_filter_rejects_glitches_on_both_channels);
        RUN_TEST(qdec_update_callback_reports_the_direction_of_overflow_and_underflow);
        RUN_TEST(pcnt_overflow_callback_fires_once_per_wrap_of_the_counter);
        RUN_TEST(pcnt_frequency_of_the_generated_clock_matches_across_system_clocks);

        // Leave the timers and clocks the way they were found
        (void)pcnt_deinit(MEAS);
        lb::generator_stop();
        (void)timx_clk_enable(lb::MEAS, false);
        (void)timx_clk_enable(lb::GEN, false);

        UNITY_END();
        LOGI(TAG, "Done with all tests on the PCNT driver");
    }

} // namespace test::timer_pcnt
