#include "stm32f411xe.h"
#include "Unity/unity.h"

#include "drivers/src/timer_internals.h"
#include "drivers/timer_extended.h"
#include "test/timer_extended.hpp"
#include "drivers/timer_types.h"
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


// Input capture, output compare (toggle mode) and the digital filter helper they share.
// Wiring and the signal source helpers live in test/loopback.hpp.
//
// ASSUMPTION: input_capture_config_t has a new timer wide 'timer_prescaler' field (PSC = timer_prescaler - 1),
// 0 and a count_mode of TIMER_COUNTER_DIR_NONE are rejected with HAL_ERR_INVALID_ARG, and
// input_capture_get_timer_frequency(...) reports the tick rate (timer clock / timer_prescaler).

namespace test::timer_ext {

    namespace {

        constexpr const char* TAG = "Timer_ext_Test";

        TIM_TypeDef* const MEAS = loopback::MEASURED;
        TIM_TypeDef* const GEN  = loopback::GENERATOR;

        // The measurer counts in 1us ticks in most of these tests
        constexpr uint32_t TICK_HZ = 1'000'000U;

        // Helpers
        // Event log filled in by the capture callback
        constexpr size_t LOG_N = 32;

        struct capture_log_t {
            volatile uint32_t events; // Every callback
            volatile uint32_t stored; // Callbacks that fit in the arrays below

            std::array<bool, LOG_N>            level;
            std::array<timer_channel_t, LOG_N> channel;

            std::array<uint32_t, LOG_N> cycles;
            std::array<uint32_t, LOG_N> value;
        };

        capture_log_t s_log;

        void reset_log() {
            s_log.events = 0;
            s_log.stored = 0;
        }

        void log_cb(void*, timer_channel_t channel, timer_count_dir_t) {
            uint32_t value = 0;
            (void)input_capture_get_sample(MEAS, channel, &value);

            const uint32_t i = s_log.stored;
            if (i < LOG_N) {
                s_log.value[i]  = value;
                s_log.cycles[i] = DWT->CYCCNT;
                s_log.level[i] =
                    gpio_get_level(loopback::IN_A.port, loopback::IN_A.pin);
                s_log.channel[i] = channel;
                s_log.stored     = i + 1;
            }
            s_log.events = s_log.events + 1;
        }

        volatile uint32_t        s_oc_events       = 0;
        volatile timer_channel_t s_oc_last_channel = TIMER_CHANNEL_NC;

        void oc_cb(void*, timer_channel_t channel, timer_count_dir_t) {
            s_oc_last_channel = channel;
            s_oc_events       = s_oc_events + 1;
        }

        input_capture_config_t make_ic_config(uint32_t   prescaler,
                                              ic_edge_t  edge,
                                              uint32_t   filter_ns = 0,
                                              timer_cb_t callback  = {}) {
            input_capture_config_t cfg{};
            cfg.clk_div         = TIM_CLK_DIV_1;
            cfg.count_mode      = TIMER_COUNTER_DIR_UP;
            cfg.timer_prescaler = static_cast<uint16_t>(prescaler);
            cfg.channels[0]     = {
                .edge      = edge,
                .pull      = TIMER_PULL_NONE,
                .prescaler = IC_PRESCALER_DIV_1,
                .channel   = TIMER_CHANNEL_1,
                .gpio_pin  = loopback::IN_A,
                .filter_ns = filter_ns,
            };
            cfg.num_channels = 1;
            cfg.callback     = callback;
            return cfg;
        }

        void start_capture(const input_capture_config_t& cfg) {
            TEST_ASSERT_EQUAL(HAL_OK, input_capture_init(MEAS, &cfg));
            TEST_ASSERT_EQUAL(HAL_OK, input_capture_start(MEAS));
        }

        // Fixed hardware register layout of an input capture channel (CCMR/CCER/DIER). Independent of the driver
        struct ic_regs_t {
            volatile uint32_t* ccmr;
            uint32_t ccmr_shift; // CCxS at +0, ICxPSC at +2, ICxF at +4
            uint32_t ccer_shift; // CCxE at +0, CCxP at +1, CCxNP at +3
            uint32_t dier_bit;
        };

        ic_regs_t get_ic_regs(TIM_TypeDef* handle, timer_channel_t channel) {
            const uint32_t n = std::to_underlying(channel); // 0 based
            return {
                .ccmr       = (n < 2U) ? &handle->CCMR1 : &handle->CCMR2,
                .ccmr_shift = (n & 1U) * 8U,
                .ccer_shift = n * 4U,
                .dier_bit   = uint32_t{1} << (n + 1U),
            };
        }

        constexpr uint32_t ALL_CCXE =
            TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
        constexpr uint32_t ALL_CCXIE =
            TIM_DIER_CC1IE | TIM_DIER_CC2IE | TIM_DIER_CC3IE | TIM_DIER_CC4IE;

        // Independent model of the digital filter, straight from the reference manual's ICxF table. Number of timer
        // clock ticks an input must stay stable for a given filter code, with and without the DTS clock divider
        struct filter_entry_t {
            uint32_t
                sampling_div; // fSAMPLING = fCK_INT (code 1-3) or fDTS (the rest) divided by this
            uint32_t
                samples; // Consecutive samples needed to validate a level change
            bool uses_dts;
        };

        constexpr std::array<filter_entry_t, 16> FILTER_TABLE = {{
            {.sampling_div = 1,
             .samples      = 0,
             .uses_dts     = false}, // Code 0: no filter
            {.sampling_div = 1, .samples = 2, .uses_dts = false},
            {.sampling_div = 1, .samples = 4, .uses_dts = false},
            {.sampling_div = 1, .samples = 8, .uses_dts = false},
            {.sampling_div = 2, .samples = 6, .uses_dts = true},
            {.sampling_div = 2, .samples = 8, .uses_dts = true},
            {.sampling_div = 4, .samples = 6, .uses_dts = true},
            {.sampling_div = 4, .samples = 8, .uses_dts = true},
            {.sampling_div = 8, .samples = 6, .uses_dts = true},
            {.sampling_div = 8, .samples = 8, .uses_dts = true},
            {.sampling_div = 16, .samples = 5, .uses_dts = true},
            {.sampling_div = 16, .samples = 6, .uses_dts = true},
            {.sampling_div = 16, .samples = 8, .uses_dts = true},
            {.sampling_div = 32, .samples = 5, .uses_dts = true},
            {.sampling_div = 32, .samples = 6, .uses_dts = true},
            {.sampling_div = 32, .samples = 8, .uses_dts = true},
        }};

        uint32_t filter_ticks(uint32_t code, tim_clk_div_t clk_div) {
            const filter_entry_t& entry = FILTER_TABLE[code];
            return entry.samples * entry.sampling_div *
                   (entry.uses_dts ? (1U << std::to_underlying(clk_div)) : 1U);
        }

        // TESTS
        // Input capture: argument validation and register programming
        void ic_init_rejects_invalid_arguments() {
            loopback::reset_fixture();
            const auto good = make_ic_config(loopback::prescaler_for_1us(MEAS),
                                             IC_EDGE_RISING);

            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG,
                              input_capture_init(nullptr, &good));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG,
                              input_capture_init(MEAS, nullptr));

            auto bad         = good;
            bad.num_channels = 0;
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG,
                              input_capture_init(MEAS, &bad));

            bad              = good;
            bad.num_channels = timer_get_num_channels(MEAS) + 1;
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG,
                              input_capture_init(MEAS, &bad));

            bad                 = good;
            bad.timer_prescaler = 0;
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG,
                              input_capture_init(MEAS, &bad));

            bad            = good;
            bad.count_mode = TIMER_COUNTER_DIR_NONE;
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG,
                              input_capture_init(MEAS, &bad));

            bad                     = good;
            bad.channels[0].channel = TIMER_CHANNEL_NC;
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG,
                              input_capture_init(MEAS, &bad));

            // Channels the lite timers don't have
            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM9, true));
            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM10, true));

            bad                     = good;
            bad.channels[0].channel = TIMER_CHANNEL_3;
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG,
                              input_capture_init(TIM9, &bad));

            bad                     = good;
            bad.channels[0].channel = TIMER_CHANNEL_2;
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG,
                              input_capture_init(TIM10, &bad));

            TEST_ASSERT_EQUAL(HAL_OK, timer_deinit(TIM9));
            TEST_ASSERT_EQUAL(HAL_OK, timer_deinit(TIM10));
            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM9, false));
            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM10, false));
        }

        void ic_init_rejects_a_locked_advanced_timer() {
            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM1, true));
            TEST_ASSERT_EQUAL(HAL_OK, timer_reset(TIM1));

            const auto good = make_ic_config(1, IC_EDGE_RISING);
            TIM1->BDTR |=
                TIM_BDTR_LOCK; // Write protection level 3. Only a peripheral reset clears it
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_STATE,
                              input_capture_init(TIM1, &good));

            TEST_ASSERT_EQUAL(HAL_OK, timer_reset(TIM1));
            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM1, false));
        }

        void ic_init_programs_every_channel_edge_and_prescaler() {
            loopback::reset_fixture();

            const std::array<board_pin_t, 4> pins = {{
                BOARD_TIM3_CH1_PB4,
                BOARD_TIM3_CH2_PB5,
                BOARD_TIM3_CH3_PB0,
                BOARD_TIM3_CH4_PB1,
            }};

            struct edge_case_t {
                ic_edge_t edge;
                bool      ccxp;
                bool      ccxnp;
            };
            const std::array<edge_case_t, 3> edges = {{
                {.edge = IC_EDGE_RISING, .ccxp = false, .ccxnp = false},
                {.edge = IC_EDGE_FALLING, .ccxp = true, .ccxnp = false},
                {.edge = IC_EDGE_RISING_FALLING, .ccxp = true, .ccxnp = true},
            }};
            const auto prescalers = std::array{IC_PRESCALER_DIV_1,
                                               IC_PRESCALER_DIV_2,
                                               IC_PRESCALER_DIV_4,
                                               IC_PRESCALER_DIV_8};

            for (uint32_t c = 0; c < 4; c++) {
                const auto channel = static_cast<timer_channel_t>(c);
                const auto regs    = get_ic_regs(MEAS, channel);

                for (const auto& edge : edges) {
                    for (const auto prescaler : prescalers) {
                        auto cfg                 = make_ic_config(1, edge.edge);
                        cfg.channels[0].channel  = channel;
                        cfg.channels[0].gpio_pin = pins[c];
                        cfg.channels[0].prescaler = prescaler;
                        TEST_ASSERT_EQUAL(HAL_OK,
                                          input_capture_init(MEAS, &cfg));

                        const uint32_t ccmr = *regs.ccmr >> regs.ccmr_shift;
                        TEST_ASSERT_EQUAL_UINT32(
                            0b01U,
                            ccmr & 0b11U); // CCxS: mapped to its own TIx
                        TEST_ASSERT_EQUAL_UINT32(std::to_underlying(prescaler),
                                                 (ccmr >> 2U) &
                                                     0b11U); // ICxPSC
                        TEST_ASSERT_EQUAL_UINT32(
                            0U,
                            (ccmr >> 4U) & 0xFU); // ICxF: no filter requested

                        const uint32_t ccer = MEAS->CCER >> regs.ccer_shift;
                        TEST_ASSERT_TRUE(ccer & 1U); // CCxE
                        TEST_ASSERT_EQUAL(edge.ccxp, (ccer >> 1U) & 1U);
                        TEST_ASSERT_EQUAL(edge.ccxnp, (ccer >> 3U) & 1U);

                        // No other channel got enabled, and no interrupt without a callback
                        TEST_ASSERT_EQUAL_UINT32(1UL << regs.ccer_shift,
                                                 MEAS->CCER & ALL_CCXE);
                        TEST_ASSERT_EQUAL_UINT32(0U, MEAS->DIER & ALL_CCXIE);
                    }
                }
            }
        }

        void
        ic_init_enables_only_the_interrupts_of_configured_channels_when_a_callback_is_given() {
            loopback::reset_fixture();

            auto cfg = make_ic_config(1, IC_EDGE_RISING, 0, {log_cb, nullptr});
            cfg.channels[1]  = {.edge      = IC_EDGE_FALLING,
                                .pull      = TIMER_PULL_NONE,
                                .prescaler = IC_PRESCALER_DIV_1,
                                .channel   = TIMER_CHANNEL_2,
                                .gpio_pin  = loopback::IN_B,
                                .filter_ns = 0};
            cfg.num_channels = 2;
            TEST_ASSERT_EQUAL(HAL_OK, input_capture_init(MEAS, &cfg));

            TEST_ASSERT_EQUAL_UINT32(TIM_DIER_CC1IE | TIM_DIER_CC2IE,
                                     MEAS->DIER & ALL_CCXIE);
            TEST_ASSERT_EQUAL_UINT32(TIM_CCER_CC1E | TIM_CCER_CC2E,
                                     MEAS->CCER & ALL_CCXE);
            TEST_ASSERT_TRUE(NVIC->ISER[TIM3_IRQn >> 5] &
                             (1UL << (TIM3_IRQn & 0x1FU)));
        }

        void ic_init_programs_the_timer_wide_settings() {
            loopback::reset_fixture();

            auto cfg    = make_ic_config(7, IC_EDGE_RISING);
            cfg.clk_div = TIM_CLK_DIV_4;
            TEST_ASSERT_EQUAL(HAL_OK, input_capture_init(MEAS, &cfg));

            TEST_ASSERT_EQUAL_UINT32(6U, MEAS->PSC);
            TEST_ASSERT_EQUAL_UINT32(UINT16_MAX,
                                     MEAS->ARR); // Free running, 16 bit
            TEST_ASSERT_EQUAL_UINT32(std::to_underlying(TIM_CLK_DIV_4),
                                     (MEAS->CR1 & TIM_CR1_CKD) >>
                                         TIM_CR1_CKD_Pos);
            TEST_ASSERT_TRUE(MEAS->CR1 & TIM_CR1_ARPE);
            TEST_ASSERT_FALSE(MEAS->CR1 &
                              TIM_CR1_CEN); // Not counting until started
            TEST_ASSERT_FALSE(MEAS->CR1 & TIM_CR1_DIR);

            cfg            = make_ic_config(1, IC_EDGE_RISING);
            cfg.count_mode = TIMER_COUNTER_DIR_DOWN;
            TEST_ASSERT_EQUAL(HAL_OK, input_capture_init(MEAS, &cfg));
            TEST_ASSERT_TRUE(MEAS->CR1 & TIM_CR1_DIR);

            // A 32 bit timer gets the full 32 bit reload value
            cfg                      = make_ic_config(1, IC_EDGE_RISING);
            cfg.channels[0].gpio_pin = loopback::GEN_A;
            TEST_ASSERT_EQUAL(HAL_OK, input_capture_init(GEN, &cfg));
            TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, GEN->ARR);
        }

        void ic_init_configures_the_gpio_pin() {
            loopback::reset_fixture();

            struct pull_case_t {
                timer_pull_t pull;
                uint32_t     pupdr;
            };
            const std::array<pull_case_t, 3> pulls = {{
                {.pull = TIMER_PULL_NONE, .pupdr = 0b00},
                {.pull = TIMER_USE_PULLUP, .pupdr = 0b01},
                {.pull = TIMER_USE_PULLDOWN, .pupdr = 0b10},
            }};

            for (const auto& p : pulls) {
                auto cfg             = make_ic_config(1, IC_EDGE_RISING);
                cfg.channels[0].pull = p.pull;
                TEST_ASSERT_EQUAL(HAL_OK, input_capture_init(MEAS, &cfg));

                TEST_ASSERT_EQUAL_UINT32(
                    0b10U,
                    loopback::moder(loopback::IN_A)); // Alternate function
                TEST_ASSERT_EQUAL_UINT32(loopback::IN_A.af,
                                         loopback::afr(loopback::IN_A));
                TEST_ASSERT_EQUAL_UINT32(p.pupdr,
                                         loopback::pupdr(loopback::IN_A));
            }
        }

        void ic_timer_prescaler_sets_psc_and_the_reported_tick_rate() {
            loopback::reset_fixture();

            const uint32_t                clk = timer_get_frequency_hz(MEAS);
            const std::array<uint32_t, 4> prescalers = {1, 2, 7, clk / TICK_HZ};

            for (const uint32_t prescaler : prescalers) {
                const auto cfg = make_ic_config(prescaler, IC_EDGE_RISING);
                TEST_ASSERT_EQUAL(HAL_OK, input_capture_init(MEAS, &cfg));
                TEST_ASSERT_EQUAL_UINT32(prescaler - 1U, MEAS->PSC);

                uint32_t hz = 0;
                TEST_ASSERT_EQUAL(HAL_OK,
                                  input_capture_get_timer_frequency(MEAS, &hz));
                TEST_ASSERT_EQUAL_UINT32(
                    clk / prescaler,
                    hz); // Ticks per second, not the timer's input clock
            }

            // 1us ticks
            uint32_t   hz  = 0;
            const auto cfg = make_ic_config(loopback::prescaler_for_1us(MEAS),
                                            IC_EDGE_RISING);
            TEST_ASSERT_EQUAL(HAL_OK, input_capture_init(MEAS, &cfg));
            TEST_ASSERT_EQUAL(HAL_OK,
                              input_capture_get_timer_frequency(MEAS, &hz));
            TEST_ASSERT_EQUAL_UINT32(TICK_HZ, hz);

            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG,
                              input_capture_get_timer_frequency(nullptr, &hz));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG,
                              input_capture_get_timer_frequency(MEAS, nullptr));
        }

        void ic_start_and_stop_run_and_freeze_the_counter() {
            loopback::reset_fixture();
            const auto cfg = make_ic_config(loopback::prescaler_for_1us(MEAS),
                                            IC_EDGE_RISING);
            TEST_ASSERT_EQUAL(HAL_OK, input_capture_init(MEAS, &cfg));

            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG,
                              input_capture_start(nullptr));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, input_capture_stop(nullptr));

            TEST_ASSERT_EQUAL(HAL_OK, input_capture_start(MEAS));
            TEST_ASSERT_TRUE(MEAS->CR1 & TIM_CR1_CEN);
            delay_us(100);
            TEST_ASSERT_TRUE(MEAS->CNT > 0U);

            TEST_ASSERT_EQUAL(HAL_OK, input_capture_stop(MEAS));
            TEST_ASSERT_FALSE(MEAS->CR1 & TIM_CR1_CEN);
            const uint32_t held = MEAS->CNT;
            delay_us(100);
            TEST_ASSERT_EQUAL_UINT32(held, MEAS->CNT);
        }

        void ic_deinit_clears_the_state_and_silences_the_callback() {
            loopback::reset_fixture();
            TEST_ASSERT_EQUAL(HAL_OK,
                              loopback::generator_toggle_start(
                                  TIMER_CHANNEL_1, loopback::GEN_A, 2'000));

            reset_log();
            start_capture(make_ic_config(loopback::prescaler_for_1us(MEAS),
                                         IC_EDGE_RISING,
                                         0,
                                         {log_cb, nullptr}));
            TEST_ASSERT_TRUE_MESSAGE(loopback::wait_for(
                                         []() {
                                             return s_log.events >= 2;
                                         },
                                         100),
                                     "No capture events before the deinit");

            TEST_ASSERT_EQUAL(HAL_OK, input_capture_deinit(MEAS));
            TEST_ASSERT_FALSE(MEAS->CR1 & TIM_CR1_CEN);
            TEST_ASSERT_EQUAL_UINT32(0U, MEAS->CCER & ALL_CCXE);
            TEST_ASSERT_EQUAL_UINT32(0U, MEAS->DIER & ALL_CCXIE);
            TEST_ASSERT_EQUAL_UINT32(
                0U, MEAS->CCMR1 & (TIM_CCMR1_CC1S | TIM_CCMR1_CC2S));
            TEST_ASSERT_EQUAL_UINT32(
                0U, MEAS->CCMR2 & (TIM_CCMR2_CC3S | TIM_CCMR2_CC4S));
            TEST_ASSERT_EQUAL_UINT32(0U, MEAS->PSC);
            TEST_ASSERT_EQUAL_UINT32(0U, MEAS->ARR);
            TEST_ASSERT_FALSE(NVIC->ISER[TIM3_IRQn >> 5] &
                              (1UL << (TIM3_IRQn & 0x1FU)));

            const uint32_t before = s_log.events;
            delay_ms(5); // Ten edges worth of time on the generator
            TEST_ASSERT_EQUAL_UINT32(before, s_log.events);
        }

        // Input capture: measuring the signal source
        void ic_measures_the_generated_clock_period() {
            loopback::reset_fixture();
            TEST_ASSERT_EQUAL(HAL_OK,
                              loopback::generator_toggle_start(
                                  TIMER_CHANNEL_1, loopback::GEN_A, 2'000));
            const uint32_t pin_hz = loopback::generator_pin_hz();
            TEST_ASSERT_EQUAL_UINT32(2'000U, pin_hz);

            reset_log();
            start_capture(make_ic_config(loopback::prescaler_for_1us(MEAS),
                                         IC_EDGE_RISING,
                                         0,
                                         {log_cb, nullptr}));
            TEST_ASSERT_TRUE_MESSAGE(loopback::wait_for(
                                         []() {
                                             return s_log.stored >= 12;
                                         },
                                         200),
                                     "Not enough capture events");
            TEST_ASSERT_EQUAL(HAL_OK, input_capture_stop(MEAS));

            const uint32_t expected_ticks  = TICK_HZ / pin_hz;
            const uint32_t expected_cycles = get_system_core_clock() / pin_hz;
            for (uint32_t i = 1; i < 12; i++) {
                const uint32_t delta_ticks =
                    (s_log.value[i] - s_log.value[i - 1]) &
                    UINT16_MAX; // 16 bit counter
                TEST_ASSERT_UINT32_WITHIN(1U, expected_ticks, delta_ticks);

                // The same period against the CPU's cycle counter. ISR entry jitter is a few us at most
                const uint32_t delta_cycles =
                    s_log.cycles[i] - s_log.cycles[i - 1];
                TEST_ASSERT_UINT32_WITHIN(
                    expected_cycles / 100U, expected_cycles, delta_cycles);
            }
        }

        void ic_both_edges_capture_the_half_periods_of_a_square_wave() {
            loopback::reset_fixture();
            TEST_ASSERT_EQUAL(HAL_OK,
                              loopback::generator_toggle_start(
                                  TIMER_CHANNEL_1, loopback::GEN_A, 2'000));

            reset_log();
            start_capture(make_ic_config(loopback::prescaler_for_1us(MEAS),
                                         IC_EDGE_RISING_FALLING,
                                         0,
                                         {log_cb, nullptr}));
            TEST_ASSERT_TRUE(loopback::wait_for(
                []() {
                    return s_log.stored >= 16;
                },
                200));
            TEST_ASSERT_EQUAL(HAL_OK, input_capture_stop(MEAS));

            for (uint32_t i = 1; i < 16; i++) {
                TEST_ASSERT_UINT32_WITHIN(
                    1U,
                    TICK_HZ / 4'000U,
                    (s_log.value[i] - s_log.value[i - 1]) &
                        UINT16_MAX); // 50% duty: 250us each
            }
        }

        void ic_edge_selection_follows_the_pin_level_at_the_capture() {
            loopback::reset_fixture();
            uint32_t compare = 0;
            uint32_t max     = 0;
            TEST_ASSERT_EQUAL(
                HAL_OK,
                loopback::generator_pwm_start(
                    1'000, 25, &compare, &max)); // 250us high, 750us low

            const uint64_t gen_clk = timer_get_frequency_hz(GEN);
            const uint32_t high_ticks =
                (uint32_t)((uint64_t)compare * TICK_HZ / gen_clk);
            const uint32_t low_ticks =
                (uint32_t)((uint64_t)(max - compare) * TICK_HZ / gen_clk);

            // The pin is read inside the capture ISR a couple of us after the edge, far below the 250us high time
            struct edge_case_t {
                ic_edge_t edge;
                bool      expected_level;
            };

            for (const auto& c : std::array<edge_case_t, 2>{{
                     {.edge = IC_EDGE_RISING, .expected_level = true},
                     {.edge = IC_EDGE_FALLING, .expected_level = false},
                 }}) {
                reset_log();
                start_capture(make_ic_config(loopback::prescaler_for_1us(MEAS),
                                             c.edge,
                                             0,
                                             {log_cb, nullptr}));
                TEST_ASSERT_TRUE(loopback::wait_for(
                    []() {
                        return s_log.stored >= 8;
                    },
                    200));
                TEST_ASSERT_EQUAL(HAL_OK, input_capture_stop(MEAS));

                for (uint32_t i = 0; i < 8; i++) {
                    TEST_ASSERT_EQUAL(c.expected_level, s_log.level[i]);
                }
            }

            // Both edges: levels alternate, and the gap after a rising edge is the high time, after a falling edge the low time
            reset_log();
            start_capture(make_ic_config(loopback::prescaler_for_1us(MEAS),
                                         IC_EDGE_RISING_FALLING,
                                         0,
                                         {.cb = log_cb, .arg = nullptr}));
            TEST_ASSERT_TRUE(loopback::wait_for(
                []() {
                    return s_log.stored >= 12;
                },
                200));
            TEST_ASSERT_EQUAL(HAL_OK, input_capture_stop(MEAS));

            for (uint32_t i = 0; i + 1 < 12; i++) {
                TEST_ASSERT_NOT_EQUAL(s_log.level[i], s_log.level[i + 1]);
                const uint32_t gap =
                    (s_log.value[i + 1] - s_log.value[i]) & UINT16_MAX;
                TEST_ASSERT_UINT32_WITHIN(
                    2U, s_log.level[i] ? high_ticks : low_ticks, gap);
            }
        }

        void ic_channels_capture_independently() {
            loopback::reset_fixture();
            // Only generator B (-> PB5, timer channel 2) toggles. Channel 1's pin idles low
            TEST_ASSERT_EQUAL(HAL_OK,
                              loopback::generator_toggle_start(
                                  TIMER_CHANNEL_2, loopback::GEN_B, 2'000));

            auto cfg = make_ic_config(loopback::prescaler_for_1us(MEAS),
                                      IC_EDGE_RISING,
                                      0,
                                      {.cb = log_cb, .arg = nullptr});
            cfg.channels[0].pull = TIMER_USE_PULLDOWN;
            cfg.channels[1]      = {
                .edge      = IC_EDGE_RISING,
                .pull      = TIMER_USE_PULLDOWN,
                .prescaler = IC_PRESCALER_DIV_1,
                .channel   = TIMER_CHANNEL_2,
                .gpio_pin  = loopback::IN_B,
                .filter_ns = 0,
            };
            cfg.num_channels = 2;

            reset_log();
            TEST_ASSERT_EQUAL(HAL_OK, input_capture_init(MEAS, &cfg));
            TEST_ASSERT_EQUAL(HAL_OK, input_capture_start(MEAS));
            TEST_ASSERT_TRUE(loopback::wait_for(
                []() {
                    return s_log.stored >= 8;
                },
                200));
            TEST_ASSERT_EQUAL(HAL_OK, input_capture_stop(MEAS));

            for (uint32_t i = 0; i < 8; i++) {
                TEST_ASSERT_EQUAL(TIMER_CHANNEL_2, s_log.channel[i]);
            }
            for (uint32_t i = 1; i < 8; i++) {
                TEST_ASSERT_UINT32_WITHIN(
                    1U,
                    TICK_HZ / 2'000U,
                    (s_log.value[i] - s_log.value[i - 1]) & UINT16_MAX);
            }
        }

        // Input capture: digital filter, input prescaler and overcapture
        void ic_digital_filter_rejects_glitches_and_passes_real_pulses() {
            loopback::reset_fixture();
            loopback::take_over_pin(loopback::GEN_A, false);

            reset_log();
            start_capture(make_ic_config(loopback::prescaler_for_1us(MEAS),
                                         IC_EDGE_RISING,
                                         500,
                                         {.cb = log_cb, .arg = nullptr}));
            delay_us(20);

            loopback::glitches(loopback::GEN_A, 50);
            delay_us(100);
            TEST_ASSERT_EQUAL_UINT32(0U, s_log.events);

            loopback::pulses(loopback::GEN_A, 50, 20, 20);
            delay_us(100);
            TEST_ASSERT_EQUAL_UINT32(50U, s_log.events);
        }

        void ic_input_prescaler_divides_the_event_rate() {
            struct div_case_t {
                ic_prescaler_t prescaler;
                uint32_t       divisor;
            };
            const std::array<div_case_t, 4> cases = {{
                {.prescaler = IC_PRESCALER_DIV_1, .divisor = 1},
                {.prescaler = IC_PRESCALER_DIV_2, .divisor = 2},
                {.prescaler = IC_PRESCALER_DIV_4, .divisor = 4},
                {.prescaler = IC_PRESCALER_DIV_8, .divisor = 8},
            }};

            for (const auto& c : cases) {
                loopback::reset_fixture();
                loopback::take_over_pin(loopback::GEN_A, false);

                auto cfg = make_ic_config(loopback::prescaler_for_1us(MEAS),
                                          IC_EDGE_RISING,
                                          0,
                                          {.cb = log_cb, .arg = nullptr});
                cfg.channels[0].prescaler = c.prescaler;
                reset_log();
                start_capture(cfg);
                delay_us(20);

                loopback::pulses(loopback::GEN_A, 64, 20, 20);
                delay_us(100);
                TEST_ASSERT_EQUAL_UINT32(64U / c.divisor, s_log.events);
            }
        }

        void ic_overcapture_is_reported_once_then_cleared() {
            loopback::reset_fixture();
            TEST_ASSERT_EQUAL(HAL_OK,
                              loopback::generator_toggle_start(
                                  TIMER_CHANNEL_1, loopback::GEN_A, 2'000));

            // No callback: nothing reads the capture register, so later edges overwrite the first one
            start_capture(make_ic_config(loopback::prescaler_for_1us(MEAS),
                                         IC_EDGE_RISING));
            delay_ms(5);

            uint32_t sample = 0;
            TEST_ASSERT_EQUAL(
                HAL_ERR_TIM_OVER_CAPTURE,
                input_capture_get_sample(MEAS, TIMER_CHANNEL_1, &sample));
            TEST_ASSERT_FALSE(MEAS->SR & TIM_SR_CC1OF);
            TEST_ASSERT_EQUAL(
                HAL_OK,
                input_capture_get_sample(
                    MEAS, TIMER_CHANNEL_1, &sample)); // No new edge yet: clean
        }

        void ic_get_sample_rejects_invalid_arguments() {
            uint32_t sample = 0;
            TEST_ASSERT_EQUAL(
                HAL_ERR_INVALID_ARG,
                input_capture_get_sample(nullptr, TIMER_CHANNEL_1, &sample));
            TEST_ASSERT_EQUAL(
                HAL_ERR_INVALID_ARG,
                input_capture_get_sample(MEAS, TIMER_CHANNEL_1, nullptr));
            TEST_ASSERT_EQUAL(
                HAL_ERR_INVALID_ARG,
                input_capture_get_sample(MEAS, TIMER_CHANNEL_NC, &sample));

            TEST_ASSERT_EQUAL(
                HAL_ERR_NOT_SUPPORTED,
                input_capture_get_sample(TIM10, TIMER_CHANNEL_2, &sample));
            TEST_ASSERT_EQUAL(
                HAL_ERR_NOT_SUPPORTED,
                input_capture_get_sample(TIM10, TIMER_CHANNEL_3, &sample));
            TEST_ASSERT_EQUAL(
                HAL_ERR_NOT_SUPPORTED,
                input_capture_get_sample(TIM9, TIMER_CHANNEL_3, &sample));
            TEST_ASSERT_EQUAL(
                HAL_ERR_NOT_SUPPORTED,
                input_capture_get_sample(TIM9, TIMER_CHANNEL_4, &sample));
        }

        // The digital filter code helper shared by input capture, the pulse counter, the decoder and PWM input
        void filter_code_is_the_smallest_that_covers_the_request() {
            loopback::reset_fixture();

            const uint64_t clk = timer_get_frequency_hz(MEAS);
            constexpr auto dividers =
                std::array{TIM_CLK_DIV_1, TIM_CLK_DIV_2, TIM_CLK_DIV_4};
            constexpr auto requests_ns = std::array{1UL,
                                                    10UL,
                                                    40UL,
                                                    100UL,
                                                    500UL,
                                                    1'000UL,
                                                    5'000UL,
                                                    10'000UL,
                                                    100'000UL};

            for (const auto divider : dividers) {
                for (const uint32_t ns : requests_ns) {
                    const uint32_t required_ticks =
                        (uint32_t)((clk * ns + 999'999'999ULL) /
                                   1'000'000'000ULL);

                    uint32_t expected =
                        0xFFU; // Smallest code whose stable time reaches the request
                    for (uint32_t code = 1; code < FILTER_TABLE.size();
                         code++) {
                        if (filter_ticks(code, divider) >= required_ticks) {
                            expected = code;
                            break;
                        }
                    }

                    uint32_t        code = 0xFFU;
                    const hal_err_t err =
                        timer_filter_ns_to_ic_code(MEAS, divider, ns, &code);
                    if (expected == 0xFFU) {
                        TEST_ASSERT_EQUAL(HAL_ERR_NOT_SUPPORTED, err);
                    } else {
                        TEST_ASSERT_EQUAL(HAL_OK, err);
                        TEST_ASSERT_EQUAL_UINT32(expected, code);
                    }
                }

                uint32_t code = 0xFFU;
                TEST_ASSERT_EQUAL(
                    HAL_OK,
                    timer_filter_ns_to_ic_code(MEAS, divider, 0, &code));
                TEST_ASSERT_EQUAL_UINT32(
                    0U, code); // No filter requested, no filter programmed
            }
        }

        void filter_code_rejects_invalid_arguments() {
            uint32_t code = 0;
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

        // Output compare, toggle mode (the PWM driver covers the PWM modes)
        void oc_compare_callback_fires_once_per_period() {
            loopback::reset_fixture();

            output_compare_config_t cfg{};
            cfg.buffer_compare_reload = false;
            cfg.mode                  = OUTPUT_COMPARE_TOGGLE_ON_MATCH;
            cfg.count_mode            = OUTPUT_COMPARE_EDGE_LEFT_ALIGNED;
            cfg.channels[0]           = {.output_polarity   = false,
                                         .output_idle_state = false,
                                         .channel           = TIMER_CHANNEL_1,
                                         .gpio_pin          = loopback::GEN_A};
            cfg.num_channels          = 1;
            cfg.callback              = {.cb = oc_cb, .arg = nullptr};
            TEST_ASSERT_EQUAL(HAL_OK, output_compare_init(GEN, &cfg));

            uint32_t max_compare = 0;
            TEST_ASSERT_EQUAL(
                HAL_OK,
                output_compare_start(
                    GEN, 1'000, &max_compare)); // 1000 periods per second
            TEST_ASSERT_EQUAL(HAL_OK,
                              output_compare_set_compare(
                                  GEN, TIMER_CHANNEL_1, max_compare / 2U));

            s_oc_events = 0;
            delay_ms(100);
            TEST_ASSERT_UINT32_WITHIN(3U, 100U, s_oc_events);
            TEST_ASSERT_EQUAL(TIMER_CHANNEL_1, s_oc_last_channel);
        }

        void oc_pause_and_resume_gate_the_toggling_output() {
            loopback::reset_fixture();
            TEST_ASSERT_EQUAL(HAL_OK,
                              loopback::generator_toggle_start(
                                  TIMER_CHANNEL_1, loopback::GEN_A, 2'000));

            TEST_ASSERT_TRUE(loopback::count_edges(loopback::GEN_A, 3'000) >=
                             5U); // Toggling: 6 edges in 3ms

            TEST_ASSERT_EQUAL(
                HAL_OK, output_compare_pause_channel(GEN, TIMER_CHANNEL_1));
            delay_us(5); // Let the pin settle to its disabled level
            TEST_ASSERT_EQUAL_UINT32(
                0U, loopback::count_edges(loopback::GEN_A, 3'000));

            TEST_ASSERT_EQUAL(
                HAL_OK, output_compare_resume_channel(GEN, TIMER_CHANNEL_1));
            TEST_ASSERT_TRUE(loopback::count_edges(loopback::GEN_A, 3'000) >=
                             5U);
        }

        // Generated clock across the supported system clocks.
        // The generator's frequency comes from the timer clock arithmetic, so it only agrees with the CPU's
        // cycle counter if timer_get_frequency_hz(...) is right for every APB / TIMPRE combination
        struct sweep_result_t {
            hal_err_t err;
            uint32_t  sysclk_hz;
            uint32_t  pin_hz;
            uint32_t  avg_ticks;
            uint32_t  avg_period_us;
        };

        hal_err_t measure_generated_clock(sweep_result_t* out) {
            constexpr uint32_t EVENTS = 12;

            TRY(loopback::reset_fixture_hal());
            TRY(loopback::generator_toggle_start(
                TIMER_CHANNEL_1, loopback::GEN_A, 2'000));
            out->sysclk_hz = get_system_core_clock();
            out->pin_hz    = loopback::generator_pin_hz();

            reset_log();
            const auto cfg = make_ic_config(loopback::prescaler_for_1us(MEAS),
                                            IC_EDGE_RISING,
                                            0,
                                            {log_cb, nullptr});
            TRY(input_capture_init(MEAS, &cfg));
            TRY(input_capture_start(MEAS));
            if (!loopback::wait_for(
                    []() {
                        return s_log.stored >= EVENTS;
                    },
                    300)) {
                return HAL_ERR_TIMEOUT;
            }
            TRY(input_capture_stop(MEAS));

            uint32_t total_ticks = 0;
            for (uint32_t i = 1; i < EVENTS; i++) {
                total_ticks +=
                    (s_log.value[i] - s_log.value[i - 1]) & UINT16_MAX;
            }
            out->avg_ticks     = total_ticks / (EVENTS - 1U);
            out->avg_period_us = loopback::cycles_to_us32(
                (s_log.cycles[EVENTS - 1U] - s_log.cycles[0]) / (EVENTS - 1U),
                out->sysclk_hz);
            return HAL_OK;
        }

        void
        ic_measured_period_tracks_the_generated_clock_across_system_clocks() {
            const system_clock_t original = get_system_core_clock_type();

            std::array<sweep_result_t, loopback::CLOCK_SWEEP.size()> results{};
            for (size_t i = 0; i < loopback::CLOCK_SWEEP.size(); i++) {
                loopback::switch_clock(
                    loopback::pick_preset(loopback::CLOCK_SWEEP[i], original));
                results[i].err = measure_generated_clock(&results[i]);
                (void)input_capture_deinit(MEAS);
                loopback::generator_stop();
            }
            loopback::switch_clock(
                original); // Back to the original clock before anything gets asserted
            loopback::reset_fixture();

            for (size_t i = 0; i < results.size(); i++) {
                TEST_ASSERT_EQUAL(HAL_OK, results[i].err);
                TEST_ASSERT_EQUAL_UINT32(
                    loopback::preset_mhz(loopback::CLOCK_SWEEP[i], original) *
                        1'000'000U,
                    results[i].sysclk_hz);
                TEST_ASSERT_EQUAL_UINT32(2'000U, results[i].pin_hz);
                TEST_ASSERT_UINT32_WITHIN(1U, 500U, results[i].avg_ticks);
                TEST_ASSERT_UINT32_WITHIN(
                    5U, 500U, results[i].avg_period_us); // 1%
            }
        }

    } // namespace

    void all() {
        LOGI(TAG, "Starting the tests on the extended timer driver");
        UNITY_BEGIN();

        RUN_TEST(ic_init_rejects_invalid_arguments);
        RUN_TEST(ic_init_rejects_a_locked_advanced_timer);
        RUN_TEST(ic_init_programs_every_channel_edge_and_prescaler);
        RUN_TEST(
            ic_init_enables_only_the_interrupts_of_configured_channels_when_a_callback_is_given);
        RUN_TEST(ic_init_programs_the_timer_wide_settings);
        RUN_TEST(ic_init_configures_the_gpio_pin);
        RUN_TEST(ic_timer_prescaler_sets_psc_and_the_reported_tick_rate);
        RUN_TEST(ic_start_and_stop_run_and_freeze_the_counter);
        RUN_TEST(ic_deinit_clears_the_state_and_silences_the_callback);
        RUN_TEST(ic_measures_the_generated_clock_period);
        RUN_TEST(ic_both_edges_capture_the_half_periods_of_a_square_wave);
        RUN_TEST(ic_edge_selection_follows_the_pin_level_at_the_capture);
        RUN_TEST(ic_channels_capture_independently);
        RUN_TEST(ic_digital_filter_rejects_glitches_and_passes_real_pulses);
        RUN_TEST(ic_input_prescaler_divides_the_event_rate);
        RUN_TEST(ic_overcapture_is_reported_once_then_cleared);
        RUN_TEST(ic_get_sample_rejects_invalid_arguments);
        RUN_TEST(filter_code_is_the_smallest_that_covers_the_request);
        RUN_TEST(filter_code_rejects_invalid_arguments);
        RUN_TEST(oc_compare_callback_fires_once_per_period);
        RUN_TEST(oc_pause_and_resume_gate_the_toggling_output);
        RUN_TEST(
            ic_measured_period_tracks_the_generated_clock_across_system_clocks);

        // Leave the timers and clocks the way they were found
        TRY_THEN_FAIL(input_capture_deinit(MEAS));
        loopback::generator_stop();
        TRY_THEN_FAIL(timx_clk_enable(loopback::MEASURED, false));
        TRY_THEN_FAIL(timx_clk_enable(loopback::GENERATOR, false));

        UNITY_END();
        LOGI(TAG, "Done with all tests on the extended timer driver");
    }

} // namespace test::timer_ext
