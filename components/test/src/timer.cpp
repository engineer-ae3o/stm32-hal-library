#include "drivers/timer_types.h"
#include "stm32f411xe.h"
#include "Unity/unity.h"

#include "drivers/timer_internals.h"
#include "drivers/timer.h"
#include "test/timer.hpp"
#include "utils/common.h"
#include "utils/clock.h"
#include "utils/tick.h"
#include "utils/err.h"
#include "utils/log.h"

#include <array>
#include <cstdint>


namespace test::timer {

    namespace {

        constexpr const char* TAG = "Timer_Test";

        // 32 bit, on APB1, supports down counting: used as a general purpose stand in for the tests
        TIM_TypeDef* const TEST_INSTANCE = TIM2;

        // Helpers
        void reset_to_baseline(TIM_TypeDef* handle) {
            TEST_ASSERT_EQUAL(HAL_OK, timer_deinit(handle));
        }

        volatile bool  s_update_done  = false;
        volatile int   s_update_count = 0;
        volatile void* s_last_arg     = nullptr;

        void update_done_cb(void* arg, timer_channel_t) {
            s_update_done = true;
            s_update_count += 1;
            s_last_arg = arg;
        }

        template<typename predicate>
        bool wait_until(predicate pred) {
            volatile uint32_t timeout = TIMEOUT;
            while (!pred() && timeout) {
                timeout -= 1;
            }
            return pred();
        }

        // TESTS
        void clk_enable_rejects_unknown_handles() {
            auto* const bogus = reinterpret_cast<TIM_TypeDef*>(1);
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, timx_clk_enable(bogus, true));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, timx_clk_enable(bogus, false));

            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TEST_INSTANCE, true)); // Leave enabled for the other tests
        }

        void clk_enable_toggles_every_valid_timer_bus_bit() {
            struct combo_t {
                TIM_TypeDef*       handle;
                volatile uint32_t* enable_reg;
                uint32_t           enable_bit;
            };
            const std::array<combo_t, 8> combos = {{
                {.handle = TIM1, .enable_reg = &RCC->APB2ENR, .enable_bit = RCC_APB2ENR_TIM1EN},
                {.handle = TIM2, .enable_reg = &RCC->APB1ENR, .enable_bit = RCC_APB1ENR_TIM2EN},
                {.handle = TIM3, .enable_reg = &RCC->APB1ENR, .enable_bit = RCC_APB1ENR_TIM3EN},
                {.handle = TIM4, .enable_reg = &RCC->APB1ENR, .enable_bit = RCC_APB1ENR_TIM4EN},
                {.handle = TIM5, .enable_reg = &RCC->APB1ENR, .enable_bit = RCC_APB1ENR_TIM5EN},
                {.handle = TIM9, .enable_reg = &RCC->APB2ENR, .enable_bit = RCC_APB2ENR_TIM9EN},
                {.handle = TIM10, .enable_reg = &RCC->APB2ENR, .enable_bit = RCC_APB2ENR_TIM10EN},
                {.handle = TIM11, .enable_reg = &RCC->APB2ENR, .enable_bit = RCC_APB2ENR_TIM11EN},
            }};

            for (const auto& combo : combos) {
                TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(combo.handle, false));
                TEST_ASSERT_FALSE(*combo.enable_reg & combo.enable_bit);
                TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(combo.handle, true));
                TEST_ASSERT_TRUE(*combo.enable_reg & combo.enable_bit);
            }

            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TEST_INSTANCE, true)); // Leave TEST_INSTANCE enabled
        }

        void init_rejects_invalid_arguments() {
            auto* const bogus = reinterpret_cast<TIM_TypeDef*>(1);
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, timer_init(bogus, TIMER_COUNTER_UP, {update_done_cb, nullptr}));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, timer_init(TEST_INSTANCE, TIMER_COUNTER_UP, {}));
        }

        void init_rejects_when_already_running() {
            reset_to_baseline(TEST_INSTANCE);
            TEST_INSTANCE->CR1 |= TIM_CR1_CEN; // Fake a running timer without going through the public API

            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_STATE, timer_init(TEST_INSTANCE, TIMER_COUNTER_UP, {update_done_cb, nullptr}));

            reset_to_baseline(TEST_INSTANCE);
        }

        void init_rejects_down_counting_on_tim9_10_and_11() {
            for (auto* const handle : {TIM9, TIM10, TIM11}) {
                TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(handle, true));
                TEST_ASSERT_EQUAL(HAL_OK, timer_deinit(handle));

                const uint32_t cr1_before = handle->CR1;
                TEST_ASSERT_EQUAL(HAL_ERR_NOT_SUPPORTED, timer_init(handle, TIMER_COUNTER_DOWN, {update_done_cb, nullptr}));
                TEST_ASSERT_EQUAL_UINT32(cr1_before, handle->CR1); // Rejected before CR1 is ever touched

                TEST_ASSERT_EQUAL(HAL_OK, timer_init(handle, TIMER_COUNTER_UP, {update_done_cb, nullptr}));
                TEST_ASSERT_EQUAL(TIMER_COUNTER_UP, (handle->CR1 & TIM_CR1_DIR) >> TIM_CR1_DIR_Pos);

                TEST_ASSERT_EQUAL(HAL_OK, timer_deinit(handle));
                TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(handle, false));
            }
        }

        void init_down_direction_actually_counts_down() {
            reset_to_baseline(TEST_INSTANCE);

            TEST_ASSERT_EQUAL(HAL_OK, timer_init(TEST_INSTANCE, TIMER_COUNTER_DOWN, {update_done_cb, nullptr}));
            TEST_ASSERT_EQUAL(TIMER_COUNTER_DOWN, (TEST_INSTANCE->CR1 & TIM_CR1_DIR));

            TEST_ASSERT_EQUAL(HAL_OK, timer_start_periodic(TEST_INSTANCE, 1'000'000)); // Slow, so CNT is easy to sample
            delay_us(50);
            const uint32_t first_sample = TEST_INSTANCE->CNT;
            delay_us(50);
            const uint32_t second_sample = TEST_INSTANCE->CNT;

            TEST_ASSERT_TRUE(first_sample > second_sample);

            TEST_ASSERT_EQUAL(HAL_OK, timer_pause(TEST_INSTANCE));
            reset_to_baseline(TEST_INSTANCE);
        }

        void deinit_rejects_unknown_handle() {
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, timer_deinit(reinterpret_cast<TIM_TypeDef*>(1)));
        }

        void deinit_clears_all_register_state() {
            reset_to_baseline(TEST_INSTANCE);

            // Set some random bits and load some more random values to the timer's registers
            TEST_INSTANCE->CR1 |= (TIM_CR1_ARPE | TIM_CR1_URS | TIM_CR1_OPM | TIM_CR1_DIR);
            TEST_INSTANCE->CR2 |= TIM_CR2_CCDS;
            TEST_INSTANCE->DIER |= (TIM_DIER_UIE | TIM_DIER_CC1IE);
            TEST_INSTANCE->CNT  = 1234;
            TEST_INSTANCE->PSC  = 10;
            TEST_INSTANCE->ARR  = 5000;
            TEST_INSTANCE->CCR1 = 11;
            TEST_INSTANCE->CCR2 = 22;
            TEST_INSTANCE->CCR3 = 33;
            TEST_INSTANCE->CCR4 = 44;

            TEST_ASSERT_EQUAL(HAL_OK, timer_deinit(TEST_INSTANCE));

            TEST_ASSERT_EQUAL_UINT32(0,
                                     TEST_INSTANCE->CR1 & (TIM_CR1_CEN | TIM_CR1_UDIS | TIM_CR1_URS | TIM_CR1_OPM | TIM_CR1_DIR | TIM_CR1_CMS |
                                                           TIM_CR1_ARPE | TIM_CR1_CKD));
            TEST_ASSERT_EQUAL_UINT32(0,
                                     TEST_INSTANCE->CR2 &
                                         (TIM_CR2_CCPC | TIM_CR2_CCUS | TIM_CR2_CCDS | TIM_CR2_MMS | TIM_CR2_TI1S | TIM_CR2_OIS1 | TIM_CR2_OIS1N |
                                          TIM_CR2_OIS2 | TIM_CR2_OIS2N | TIM_CR2_OIS3 | TIM_CR2_OIS3N | TIM_CR2_OIS4));
            TEST_ASSERT_EQUAL_UINT32(0,
                                     TEST_INSTANCE->DIER & (TIM_DIER_UIE | TIM_DIER_CC1IE | TIM_DIER_CC2IE | TIM_DIER_CC3IE | TIM_DIER_CC4IE |
                                                            TIM_DIER_COMIE | TIM_DIER_TIE | TIM_DIER_BIE | TIM_DIER_UDE | TIM_DIER_CC1DE |
                                                            TIM_DIER_CC2DE | TIM_DIER_CC3DE | TIM_DIER_CC4DE | TIM_DIER_COMDE | TIM_DIER_TDE));
            TEST_ASSERT_EQUAL_UINT32(0, TEST_INSTANCE->CNT);
            TEST_ASSERT_EQUAL_UINT32(0, TEST_INSTANCE->PSC);
            TEST_ASSERT_EQUAL_UINT32(0, TEST_INSTANCE->ARR);
            TEST_ASSERT_EQUAL_UINT32(0, TEST_INSTANCE->CCR1);
            TEST_ASSERT_EQUAL_UINT32(0, TEST_INSTANCE->CCR2);
            TEST_ASSERT_EQUAL_UINT32(0, TEST_INSTANCE->CCR3);
            TEST_ASSERT_EQUAL_UINT32(0, TEST_INSTANCE->CCR4);

            // This test should only pass if TEST_INSTANCE is not TIM1 or TIM10
            TEST_ASSERT_FALSE(NVIC->ISER[TIM2_IRQn >> 5] & (1UL << (TIM2_IRQn & 0x1FU)));
        }

        void deinit_stops_a_timer_that_is_currently_running() {
            reset_to_baseline(TEST_INSTANCE);
            TEST_ASSERT_EQUAL(HAL_OK, timer_init(TEST_INSTANCE, TIMER_COUNTER_UP, {update_done_cb, nullptr}));
            TEST_ASSERT_FALSE(TEST_INSTANCE->CR1 & TIM_CR1_CEN);
            TEST_ASSERT_EQUAL(HAL_OK, timer_start_periodic(TEST_INSTANCE, 1'000'000));
            TEST_ASSERT_TRUE(TEST_INSTANCE->CR1 & TIM_CR1_CEN);

            // deinit has no CEN guard, unlike init/start/resume: it must be able to stop a live timer directly
            TEST_ASSERT_EQUAL(HAL_OK, timer_deinit(TEST_INSTANCE));
            TEST_ASSERT_FALSE(TEST_INSTANCE->CR1 & TIM_CR1_CEN);
        }

        void deinit_clears_rcr_on_tim1_only() {
            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM1, true));
            TEST_ASSERT_EQUAL(HAL_OK, timer_deinit(TIM1));

            TIM1->RCR = 0x55;
            TEST_ASSERT_EQUAL(HAL_OK, timer_deinit(TIM1));
            TEST_ASSERT_EQUAL_UINT32(0, TIM1->RCR & TIM_RCR_REP);

            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM1, false));
        }

        void deinit_on_tim10_does_not_touch_the_shared_irq_line_that_tim1_uses() {
            // TIM1 and TIM10 share one NVIC line (TIM1_UP_TIM10_IRQn).
            // Deinitializing either one must leave that line alone
            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM1, true));
            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM10, true));

            s_update_done = false;
            TEST_ASSERT_EQUAL(HAL_OK, timer_init(TIM1, TIMER_COUNTER_UP, {update_done_cb, nullptr}));
            TEST_ASSERT_EQUAL(HAL_OK, timer_start_oneshot(TIM1, 1'000));
            TEST_ASSERT_TRUE(NVIC->ISER[TIM1_UP_TIM10_IRQn >> 5] & (1UL << (TIM1_UP_TIM10_IRQn & 0x1FU)));

            // Deiniting TIM10 must NOT rip out the line TIM1 still depends on
            TEST_ASSERT_EQUAL(HAL_OK, timer_deinit(TIM10));
            TEST_ASSERT_TRUE(NVIC->ISER[TIM1_UP_TIM10_IRQn >> 5] & (1UL << (TIM1_UP_TIM10_IRQn & 0x1FU)));

            // And TIM1's still pending oneshot actually completes and fires its callback
            TEST_ASSERT_TRUE_MESSAGE(wait_until([]() {
                                         return s_update_done;
                                     }),
                                     "TIM1 oneshot never fired after TIM10 was deinited");
            delay_ms(2);
            TEST_ASSERT_FALSE(TIM1->CR1 & TIM_CR1_CEN);

            TEST_ASSERT_EQUAL(HAL_OK, timer_deinit(TIM1));
            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM1, false));
            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM10, false));
        }

        void start_oneshot_rejects_invalid_arguments() {
            auto* const bogus = reinterpret_cast<TIM_TypeDef*>(1);
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, timer_start_oneshot(bogus, 1'000));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, timer_start_oneshot(TEST_INSTANCE, 0));
        }

        void start_oneshot_rejects_when_already_running() {
            reset_to_baseline(TEST_INSTANCE);
            TEST_INSTANCE->CR1 |= TIM_CR1_CEN; // Fake a running timer without going through the public API

            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_STATE, timer_start_oneshot(TEST_INSTANCE, 1'000));

            reset_to_baseline(TEST_INSTANCE);
        }

        void start_oneshot_rejects_a_timeout_that_overflows_the_prescaler_range() {
            // TIM3 is a 16 bit timer: PSC and ARR are both capped at UINT16_MAX, so their
            // product can't come close to covering a UINT32_MAX microsecond timeout at any bus clock.
            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM3, true));
            TEST_ASSERT_EQUAL(HAL_OK, timer_deinit(TIM3));

            TEST_ASSERT_EQUAL(HAL_ERR_NOT_SUPPORTED, timer_start_oneshot(TIM3, UINT32_MAX));
            TEST_ASSERT_FALSE(TIM3->CR1 & TIM_CR1_CEN);

            TEST_ASSERT_EQUAL(HAL_OK, timer_deinit(TIM3));
            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM3, false));
        }

        void start_oneshot_completes_stops_itself_and_invokes_the_callback() {
            reset_to_baseline(TEST_INSTANCE);

            s_update_done = false;
            s_last_arg    = nullptr;
            int marker    = 42;
            TEST_ASSERT_EQUAL(HAL_OK, timer_init(TEST_INSTANCE, TIMER_COUNTER_UP, {update_done_cb, &marker}));
            TEST_ASSERT_EQUAL(HAL_OK, timer_start_oneshot(TEST_INSTANCE, 1'000));

            constexpr uint32_t expected_bits = TIM_CR1_CEN | TIM_CR1_OPM | TIM_CR1_ARPE | TIM_CR1_URS;
            TEST_ASSERT_EQUAL_UINT32(expected_bits, TEST_INSTANCE->CR1 & expected_bits);

            TEST_ASSERT_TRUE_MESSAGE(wait_until([]() {
                                         return s_update_done;
                                     }),
                                     "Oneshot timer never fired its callback");

            TEST_ASSERT_FALSE(TEST_INSTANCE->CR1 & TIM_CR1_CEN); // The timer hardware auto clears CEN in OPM on a UEV
            TEST_ASSERT_FALSE(TEST_INSTANCE->SR & TIM_SR_UIF);   // The ISR clears the UEV interrupt flag
            TEST_ASSERT_EQUAL_PTR(&marker, s_last_arg);

            reset_to_baseline(TEST_INSTANCE);
        }

        void start_oneshot_uses_the_shared_irq_for_tim1() {
            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM1, true));

            s_update_done = false;
            TEST_ASSERT_EQUAL(HAL_OK, timer_init(TIM1, TIMER_COUNTER_UP, {update_done_cb, nullptr}));
            TEST_ASSERT_EQUAL(HAL_OK, timer_start_oneshot(TIM1, 1'000));

            // TIM1's update event is dispatched through TIM1_UP_TIM10_IRQn, not TIM1_CC_IRQn
            TEST_ASSERT_TRUE(NVIC->ISER[TIM1_UP_TIM10_IRQn >> 5] & (1UL << (TIM1_UP_TIM10_IRQn & 0x1FU)));

            TEST_ASSERT_TRUE_MESSAGE(wait_until([]() {
                                         return s_update_done;
                                     }),
                                     "TIM1 oneshot never fired via the shared IRQ");

            TEST_ASSERT_EQUAL(HAL_OK, timer_deinit(TIM1));
            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM1, false));
        }

        void start_periodic_rejects_invalid_arguments() {
            auto* const bogus = reinterpret_cast<TIM_TypeDef*>(1);
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, timer_start_periodic(bogus, 1'000));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, timer_start_periodic(TEST_INSTANCE, 0));
        }

        void start_periodic_rejects_when_already_running() {
            reset_to_baseline(TEST_INSTANCE);
            TEST_INSTANCE->CR1 |= TIM_CR1_CEN; // Fake a running timer without going through the public API

            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_STATE, timer_start_periodic(TEST_INSTANCE, 1'000));

            reset_to_baseline(TEST_INSTANCE);
        }

        void start_periodic_repeats_until_paused() {
            reset_to_baseline(TEST_INSTANCE);

            s_update_count = 0;
            TEST_ASSERT_EQUAL(HAL_OK, timer_init(TEST_INSTANCE, TIMER_COUNTER_UP, {update_done_cb, nullptr}));
            TEST_ASSERT_EQUAL(HAL_OK, timer_start_periodic(TEST_INSTANCE, 200)); // Short period: several fire quickly

            TEST_ASSERT_FALSE(TEST_INSTANCE->CR1 & TIM_CR1_OPM);
            constexpr uint32_t expected_bits = TIM_CR1_CEN | TIM_CR1_ARPE | TIM_CR1_URS;
            TEST_ASSERT_EQUAL_UINT32(expected_bits, TEST_INSTANCE->CR1 & expected_bits);

            TEST_ASSERT_TRUE_MESSAGE(wait_until([]() {
                                         return s_update_count >= 100;
                                     }),
                                     "Periodic timer did not fire repeatedly");
            TEST_ASSERT_TRUE(TEST_INSTANCE->CR1 & TIM_CR1_CEN); // Should still be running, unlike OPM

            TEST_ASSERT_EQUAL(HAL_OK, timer_pause(TEST_INSTANCE));
            reset_to_baseline(TEST_INSTANCE);
        }

        void pause_rejects_null_and_a_timer_that_is_not_running() {
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, timer_pause(nullptr));

            reset_to_baseline(TEST_INSTANCE);
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_STATE, timer_pause(TEST_INSTANCE));
        }

        void resume_rejects_null_and_a_timer_that_is_already_running() {
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, timer_resume(nullptr));

            reset_to_baseline(TEST_INSTANCE);
            TEST_ASSERT_EQUAL(HAL_OK, timer_init(TEST_INSTANCE, TIMER_COUNTER_UP, {update_done_cb, nullptr}));
            TEST_ASSERT_EQUAL(HAL_OK, timer_start_periodic(TEST_INSTANCE, 1'000'000));

            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_STATE, timer_resume(TEST_INSTANCE));

            reset_to_baseline(TEST_INSTANCE);
        }

        void pause_and_resume_halt_and_continue_the_counter() {
            reset_to_baseline(TEST_INSTANCE);
            TEST_ASSERT_EQUAL(HAL_OK, timer_init(TEST_INSTANCE, TIMER_COUNTER_UP, {update_done_cb, nullptr}));
            TEST_ASSERT_EQUAL(HAL_OK, timer_start_periodic(TEST_INSTANCE, 1'000'000)); // Long enough; only CNT progression matters

            delay_us(50);
            TEST_ASSERT_EQUAL(HAL_OK, timer_pause(TEST_INSTANCE));
            const uint32_t held_cnt = TEST_INSTANCE->CNT;
            delay_us(200);
            TEST_ASSERT_EQUAL_UINT32(held_cnt, TEST_INSTANCE->CNT); // Should be frozen while paused

            TEST_ASSERT_EQUAL(HAL_OK, timer_resume(TEST_INSTANCE));
            delay_us(50);
            TEST_ASSERT_TRUE(TEST_INSTANCE->CNT > held_cnt); // Should be advancing again

            reset_to_baseline(TEST_INSTANCE);
        }

        void restart_reapplies_periodic_mode_and_keeps_repeating() {
            reset_to_baseline(TEST_INSTANCE);

            s_update_count = 0;
            TEST_ASSERT_EQUAL(HAL_OK, timer_init(TEST_INSTANCE, TIMER_COUNTER_UP, {update_done_cb, nullptr}));
            TEST_ASSERT_EQUAL(HAL_OK, timer_start_periodic(TEST_INSTANCE, 1'000));

            delay_us(50);
            TEST_ASSERT_EQUAL(HAL_OK, timer_restart(TEST_INSTANCE, 200)); // Much shorter: observe several cycles quickly
            TEST_ASSERT_FALSE(TEST_INSTANCE->CR1 & TIM_CR1_OPM);

            TEST_ASSERT_TRUE_MESSAGE(wait_until([]() {
                                         return s_update_count >= 100;
                                     }),
                                     "Restarted periodic timer did not keep repeating");
            TEST_ASSERT_TRUE(TEST_INSTANCE->CR1 & TIM_CR1_CEN);

            reset_to_baseline(TEST_INSTANCE);
        }

        void is_timer_on_apb1_matches_the_bus_map() {
            for (auto* const handle : {TIM2, TIM3, TIM4, TIM5}) {
                TEST_ASSERT_TRUE(is_timer_on_apb1(handle));
            }
            for (auto* const handle : {TIM1, TIM9, TIM10, TIM11}) {
                TEST_ASSERT_FALSE(is_timer_on_apb1(handle));
            }
        }

        void is_timer_32_bits_matches_the_counter_width_map() {
            for (auto* const handle : {TIM2, TIM5}) {
                TEST_ASSERT_TRUE(is_timer_32_bits(handle));
            }
            for (auto* const handle : {TIM1, TIM3, TIM4, TIM9, TIM10, TIM11}) {
                TEST_ASSERT_FALSE(is_timer_32_bits(handle));
            }
        }

        void get_frequency_hz_matches_the_apb_doubling_rules() {
            const uint32_t sysclk = get_system_core_clock();
            const uint32_t apb1   = get_apb1_core_clock();
            uint32_t       freq   = 0;

            const uint32_t timpre_before = RCC->DCKCFGR & RCC_DCKCFGR_TIMPRE;

            RCC->DCKCFGR &= ~RCC_DCKCFGR_TIMPRE;          // Standard mode
            freq = timer_get_frequency_hz(TEST_INSTANCE); // TIM2 is on APB1

            const uint32_t expected_std_f = (sysclk == apb1) ? apb1 : (apb1 * 2);
            TEST_ASSERT_EQUAL_UINT32(expected_std_f, freq);

            RCC->DCKCFGR |= RCC_DCKCFGR_TIMPRE; // High frequency mode
            freq = timer_get_frequency_hz(TEST_INSTANCE);

            const uint32_t expected_hf = ((sysclk == apb1) || ((sysclk / apb1) == 2)) ? sysclk : (apb1 * 4);
            TEST_ASSERT_EQUAL_UINT32(expected_hf, freq);

            if (timpre_before) {
                RCC->DCKCFGR |= RCC_DCKCFGR_TIMPRE;
            } else {
                RCC->DCKCFGR &= ~RCC_DCKCFGR_TIMPRE;
            }
        }

        void set_arr_and_psc_rejects_a_timeout_that_overflows_the_supported_range() {
            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM3, true));
            TEST_ASSERT_EQUAL(HAL_ERR_NOT_SUPPORTED, timer_set_arr_and_psc(TIM3, UINT32_MAX));
            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM3, false));
        }

        void set_arr_and_psc_computes_values_that_reconstruct_the_requested_timeout() {
            reset_to_baseline(TEST_INSTANCE);

            constexpr uint32_t REQUESTED_TIMEOUT_US = 5000;
            TEST_ASSERT_EQUAL(HAL_OK, timer_set_arr_and_psc(TEST_INSTANCE, REQUESTED_TIMEOUT_US));

            const uint32_t freq_hz = timer_get_frequency_hz(TEST_INSTANCE);

            const uint64_t psc_plus_1          = (uint64_t)TEST_INSTANCE->PSC + 1;
            const uint64_t arr_plus_1          = (uint64_t)TEST_INSTANCE->ARR + 1;
            const uint64_t achieved_timeout_us = (psc_plus_1 * arr_plus_1 * 1'000'000ULL) / freq_hz;

            constexpr uint64_t tolerance = REQUESTED_TIMEOUT_US / 100;
            TEST_ASSERT_TRUE(achieved_timeout_us >= (REQUESTED_TIMEOUT_US - tolerance));
            TEST_ASSERT_TRUE(achieved_timeout_us <= (REQUESTED_TIMEOUT_US + tolerance));

            reset_to_baseline(TEST_INSTANCE);
        }

    } // namespace

    void all() {
        LOGI(TAG, "Starting the tests on the timer driver");
        UNITY_BEGIN();

        RUN_TEST(clk_enable_rejects_unknown_handles);
        RUN_TEST(clk_enable_toggles_every_valid_timer_bus_bit);
        RUN_TEST(init_rejects_invalid_arguments);
        RUN_TEST(init_rejects_when_already_running);
        RUN_TEST(init_rejects_down_counting_on_tim9_10_and_11);
        RUN_TEST(init_down_direction_actually_counts_down);
        RUN_TEST(deinit_rejects_unknown_handle);
        RUN_TEST(deinit_clears_all_register_state);
        RUN_TEST(deinit_stops_a_timer_that_is_currently_running);
        RUN_TEST(deinit_clears_rcr_on_tim1_only);
        RUN_TEST(deinit_on_tim10_does_not_touch_the_shared_irq_line_that_tim1_uses);
        RUN_TEST(start_oneshot_rejects_invalid_arguments);
        RUN_TEST(start_oneshot_rejects_when_already_running);
        RUN_TEST(start_oneshot_rejects_a_timeout_that_overflows_the_prescaler_range);
        RUN_TEST(start_oneshot_completes_stops_itself_and_invokes_the_callback);
        RUN_TEST(start_oneshot_uses_the_shared_irq_for_tim1);
        RUN_TEST(start_periodic_rejects_invalid_arguments);
        RUN_TEST(start_periodic_rejects_when_already_running);
        RUN_TEST(start_periodic_repeats_until_paused);
        RUN_TEST(pause_rejects_null_and_a_timer_that_is_not_running);
        RUN_TEST(resume_rejects_null_and_a_timer_that_is_already_running);
        RUN_TEST(pause_and_resume_halt_and_continue_the_counter);
        RUN_TEST(restart_reapplies_periodic_mode_and_keeps_repeating);
        RUN_TEST(is_timer_on_apb1_matches_the_bus_map);
        RUN_TEST(is_timer_32_bits_matches_the_counter_width_map);
        RUN_TEST(get_frequency_hz_matches_the_apb_doubling_rules);
        RUN_TEST(set_arr_and_psc_rejects_a_timeout_that_overflows_the_supported_range);
        RUN_TEST(set_arr_and_psc_computes_values_that_reconstruct_the_requested_timeout);

        UNITY_END();
        LOGI(TAG, "Done with all tests on the timer driver");
    }

} // namespace test::timer
