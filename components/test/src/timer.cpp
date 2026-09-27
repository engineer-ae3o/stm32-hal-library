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
#include <utility>


namespace test::timer {

    namespace {

        constexpr const char* TAG = "Timer_Test";

        // 32-bit, APB1, supports down counting: used as a general purpose stand in
        TIM_TypeDef* const TEST_INSTANCE = TIM2;

        // Helpers
        void reset_to_baseline(TIM_TypeDef* handle) {
            TEST_ASSERT_EQUAL(HAL_OK, timer_deinit(handle));
        }

        volatile bool  s_update_done  = false;
        volatile int   s_update_count = 0;
        volatile void* s_last_arg     = nullptr;

        void update_done_cb(void* arg) {
            (void)arg;
            s_update_done = true;
            s_update_count += 1;
        }

        void arg_capture_cb(void* arg) {
            s_last_arg    = arg;
            s_update_done = true;
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
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, timer_clock_enable(bogus, true));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, timer_clock_enable(bogus, false));

            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM2, true)); // leave enabled: TEST_INSTANCE for the rest of the suite
        }

        void clk_enable_toggles_every_valid_timer_bus_bit() {
            struct combo_t {
                TIM_TypeDef*       handle;
                volatile uint32_t* enr;
                uint32_t           bit;
            };
            const std::array<combo_t, 8> combos = {{
                {TIM1, &RCC->APB2ENR, RCC_APB2ENR_TIM1EN},
                {TIM2, &RCC->APB1ENR, RCC_APB1ENR_TIM2EN},
                {TIM3, &RCC->APB1ENR, RCC_APB1ENR_TIM3EN},
                {TIM4, &RCC->APB1ENR, RCC_APB1ENR_TIM4EN},
                {TIM5, &RCC->APB1ENR, RCC_APB1ENR_TIM5EN},
                {TIM9, &RCC->APB2ENR, RCC_APB2ENR_TIM9EN},
                {TIM10, &RCC->APB2ENR, RCC_APB2ENR_TIM10EN},
                {TIM11, &RCC->APB2ENR, RCC_APB2ENR_TIM11EN},
            }};

            for (const auto& combo : combos) {
                TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(combo.handle, false));
                TEST_ASSERT_FALSE(*combo.enr & combo.bit);
                TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(combo.handle, true));
                TEST_ASSERT_TRUE(*combo.enr & combo.bit);
            }

            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM2, true)); // leave TEST_INSTANCE enabled
        }

        void init_rejects_invalid_arguments() {
            auto* const bogus = reinterpret_cast<TIM_TypeDef*>(1);
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, timer_init(bogus, TIMER_COUNTER_UP, update_done_cb, nullptr));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, timer_init(TEST_INSTANCE, TIMER_COUNTER_UP, nullptr, nullptr));
        }

        void init_rejects_when_already_running() {
            reset_to_baseline(TEST_INSTANCE);
            TEST_INSTANCE->CR1 |= TIM_CR1_CEN; // fake a running timer without going through the public API

            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_STATE, timer_init(TEST_INSTANCE, TIMER_COUNTER_UP, update_done_cb, nullptr));

            TEST_INSTANCE->CR1 &= ~TIM_CR1_CEN;
            reset_to_baseline(TEST_INSTANCE);
        }

        void init_rejects_down_counting_on_tim9_10_and_11() {
            for (auto* const handle : {TIM9, TIM10, TIM11}) {
                TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(handle, true));
                TEST_ASSERT_EQUAL(HAL_OK, timer_deinit(handle));

                const uint32_t cr1_before = handle->CR1;
                TEST_ASSERT_EQUAL(HAL_ERR_NOT_SUPPORTED, timer_init(handle, TIMER_COUNTER_DOWN, update_done_cb, nullptr));
                TEST_ASSERT_EQUAL_UINT32(cr1_before, handle->CR1); // rejected before CR1 is ever touched

                TEST_ASSERT_EQUAL(HAL_OK, timer_init(handle, TIMER_COUNTER_UP, update_done_cb, nullptr));
                TEST_ASSERT_FALSE(handle->CR1 & TIM_CR1_DIR);

                TEST_ASSERT_EQUAL(HAL_OK, timer_deinit(handle));
                TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(handle, false));
            }
        }

        void init_sets_direction_bit_and_preserves_other_cr1_bits() {
            reset_to_baseline(TEST_INSTANCE);
            TEST_INSTANCE->CR1 |= TIM_CR1_UDIS; // unrelated bit timer_init must not touch

            TEST_ASSERT_EQUAL(HAL_OK, timer_init(TEST_INSTANCE, TIMER_COUNTER_UP, update_done_cb, nullptr));
            TEST_ASSERT_FALSE(TEST_INSTANCE->CR1 & TIM_CR1_DIR);
            TEST_ASSERT_TRUE(TEST_INSTANCE->CR1 & TIM_CR1_UDIS);

            TEST_ASSERT_EQUAL(HAL_OK, timer_init(TEST_INSTANCE, TIMER_COUNTER_DOWN, update_done_cb, nullptr));
            TEST_ASSERT_TRUE(TEST_INSTANCE->CR1 & TIM_CR1_DIR);
            TEST_ASSERT_TRUE(TEST_INSTANCE->CR1 & TIM_CR1_UDIS); // still preserved

            reset_to_baseline(TEST_INSTANCE);
        }

        void init_down_direction_actually_counts_down() {
            reset_to_baseline(TEST_INSTANCE);

            TEST_ASSERT_EQUAL(HAL_OK, timer_init(TEST_INSTANCE, TIMER_COUNTER_DOWN, update_done_cb, nullptr));
            TEST_ASSERT_TRUE(TEST_INSTANCE->CR1 & TIM_CR1_DIR);

            TEST_ASSERT_EQUAL(HAL_OK, timer_start_periodic(TEST_INSTANCE, 1'000'000)); // slow, so CNT is easy to sample
            delay_us(50);
            const uint32_t first_sample = TEST_INSTANCE->CNT;
            delay_us(50);
            const uint32_t second_sample = TEST_INSTANCE->CNT;

            TEST_ASSERT_TRUE(second_sample < first_sample);

            TEST_ASSERT_EQUAL(HAL_OK, timer_pause(TEST_INSTANCE));
            reset_to_baseline(TEST_INSTANCE);
        }

        void deinit_rejects_unknown_handle() {
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, timer_deinit(reinterpret_cast<TIM_TypeDef*>(1)));
        }

        void deinit_clears_all_register_state() {
            reset_to_baseline(TEST_INSTANCE);

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

            TEST_ASSERT_FALSE(NVIC->ISER[TIM2_IRQn >> 5] & (1UL << (TIM2_IRQn & 0x1FU)));
        }

        void deinit_clears_the_registered_callback() {
            reset_to_baseline(TEST_INSTANCE);

            s_update_done = false;
            TEST_ASSERT_EQUAL(HAL_OK, timer_init(TEST_INSTANCE, TIMER_COUNTER_UP, update_done_cb, nullptr));
            TEST_ASSERT_EQUAL(HAL_OK, timer_deinit(TEST_INSTANCE));

            // The callback was cleared by deinit; starting a oneshot now must complete but never call it
            TEST_ASSERT_EQUAL(HAL_OK, timer_start_oneshot(TEST_INSTANCE, 1000));
            TEST_ASSERT_TRUE_MESSAGE(wait_until([]() {
                                         return !(TEST_INSTANCE->CR1 & TIM_CR1_CEN);
                                     }),
                                     "Oneshot timer never completed");
            TEST_ASSERT_FALSE(s_update_done);

            reset_to_baseline(TEST_INSTANCE);
        }

        void deinit_stops_a_timer_that_is_currently_running() {
            reset_to_baseline(TEST_INSTANCE);
            TEST_ASSERT_EQUAL(HAL_OK, timer_init(TEST_INSTANCE, TIMER_COUNTER_UP, update_done_cb, nullptr));
            TEST_ASSERT_EQUAL(HAL_OK, timer_start_periodic(TEST_INSTANCE, 1'000'000));
            TEST_ASSERT_TRUE(TEST_INSTANCE->CR1 & TIM_CR1_CEN);

            // deinit has no CEN guard, unlike init/start/resume: it must be able to stop a live timer directly
            TEST_ASSERT_EQUAL(HAL_OK, timer_deinit(TEST_INSTANCE));
            TEST_ASSERT_FALSE(TEST_INSTANCE->CR1 & TIM_CR1_CEN);
        }

        void deinit_clears_rcr_on_tim1_only() {
            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM1, true));
            TEST_ASSERT_EQUAL(HAL_OK, timer_deinit(TIM1));

            TIM1->RCR = 0x55;
            TEST_ASSERT_EQUAL(HAL_OK, timer_deinit(TIM1));
            TEST_ASSERT_EQUAL_UINT32(0, TIM1->RCR & TIM_RCR_REP);

            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM1, false));
        }

        void deinit_on_tim10_does_not_touch_the_shared_irq_line_that_tim1_uses() {
            // TIM1 and TIM10 share one NVIC line (TIM1_UP_TIM10_IRQn). Deiniting either one
            // must leave that line alone -- disabling it unconditionally would silently kill
            // the other timer's interrupt if it's still running. This is a deliberate
            // limitation of the driver, not an oversight: verify it stays that way.
            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM1, true));
            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM10, true));
            TEST_ASSERT_EQUAL(HAL_OK, timer_deinit(TIM1));
            TEST_ASSERT_EQUAL(HAL_OK, timer_deinit(TIM10));

            s_update_done = false;
            TEST_ASSERT_EQUAL(HAL_OK, timer_init(TIM1, TIMER_COUNTER_UP, update_done_cb, nullptr));
            TEST_ASSERT_EQUAL(HAL_OK, timer_start_oneshot(TIM1, 1000));
            TEST_ASSERT_TRUE(NVIC->ISER[TIM1_UP_TIM10_IRQn >> 5] & (1UL << (TIM1_UP_TIM10_IRQn & 0x1FU)));

            // Deiniting TIM10 must NOT rip out the line TIM1 still depends on
            TEST_ASSERT_EQUAL(HAL_OK, timer_deinit(TIM10));
            TEST_ASSERT_TRUE(NVIC->ISER[TIM1_UP_TIM10_IRQn >> 5] & (1UL << (TIM1_UP_TIM10_IRQn & 0x1FU)));

            // And TIM1's still-pending oneshot actually completes and fires its callback
            TEST_ASSERT_TRUE_MESSAGE(wait_until([]() {
                                         return s_update_done;
                                     }),
                                     "TIM1 oneshot never fired after TIM10 was deinited");
            TEST_ASSERT_FALSE(TIM1->CR1 & TIM_CR1_CEN);

            TEST_ASSERT_EQUAL(HAL_OK, timer_deinit(TIM1));
            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM1, false));
            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM10, false));
        }

        void start_oneshot_rejects_invalid_arguments() {
            auto* const bogus = reinterpret_cast<TIM_TypeDef*>(1);
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, timer_start_oneshot(bogus, 1000));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, timer_start_oneshot(TEST_INSTANCE, 0));
        }

        void start_oneshot_rejects_when_already_running() {
            reset_to_baseline(TEST_INSTANCE);
            TEST_INSTANCE->CR1 |= TIM_CR1_CEN;

            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_STATE, timer_start_oneshot(TEST_INSTANCE, 1000));

            TEST_INSTANCE->CR1 &= ~TIM_CR1_CEN;
            reset_to_baseline(TEST_INSTANCE);
        }

        void start_oneshot_rejects_a_timeout_that_overflows_the_prescaler_range() {
            // TIM3 is a 16-bit timer: PSC and ARR are both capped at UINT16_MAX, so their
            // product can't come close to covering a UINT32_MAX-microsecond timeout at any bus clock.
            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM3, true));
            TEST_ASSERT_EQUAL(HAL_OK, timer_deinit(TIM3));

            TEST_ASSERT_EQUAL(HAL_ERR_NOT_SUPPORTED, timer_start_oneshot(TIM3, UINT32_MAX));
            TEST_ASSERT_FALSE(TIM3->CR1 & TIM_CR1_CEN);

            TEST_ASSERT_EQUAL(HAL_OK, timer_deinit(TIM3));
            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM3, false));
        }

        void start_oneshot_completes_stops_itself_and_invokes_the_callback() {
            reset_to_baseline(TEST_INSTANCE);

            s_update_done = false;
            s_last_arg    = nullptr;
            int marker    = 42;
            TEST_ASSERT_EQUAL(HAL_OK, timer_init(TEST_INSTANCE, TIMER_COUNTER_UP, arg_capture_cb, &marker));
            TEST_ASSERT_EQUAL(HAL_OK, timer_start_oneshot(TEST_INSTANCE, 1000));

            constexpr uint32_t expected_bits = TIM_CR1_CEN | TIM_CR1_OPM | TIM_CR1_ARPE | TIM_CR1_URS;
            TEST_ASSERT_EQUAL_UINT32(expected_bits, TEST_INSTANCE->CR1 & expected_bits);

            TEST_ASSERT_TRUE_MESSAGE(wait_until([]() {
                                         return s_update_done;
                                     }),
                                     "Oneshot timer never fired its callback");

            TEST_ASSERT_FALSE(TEST_INSTANCE->CR1 & TIM_CR1_CEN); // hardware auto-clears CEN in OPM
            TEST_ASSERT_FALSE(TEST_INSTANCE->SR & TIM_SR_UIF);   // the ISR clears the flag
            TEST_ASSERT_EQUAL_PTR(&marker, s_last_arg);

            reset_to_baseline(TEST_INSTANCE);
        }

        void start_oneshot_clears_the_registered_callback_after_it_fires() {
            reset_to_baseline(TEST_INSTANCE);

            s_update_done = false;
            TEST_ASSERT_EQUAL(HAL_OK, timer_init(TEST_INSTANCE, TIMER_COUNTER_UP, update_done_cb, nullptr));
            TEST_ASSERT_EQUAL(HAL_OK, timer_start_oneshot(TEST_INSTANCE, 1000));
            TEST_ASSERT_TRUE_MESSAGE(wait_until([]() {
                                         return s_update_done;
                                     }),
                                     "First oneshot never fired");

            // OPM clears the registered callback the moment it fires. Starting again without
            // re-initing must still run to completion, but the (now-cleared) callback must not fire.
            s_update_done = false;
            TEST_ASSERT_EQUAL(HAL_OK, timer_start_oneshot(TEST_INSTANCE, 1000));
            TEST_ASSERT_TRUE_MESSAGE(wait_until([]() {
                                         return !(TEST_INSTANCE->CR1 & TIM_CR1_CEN);
                                     }),
                                     "Second oneshot never completed");
            TEST_ASSERT_FALSE(s_update_done);

            reset_to_baseline(TEST_INSTANCE);
        }

        void start_oneshot_uses_the_shared_irq_for_tim1() {
            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM1, true));
            TEST_ASSERT_EQUAL(HAL_OK, timer_deinit(TIM1));

            s_update_done = false;
            TEST_ASSERT_EQUAL(HAL_OK, timer_init(TIM1, TIMER_COUNTER_UP, update_done_cb, nullptr));
            TEST_ASSERT_EQUAL(HAL_OK, timer_start_oneshot(TIM1, 1000));

            // TIM1's update event is dispatched through TIM1_UP_TIM10_IRQn, not TIM1_CC_IRQn
            TEST_ASSERT_TRUE(NVIC->ISER[TIM1_UP_TIM10_IRQn >> 5] & (1UL << (TIM1_UP_TIM10_IRQn & 0x1FU)));

            TEST_ASSERT_TRUE_MESSAGE(wait_until([]() {
                                         return s_update_done;
                                     }),
                                     "TIM1 oneshot never fired via the shared IRQ");

            TEST_ASSERT_EQUAL(HAL_OK, timer_deinit(TIM1));
            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM1, false));
        }

        void start_periodic_rejects_invalid_arguments() {
            auto* const bogus = reinterpret_cast<TIM_TypeDef*>(1);
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, timer_start_periodic(bogus, 1000));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, timer_start_periodic(TEST_INSTANCE, 0));
        }

        void start_periodic_rejects_when_already_running() {
            reset_to_baseline(TEST_INSTANCE);
            TEST_INSTANCE->CR1 |= TIM_CR1_CEN;

            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_STATE, timer_start_periodic(TEST_INSTANCE, 1000));

            TEST_INSTANCE->CR1 &= ~TIM_CR1_CEN;
            reset_to_baseline(TEST_INSTANCE);
        }

        void start_periodic_repeats_until_paused() {
            reset_to_baseline(TEST_INSTANCE);

            s_update_count = 0;
            TEST_ASSERT_EQUAL(HAL_OK, timer_init(TEST_INSTANCE, TIMER_COUNTER_UP, update_done_cb, nullptr));
            TEST_ASSERT_EQUAL(HAL_OK, timer_start_periodic(TEST_INSTANCE, 200)); // short period: several fire quickly

            TEST_ASSERT_FALSE(TEST_INSTANCE->CR1 & TIM_CR1_OPM);
            constexpr uint32_t expected_bits = TIM_CR1_CEN | TIM_CR1_ARPE | TIM_CR1_URS;
            TEST_ASSERT_EQUAL_UINT32(expected_bits, TEST_INSTANCE->CR1 & expected_bits);

            TEST_ASSERT_TRUE_MESSAGE(wait_until([]() {
                                         return s_update_count >= 5;
                                     }),
                                     "Periodic timer did not fire repeatedly");
            TEST_ASSERT_TRUE(TEST_INSTANCE->CR1 & TIM_CR1_CEN); // still running, unlike OPM

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
            TEST_ASSERT_EQUAL(HAL_OK, timer_init(TEST_INSTANCE, TIMER_COUNTER_UP, update_done_cb, nullptr));
            TEST_ASSERT_EQUAL(HAL_OK, timer_start_periodic(TEST_INSTANCE, 1'000'000));

            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_STATE, timer_resume(TEST_INSTANCE));

            TEST_ASSERT_EQUAL(HAL_OK, timer_pause(TEST_INSTANCE));
            reset_to_baseline(TEST_INSTANCE);
        }

        void pause_and_resume_actually_halt_and_continue_the_counter() {
            reset_to_baseline(TEST_INSTANCE);
            TEST_ASSERT_EQUAL(HAL_OK, timer_init(TEST_INSTANCE, TIMER_COUNTER_UP, update_done_cb, nullptr));
            TEST_ASSERT_EQUAL(HAL_OK, timer_start_periodic(TEST_INSTANCE, 1'000'000)); // long; only CNT progression matters

            delay_us(50);
            TEST_ASSERT_EQUAL(HAL_OK, timer_pause(TEST_INSTANCE));
            const uint32_t held_cnt = TEST_INSTANCE->CNT;
            delay_us(200);
            TEST_ASSERT_EQUAL_UINT32(held_cnt, TEST_INSTANCE->CNT); // frozen while paused

            TEST_ASSERT_EQUAL(HAL_OK, timer_resume(TEST_INSTANCE));
            delay_us(50);
            TEST_ASSERT_TRUE(TEST_INSTANCE->CNT > held_cnt); // advancing again

            TEST_ASSERT_EQUAL(HAL_OK, timer_pause(TEST_INSTANCE));
            reset_to_baseline(TEST_INSTANCE);
        }

        void restart_rejects_when_the_timer_is_not_currently_running() {
            // timer_restart pauses first internally, which requires CEN to already be set
            reset_to_baseline(TEST_INSTANCE);
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_STATE, timer_restart(TEST_INSTANCE, 1000));
        }

        void restart_reapplies_oneshot_mode_with_a_new_timeout() {
            reset_to_baseline(TEST_INSTANCE);
            s_update_done = false;
            TEST_ASSERT_EQUAL(HAL_OK, timer_init(TEST_INSTANCE, TIMER_COUNTER_UP, update_done_cb, nullptr));
            TEST_ASSERT_EQUAL(HAL_OK, timer_start_oneshot(TEST_INSTANCE, 1'000'000)); // long, so it's safe to interrupt

            delay_us(50);
            TEST_ASSERT_EQUAL(HAL_OK, timer_restart(TEST_INSTANCE, 1000)); // much shorter this time
            const uint32_t cnt_after_restart = TEST_INSTANCE->CNT;         // sampled first, before any other assertion adds overhead

            constexpr uint32_t expected_bits = TIM_CR1_CEN | TIM_CR1_OPM;
            TEST_ASSERT_EQUAL_UINT32(expected_bits, TEST_INSTANCE->CR1 & expected_bits);

            // Restarted from (near) zero, not continuing from wherever the old 1s-period counter had drifted to.
            // Threshold is a fraction of the new period's ARR rather than an absolute tick count, so it
            // doesn't depend on the timer's clock rate or on debug-build instruction overhead.
            TEST_ASSERT_TRUE(cnt_after_restart < (TEST_INSTANCE->ARR / 10));

            TEST_ASSERT_TRUE_MESSAGE(wait_until([]() {
                                         return s_update_done;
                                     }),
                                     "Restarted oneshot never fired");
            TEST_ASSERT_FALSE(TEST_INSTANCE->CR1 & TIM_CR1_CEN); // still one-pulse behavior after restart

            reset_to_baseline(TEST_INSTANCE);
        }

        void restart_reapplies_periodic_mode_and_keeps_repeating() {
            reset_to_baseline(TEST_INSTANCE);
            s_update_count = 0;
            TEST_ASSERT_EQUAL(HAL_OK, timer_init(TEST_INSTANCE, TIMER_COUNTER_UP, update_done_cb, nullptr));
            TEST_ASSERT_EQUAL(HAL_OK, timer_start_periodic(TEST_INSTANCE, 1'000'000));

            delay_us(50);
            TEST_ASSERT_EQUAL(HAL_OK, timer_restart(TEST_INSTANCE, 200)); // much shorter: observe several cycles quickly
            TEST_ASSERT_FALSE(TEST_INSTANCE->CR1 & TIM_CR1_OPM);

            TEST_ASSERT_TRUE_MESSAGE(wait_until([]() {
                                         return s_update_count >= 5;
                                     }),
                                     "Restarted periodic timer did not keep repeating");
            TEST_ASSERT_TRUE(TEST_INSTANCE->CR1 & TIM_CR1_CEN);

            TEST_ASSERT_EQUAL(HAL_OK, timer_pause(TEST_INSTANCE));
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

        void get_frequency_hz_rejects_null_arguments() {
            uint32_t freq = 0;
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, timer_get_frequency_hz(nullptr, &freq));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, timer_get_frequency_hz(TEST_INSTANCE, nullptr));
        }

        void get_frequency_hz_matches_the_apb_doubling_rules() {
            const uint32_t sysclk = get_system_core_clock();
            const uint32_t apb1   = get_apb1_core_clock();
            uint32_t       freq   = 0;

            const uint32_t timpre_before = RCC->DCKCFGR & RCC_DCKCFGR_TIMPRE;

            RCC->DCKCFGR &= ~RCC_DCKCFGR_TIMPRE;                                     // standard mode
            TEST_ASSERT_EQUAL(HAL_OK, timer_get_frequency_hz(TEST_INSTANCE, &freq)); // TIM2 is on APB1
            const uint32_t expected_std = (sysclk == apb1) ? apb1 : (apb1 * 2);
            TEST_ASSERT_EQUAL_UINT32(expected_std, freq);

            RCC->DCKCFGR |= RCC_DCKCFGR_TIMPRE; // high frequency mode
            TEST_ASSERT_EQUAL(HAL_OK, timer_get_frequency_hz(TEST_INSTANCE, &freq));
            const uint32_t expected_hf = ((sysclk == apb1) || ((sysclk / apb1) == 2)) ? sysclk : (apb1 * 4);
            TEST_ASSERT_EQUAL_UINT32(expected_hf, freq);

            if (timpre_before) {
                RCC->DCKCFGR |= RCC_DCKCFGR_TIMPRE;
            } else {
                RCC->DCKCFGR &= ~RCC_DCKCFGR_TIMPRE;
            }
        }

        void set_arr_and_psc_rejects_a_timeout_that_overflows_the_supported_range() {
            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM3, true));
            TEST_ASSERT_EQUAL(HAL_ERR_NOT_SUPPORTED, timer_set_arr_and_psc(TIM3, UINT32_MAX));
            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM3, false));
        }

        void set_arr_and_psc_computes_values_that_reconstruct_the_requested_timeout() {
            reset_to_baseline(TEST_INSTANCE);

            constexpr uint32_t REQUESTED_TIMEOUT_US = 5000;
            TEST_ASSERT_EQUAL(HAL_OK, timer_set_arr_and_psc(TEST_INSTANCE, REQUESTED_TIMEOUT_US));

            uint32_t freq_hz = 0;
            TEST_ASSERT_EQUAL(HAL_OK, timer_get_frequency_hz(TEST_INSTANCE, &freq_hz));

            const uint64_t psc_plus_1          = (uint64_t)TEST_INSTANCE->PSC + 1;
            const uint64_t arr_plus_1          = (uint64_t)TEST_INSTANCE->ARR + 1;
            const uint64_t achieved_timeout_us = (psc_plus_1 * arr_plus_1 * 1'000'000ULL) / freq_hz;

            const uint64_t tolerance = REQUESTED_TIMEOUT_US / 100 + 1; // 1% slack for rounding
            TEST_ASSERT_TRUE(achieved_timeout_us >= REQUESTED_TIMEOUT_US - tolerance);
            TEST_ASSERT_TRUE(achieved_timeout_us <= REQUESTED_TIMEOUT_US + tolerance);

            reset_to_baseline(TEST_INSTANCE);
        }

        void register_callback_rejects_out_of_range_index() {
            // 8 timers are registered (TIM1,2,3,4,5,9,10,11); index 8 is one past the last
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, timer_register_callback(update_done_cb, nullptr, 8));
        }

        void register_callback_lets_the_isr_invoke_it_directly_bypassing_init() {
            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM3, true));
            TEST_ASSERT_EQUAL(HAL_OK, timer_deinit(TIM3));

            constexpr uint8_t TIM3_IDX = 2; // matches the declared array order in timer.c: TIM1,TIM2,TIM3,...
            s_update_done              = false;
            TEST_ASSERT_EQUAL(HAL_OK, timer_register_callback(update_done_cb, nullptr, TIM3_IDX));

            // timer_init is never called; the callback is wired purely through the index-based path
            TEST_ASSERT_EQUAL(HAL_OK, timer_start_oneshot(TIM3, 1000));
            TEST_ASSERT_TRUE_MESSAGE(wait_until([]() {
                                         return s_update_done;
                                     }),
                                     "Callback registered via timer_register_callback never fired");

            TEST_ASSERT_EQUAL(HAL_OK, timer_deinit(TIM3));
            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM3, false));
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
        RUN_TEST(init_sets_direction_bit_and_preserves_other_cr1_bits);
        RUN_TEST(init_down_direction_actually_counts_down);
        RUN_TEST(deinit_rejects_unknown_handle);
        RUN_TEST(deinit_clears_all_register_state);
        RUN_TEST(deinit_clears_the_registered_callback);
        RUN_TEST(deinit_stops_a_timer_that_is_currently_running);
        RUN_TEST(deinit_clears_rcr_on_tim1_only);
        RUN_TEST(deinit_on_tim10_does_not_touch_the_shared_irq_line_that_tim1_uses);
        RUN_TEST(start_oneshot_rejects_invalid_arguments);
        RUN_TEST(start_oneshot_rejects_when_already_running);
        RUN_TEST(start_oneshot_rejects_a_timeout_that_overflows_the_prescaler_range);
        RUN_TEST(start_oneshot_completes_stops_itself_and_invokes_the_callback);
        RUN_TEST(start_oneshot_clears_the_registered_callback_after_it_fires);
        RUN_TEST(start_oneshot_uses_the_shared_irq_for_tim1);
        RUN_TEST(start_periodic_rejects_invalid_arguments);
        RUN_TEST(start_periodic_rejects_when_already_running);
        RUN_TEST(start_periodic_repeats_until_paused);
        RUN_TEST(pause_rejects_null_and_a_timer_that_is_not_running);
        RUN_TEST(resume_rejects_null_and_a_timer_that_is_already_running);
        RUN_TEST(pause_and_resume_actually_halt_and_continue_the_counter);
        RUN_TEST(restart_rejects_when_the_timer_is_not_currently_running);
        RUN_TEST(restart_reapplies_oneshot_mode_with_a_new_timeout);
        RUN_TEST(restart_reapplies_periodic_mode_and_keeps_repeating);
        RUN_TEST(is_timer_on_apb1_matches_the_bus_map);
        RUN_TEST(is_timer_32_bits_matches_the_counter_width_map);
        RUN_TEST(get_frequency_hz_rejects_null_arguments);
        RUN_TEST(get_frequency_hz_matches_the_apb_doubling_rules);
        RUN_TEST(set_arr_and_psc_rejects_a_timeout_that_overflows_the_supported_range);
        RUN_TEST(set_arr_and_psc_computes_values_that_reconstruct_the_requested_timeout);
        RUN_TEST(register_callback_rejects_out_of_range_index);
        RUN_TEST(register_callback_lets_the_isr_invoke_it_directly_bypassing_init);

        UNITY_END();
        LOGI(TAG, "Done with all tests on the timer driver");
    }

} // namespace test::timer
