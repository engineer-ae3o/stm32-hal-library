#include "stm32f411xe.h"
#include "Unity/unity.h"

#include "utils/common.h"
#include "drivers/iwdg.h"
#include "utils/board.h"
#include "utils/err.h"

#include <cstdint>


namespace test::iwdg {

    namespace {

        constexpr const char* TAG = "IWDG_Test";

        constexpr uint32_t COUNTER_CLK_Hz = LSI_VALUE_Hz / (1U << (IWDG_PRESCALER + 2));
        constexpr uint32_t MAX_RELOAD_S   = IWDG_RLR_RL_Msk / COUNTER_CLK_Hz;

        // TESTS
        void reset_flag_is_clear_before_first_start() {
            // Must run before iwdg_start is ever called anywhere in the suite. If this is set, the
            // IWDG actually expired and reset the chip on a previous run. A real timeout event, not
            // a test artifact. We clear it so a future timeout is detectable again on the next boot.
            TEST_ASSERT_FALSE_MESSAGE(RCC->CSR & RCC_CSR_IWDGRSTF, "IWDG previously reset the chip; investigate before continuing");
            RCC->CSR |= RCC_CSR_RMVF;
        }

        void invalid_arg_is_rejected_and_leaves_registers_untouched() {
            const uint32_t pr_before  = IWDG->PR;
            const uint32_t rlr_before = IWDG->RLR;

            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, iwdg_start(MAX_RELOAD_S + 1U));

            // Rejected call must not have touched PR/RLR or unlocked the key register sequence
            TEST_ASSERT_EQUAL_UINT32(pr_before, IWDG->PR);
            TEST_ASSERT_EQUAL_UINT32(rlr_before, IWDG->RLR);
        }

        void reload_and_prescaler_are_set_correctly() {
            constexpr uint32_t REQUEST_S       = 10U;
            constexpr uint32_t EXPECTED_RELOAD = COUNTER_CLK_Hz * REQUEST_S;

            TEST_ASSERT_EQUAL(HAL_OK, iwdg_start(REQUEST_S));

            TEST_ASSERT_EQUAL_UINT32(IWDG_PRESCALER, (IWDG->PR & IWDG_PR_PR_Msk) >> IWDG_PR_PR_Pos);
            TEST_ASSERT_EQUAL_UINT32(EXPECTED_RELOAD, (IWDG->RLR & IWDG_RLR_RL_Msk) >> IWDG_RLR_RL_Pos);
        }

        // A second, larger request must fully replace the first, not accumulate into it.
        void reload_value_can_grow_on_a_second_call() {
            constexpr uint32_t FIRST_S  = 5U;
            constexpr uint32_t SECOND_S = 20U;
            constexpr uint32_t EXPECTED = COUNTER_CLK_Hz * SECOND_S;

            TEST_ASSERT_EQUAL(HAL_OK, iwdg_start(FIRST_S));
            TEST_ASSERT_EQUAL(HAL_OK, iwdg_start(SECOND_S));

            TEST_ASSERT_EQUAL_UINT32(EXPECTED, IWDG->RLR & IWDG_RLR_RL_Msk);
        }

        // The inverse: a second, smaller request must actually shrink the reload, proving
        // the write clears the previous value's bits rather than only ever setting new ones.
        void reload_value_can_shrink_on_a_second_call() {
            constexpr uint32_t FIRST_S  = 30U;
            constexpr uint32_t SECOND_S = 3U;
            constexpr uint32_t EXPECTED = COUNTER_CLK_Hz * SECOND_S;

            TEST_ASSERT_EQUAL(HAL_OK, iwdg_start(FIRST_S));
            TEST_ASSERT_EQUAL(HAL_OK, iwdg_start(SECOND_S));

            TEST_ASSERT_EQUAL_UINT32(EXPECTED, IWDG->RLR & IWDG_RLR_RL_Msk);
        }

        void max_reload_value_is_accepted_at_the_boundary() {
            TEST_ASSERT_EQUAL(HAL_OK, iwdg_start(MAX_RELOAD_S));
            TEST_ASSERT_TRUE(IWDG->RLR & IWDG_RLR_RL_Msk);
        }

        void debug_freeze_bit_is_set() {
            TEST_ASSERT_TRUE(DBGMCU->APB1FZ & DBGMCU_APB1_FZ_DBG_IWDG_STOP);
        }

        // KR is write-only with no readable side effect, so this can only confirm the call
        // doesn't hang or fault. The real proof that kicking works is the suite reaching
        // its end without a spontaneous reset.
        void kick_does_not_hang_or_fault() {
            for (int i = 0; i < 5; ++i) {
                iwdg_kick();
            }
            TEST_PASS();
        }

    } // namespace

    void all() {
        LOGI(TAG, "Starting the tests on the IWDG driver");

        RUN_TEST(reset_flag_is_clear_before_first_start);
        RUN_TEST(invalid_arg_is_rejected_and_leaves_registers_untouched);
        RUN_TEST(reload_and_prescaler_are_set_correctly);
        iwdg_kick();
        RUN_TEST(reload_value_can_grow_on_a_second_call);
        iwdg_kick();
        RUN_TEST(reload_value_can_shrink_on_a_second_call);
        iwdg_kick();
        RUN_TEST(max_reload_value_is_accepted_at_the_boundary);
        iwdg_kick();
        RUN_TEST(debug_freeze_bit_is_set);
        iwdg_kick();
        RUN_TEST(kick_does_not_hang_or_fault);

        LOGI(TAG, "Done with all tests on the IWDG driver");
    }

} // namespace test::iwdg
