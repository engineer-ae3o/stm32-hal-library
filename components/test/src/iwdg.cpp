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

        constexpr iwdg_prescaler_t TEST_PRESCALER = IWDG_PRESCALER_DIV256;
        constexpr uint32_t         COUNTER_CLK_Hz = LSI_VALUE_Hz / (1U << (TEST_PRESCALER + 2));
        constexpr uint32_t         MAX_RELOAD_MS  = (IWDG_RLR_RL_Msk * 1000) / COUNTER_CLK_Hz;

        // TESTS
        void reset_flag_is_clear_before_first_start() {
            // Must run before iwdg_start is ever called anywhere in the suite. If this is set, the
            // IWDG actually expired and reset the chip on a previous run. A real timeout event, not
            // a test artifact. We clear it so a future timeout is detectable again on the next boot.
            TEST_ASSERT_FALSE_MESSAGE(RCC->CSR & RCC_CSR_IWDGRSTF, "The IWDG previously reset the chip; investigate before continuing");
            RCC->CSR |= RCC_CSR_RMVF;
        }

        void invalid_arg_is_rejected_and_leaves_registers_untouched() {
            const uint32_t pr_before  = IWDG->PR;
            const uint32_t rlr_before = IWDG->RLR;

            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, iwdg_start(TEST_PRESCALER, MAX_RELOAD_MS + 1000));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, iwdg_start(static_cast<iwdg_prescaler_t>(IWDG_PRESCALER_DIV256 + 10), 1000));

            // Rejected calls must not have touched PR/RLR or unlocked the key register sequence
            TEST_ASSERT_EQUAL_UINT32(pr_before, IWDG->PR);
            TEST_ASSERT_EQUAL_UINT32(rlr_before, IWDG->RLR);
        }

        void reload_and_prescaler_are_set_correctly() {
            constexpr uint32_t REQUEST_S       = 10;
            constexpr uint32_t EXPECTED_RELOAD = COUNTER_CLK_Hz * REQUEST_S;

            TEST_ASSERT_EQUAL(HAL_OK, iwdg_start(TEST_PRESCALER, REQUEST_S * 1000));

            TEST_ASSERT_EQUAL_UINT32(TEST_PRESCALER, (IWDG->PR & IWDG_PR_PR_Msk) >> IWDG_PR_PR_Pos);
            TEST_ASSERT_EQUAL_UINT32(EXPECTED_RELOAD, (IWDG->RLR & IWDG_RLR_RL_Msk) >> IWDG_RLR_RL_Pos);
        }

        void reload_value_can_grow_on_a_second_call() {
            // A second request must fully replace the first, not accumulate into it.
            constexpr uint32_t FIRST_S         = 5;
            constexpr uint32_t SECOND_S        = 20;
            constexpr uint32_t EXPECTED_RELOAD = COUNTER_CLK_Hz * SECOND_S;

            TEST_ASSERT_EQUAL(HAL_OK, iwdg_start(TEST_PRESCALER, FIRST_S * 1000));
            TEST_ASSERT_EQUAL(HAL_OK, iwdg_start(TEST_PRESCALER, SECOND_S * 1000));

            TEST_ASSERT_EQUAL_UINT32(EXPECTED_RELOAD, (IWDG->RLR & IWDG_RLR_RL_Msk) >> IWDG_RLR_RL_Pos);
        }

        void reload_value_can_shrink_on_a_second_call() {
            // The inverse: a second, smaller request must actually shrink the reload, proving
            // the write clears the previous value's bits rather than only ever setting new ones.
            constexpr uint32_t FIRST_S         = 30;
            constexpr uint32_t SECOND_S        = 3;
            constexpr uint32_t EXPECTED_RELOAD = COUNTER_CLK_Hz * SECOND_S;

            TEST_ASSERT_EQUAL(HAL_OK, iwdg_start(TEST_PRESCALER, FIRST_S * 1000));
            TEST_ASSERT_EQUAL(HAL_OK, iwdg_start(TEST_PRESCALER, SECOND_S * 1000));

            TEST_ASSERT_EQUAL_UINT32(EXPECTED_RELOAD, (IWDG->RLR & IWDG_RLR_RL_Msk) >> IWDG_RLR_RL_Pos);
        }

        void prescaler_can_change_on_a_second_call() {
            // Changing prescaler on a later call must actually take effect,
            // not get masked out by a stale PR value from a previous call.
            constexpr iwdg_prescaler_t FIRST_PRESCALER = IWDG_PRESCALER_DIV4;
            constexpr uint32_t         FIRST_CLK_Hz    = LSI_VALUE_Hz / (1U << (FIRST_PRESCALER + 2));
            constexpr uint32_t         FIRST_RELOAD_MS = (IWDG_RLR_RL_Msk * 1000) / FIRST_CLK_Hz;

            constexpr iwdg_prescaler_t SECOND_PRESCALER = IWDG_PRESCALER_DIV256;
            constexpr uint32_t         SECOND_CLK_Hz    = LSI_VALUE_Hz / (1U << (SECOND_PRESCALER + 2));
            constexpr uint32_t         SECOND_RELOAD_MS = (IWDG_RLR_RL_Msk * 1000) / SECOND_CLK_Hz;

            TEST_ASSERT_EQUAL(HAL_OK, iwdg_start(FIRST_PRESCALER, FIRST_RELOAD_MS));
            TEST_ASSERT_EQUAL(HAL_OK, iwdg_start(SECOND_PRESCALER, SECOND_RELOAD_MS));

            TEST_ASSERT_EQUAL_UINT32(SECOND_PRESCALER, (IWDG->PR & IWDG_PR_PR_Msk) >> IWDG_PR_PR_Pos);
        }

        void max_reload_value_is_accepted_at_the_boundary() {
            TEST_ASSERT_EQUAL(HAL_OK, iwdg_start(TEST_PRESCALER, MAX_RELOAD_MS));

            const uint32_t expected_reload = (COUNTER_CLK_Hz * MAX_RELOAD_MS) / 1000;
            TEST_ASSERT_EQUAL_UINT32(expected_reload, (IWDG->RLR & IWDG_RLR_RL_Msk) >> IWDG_RLR_RL_Pos);
        }

        void debug_freeze_bit_is_set() {
            TEST_ASSERT_TRUE(DBGMCU->APB1FZ & DBGMCU_APB1_FZ_DBG_IWDG_STOP);
        }

        void kick_does_not_hang_or_fault() {
            // The KR is write only with no readable side effect, so this can only confirm the call
            // doesn't hang or fault. The real proof that kicking works is the suite reaching
            // its end without a spontaneous reset.
            for (size_t i = 0; i < 100; i++) {
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
        RUN_TEST(prescaler_can_change_on_a_second_call);
        iwdg_kick();
        RUN_TEST(max_reload_value_is_accepted_at_the_boundary);
        iwdg_kick();
        RUN_TEST(debug_freeze_bit_is_set);
        iwdg_kick();
        RUN_TEST(kick_does_not_hang_or_fault);

        LOGI(TAG, "Done with all tests on the IWDG driver");
    }

} // namespace test::iwdg
