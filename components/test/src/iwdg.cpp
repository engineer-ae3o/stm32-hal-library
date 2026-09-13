#include "stm32f411xe.h"
#include "Unity/unity.h"

#include "utils/common.h"
#include "drivers/iwdg.h"

#include <cstdint>


namespace test::iwdg {

    namespace {

        constexpr const char* TAG = "IWDG_Test";

        // Mirrors the driver's internal derivation: fixed prescaler of 256 (0b110),
        // LSI at 32.768kHz -> 128Hz counter clock, 12-bit reload register.
        constexpr uint32_t LSI_HZ         = 32'768U;
        constexpr uint32_t PRESCALER_BITS = 0b110U;
        constexpr uint32_t COUNTER_CLK_HZ = LSI_HZ / (1U << (PRESCALER_BITS + 2));
        constexpr uint32_t MAX_RELOAD     = 0xFFFU;

        // TESTS

        // Must run before iwdg_start is ever called anywhere in the suite. If this is set,
        // the IWDG actually expired and reset the chip on a previous run — a real timeout
        // event, not a test artifact. We clear it so a future genuine timeout is detectable
        // again on the next boot.
        void reset_flag_is_clear_before_first_start() {
            TEST_ASSERT_FALSE_MESSAGE(RCC->CSR & RCC_CSR_IWDGRSTF, "IWDG previously reset the chip; investigate before continuing");
            RCC->CSR |= RCC_CSR_RMVF;
        }

        void reload_and_prescaler_are_set_correctly() {
            constexpr uint32_t REQUEST_S       = 10U;
            constexpr uint32_t EXPECTED_RELOAD = COUNTER_CLK_HZ * REQUEST_S; // 1280, well under 0xFFF

            iwdg_start(REQUEST_S);

            TEST_ASSERT_EQUAL_UINT32(PRESCALER_BITS, IWDG->PR & 0b111U);
            TEST_ASSERT_EQUAL_UINT32(EXPECTED_RELOAD, IWDG->RLR & 0xFFFU);
        }

        void reload_value_clamps_to_12_bits() {
            // 100s * 128Hz = 12800, far past the 12-bit reload register's range.
            constexpr uint32_t REQUEST_S = 100U;

            iwdg_start(REQUEST_S);

            TEST_ASSERT_EQUAL_UINT32(MAX_RELOAD, IWDG->RLR & 0xFFFU);
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
        RUN_TEST(reload_and_prescaler_are_set_correctly);
        iwdg_kick();
        RUN_TEST(reload_value_clamps_to_12_bits);
        iwdg_kick();
        RUN_TEST(debug_freeze_bit_is_set);
        iwdg_kick();
        RUN_TEST(kick_does_not_hang_or_fault);

        LOGI(TAG, "Done with all tests on the IWDG driver");
    }

} // namespace test::iwdg
