#include "Unity/unity.h"

#include "utils/board.h"
#include "utils/clock.h"
#include "utils/log.h"

#include <array>
#include <cstdint>


namespace test::clock {

    namespace {

        constexpr const char* TAG = "Clock_Test";

        struct system_clock_case_t {
            system_clock_t clock;
            uint32_t       expected_sysclk;
            uint32_t       expected_apb1;
            uint32_t       expected_apb2;
        };

        struct audio_clock_case_t {
            audio_clock_t clock;
            uint32_t      expected_freq;
        };

        // Expected values hand-derived from the PLLM/PLLN/PLLP presets in clock.c.
        // Ordering deliberately interleaves PLL-driven and DIRECT presets, and crosses
        // between HSE- and HSI-sourced PLLs, so a single sweep exercises every transition
        // class: PLL->PLL (different freq), PLL->DIRECT, DIRECT->PLL, and HSE<->HSI handoff.
        constexpr std::array<system_clock_case_t, 12> SYSTEM_CLOCK_CASES{{
            {HSI_PLL_DIRECT, HSI_VALUE_Hz, HSI_VALUE_Hz, HSI_VALUE_Hz},
            {HSE_PLL_100MHz, 100'000'000U, 50'000'000U, 100'000'000U},
            {HSE_PLL_DIRECT, HSE_VALUE_Hz, HSE_VALUE_Hz, HSE_VALUE_Hz},
            {HSI_PLL_96MHz, 96'000'000U, 48'000'000U, 96'000'000U},
            {HSE_PLL_84MHz, 84'000'000U, 42'000'000U, 84'000'000U},
            {HSI_PLL_DIRECT, HSI_VALUE_Hz, HSI_VALUE_Hz, HSI_VALUE_Hz},
            {HSE_PLL_64MHz, 64'000'000U, 32'000'000U, 64'000'000U},
            {HSI_PLL_48MHz, 48'000'000U, 48'000'000U, 48'000'000U},
            {HSE_PLL_96MHz, 96'000'000U, 48'000'000U, 96'000'000U},
            {HSI_PLL_84MHz, 84'000'000U, 42'000'000U, 84'000'000U},
            {HSE_PLL_48MHz, 48'000'000U, 48'000'000U, 48'000'000U},
            {HSI_PLL_64MHz, 64'000'000U, 32'000'000U, 64'000'000U},
        }};

        // plln/pllr in each preset are chosen so the audio PLL's VCO is normalized to 1MHz
        // regardless of whether HSE or HSI feeds it - so the resulting frequency should be
        // identical no matter which oscillator is currently driving the main SYSCLK.
        constexpr std::array<audio_clock_case_t, 4> AUDIO_CLOCK_CASES{{
            {AUDIO_PLL_76_8MHz, 76'800'000U},
            {AUDIO_PLL_135_5MHz, 135'500'000U},
            {AUDIO_PLL_151MHz, 151'000'000U},
            {AUDIO_PLL_172MHz, 172'000'000U},
        }};

        void assert_system_clock(system_clock_t expected_type, uint32_t expected_sysclk, uint32_t expected_apb1, uint32_t expected_apb2) {
            TEST_ASSERT_EQUAL(expected_type, get_system_core_clock_type());
            TEST_ASSERT_EQUAL_UINT32(expected_sysclk, get_system_core_clock());
            TEST_ASSERT_EQUAL_UINT32(expected_apb1, get_apb1_core_clock());
            TEST_ASSERT_EQUAL_UINT32(expected_apb2, get_apb2_core_clock());

            TEST_ASSERT_TRUE(get_system_core_clock() <= MAX_SYSTEM_CLOCK_Hz);
            TEST_ASSERT_TRUE(get_apb1_core_clock() <= MAX_APB1_CLOCK_Hz);
            TEST_ASSERT_TRUE(get_apb2_core_clock() <= MAX_APB2_CLOCK_Hz);
        }

        void assert_audio_clock(audio_clock_t expected_type, uint32_t expected_freq) {
            TEST_ASSERT_EQUAL(expected_type, get_audio_pll_clock_type());
            TEST_ASSERT_EQUAL_UINT32(expected_freq, get_audio_pll_clock());
            TEST_ASSERT_TRUE(get_audio_pll_clock() <= MAX_AUDIO_PLL_CLOCK_Hz);
        }

        // Must run first: verifies the reset-default state before any test has touched the
        // clock config. Order-dependent by nature, since unlike the heap tests there is no
        // way to "free" a clock state back to a blank slate between cases.
        void default_boot_state_is_correct() {
            assert_system_clock(HSI_PLL_DIRECT, HSI_VALUE_Hz, HSI_VALUE_Hz, HSI_VALUE_Hz);
            assert_audio_clock(AUDIO_PLL_DISABLE, 0);
        }

        void each_system_clock_preset_configures_correctly() {
            for (const auto& test_case : SYSTEM_CLOCK_CASES) {
                system_core_clock_config(test_case.clock);
                assert_system_clock(test_case.clock, test_case.expected_sysclk, test_case.expected_apb1, test_case.expected_apb2);
            }
        }

        // Explicit round trips, in addition to the sweep above, to pin down the specific
        // PLL-active-and-must-be-disabled-first path discussed while fixing the original bug.
        void direct_to_pll_round_trips_succeed() {
            system_core_clock_config(HSE_PLL_DIRECT);
            assert_system_clock(HSE_PLL_DIRECT, HSE_VALUE_Hz, HSE_VALUE_Hz, HSE_VALUE_Hz);

            system_core_clock_config(HSE_PLL_100MHz);
            assert_system_clock(HSE_PLL_100MHz, 100'000'000U, 50'000'000U, 100'000'000U);

            system_core_clock_config(HSE_PLL_DIRECT);
            assert_system_clock(HSE_PLL_DIRECT, HSE_VALUE_Hz, HSE_VALUE_Hz, HSE_VALUE_Hz);

            system_core_clock_config(HSI_PLL_DIRECT);
            assert_system_clock(HSI_PLL_DIRECT, HSI_VALUE_Hz, HSI_VALUE_Hz, HSI_VALUE_Hz);

            system_core_clock_config(HSI_PLL_100MHz);
            assert_system_clock(HSI_PLL_100MHz, 100'000'000U, 50'000'000U, 100'000'000U);

            system_core_clock_config(HSI_PLL_DIRECT);
            assert_system_clock(HSI_PLL_DIRECT, HSI_VALUE_Hz, HSI_VALUE_Hz, HSI_VALUE_Hz);
        }

        // Reconfiguring to the preset that's already active must not hang or corrupt state,
        // even though the PLL is already on and already selected as SYSCLK when this runs.
        void reconfiguring_same_preset_is_idempotent() {
            system_core_clock_config(HSE_PLL_100MHz);
            assert_system_clock(HSE_PLL_100MHz, 100'000'000U, 50'000'000U, 100'000'000U);

            system_core_clock_config(HSE_PLL_100MHz);
            assert_system_clock(HSE_PLL_100MHz, 100'000'000U, 50'000'000U, 100'000'000U);
        }

        void each_audio_clock_preset_configures_correctly() {
            // Hold SYSCLK on a known PLL-driven preset for this pass; source-independence
            // is verified separately below.
            system_core_clock_config(HSE_PLL_100MHz);

            for (const auto& test_case : AUDIO_CLOCK_CASES) {
                audio_pll_clock_config(test_case.clock);
                assert_audio_clock(test_case.clock, test_case.expected_freq);
            }

            audio_pll_clock_config(AUDIO_PLL_DISABLE);
            assert_audio_clock(AUDIO_PLL_DISABLE, 0);
        }

        // The audio PLL normalizes its input to 1MHz regardless of source, so the resulting
        // frequency must be identical whether HSE or HSI is feeding the main PLL.
        void audio_clock_frequency_is_source_independent() {
            system_core_clock_config(HSE_PLL_100MHz);
            audio_pll_clock_config(AUDIO_PLL_172MHz);
            assert_audio_clock(AUDIO_PLL_172MHz, 172'000'000U);

            // system_core_clock_config re-applies get_audio_pll_clock_type() internally after
            // switching sources - confirms that restore path also holds up across HSE<->HSI.
            system_core_clock_config(HSI_PLL_96MHz);
            assert_audio_clock(AUDIO_PLL_172MHz, 172'000'000U);

            audio_pll_clock_config(AUDIO_PLL_DISABLE);
        }

        // Regression coverage for the RCC->PLLCFGR/RCC->CFGR mixup: this is the exact code
        // path (audio PLL config while SYSCLK is fed directly, not via the main PLL) that bug
        // lived in.
        void audio_clock_works_with_direct_sysclk_sources() {
            system_core_clock_config(HSE_PLL_DIRECT);
            audio_pll_clock_config(AUDIO_PLL_172MHz);
            assert_audio_clock(AUDIO_PLL_172MHz, 172'000'000U);
            assert_system_clock(HSE_PLL_DIRECT, HSE_VALUE_Hz, HSE_VALUE_Hz, HSE_VALUE_Hz);

            system_core_clock_config(HSI_PLL_DIRECT);
            audio_pll_clock_config(AUDIO_PLL_76_8MHz);
            assert_audio_clock(AUDIO_PLL_76_8MHz, 76'800'000U);
            assert_system_clock(HSI_PLL_DIRECT, HSI_VALUE_Hz, HSI_VALUE_Hz, HSI_VALUE_Hz);

            audio_pll_clock_config(AUDIO_PLL_DISABLE);
        }

        void update_functions_are_read_only_and_consistent() {
            system_core_clock_config(HSE_PLL_96MHz);
            audio_pll_clock_config(AUDIO_PLL_151MHz);

            const uint32_t sysclk_before = get_system_core_clock();
            const uint32_t apb1_before   = get_apb1_core_clock();
            const uint32_t apb2_before   = get_apb2_core_clock();
            const uint32_t audio_before  = get_audio_pll_clock_type();

            // Calling the *_update() functions directly (bypassing *_config()) must be a
            // pure re-derivation from the live registers, not a source of drift.
            system_core_clock_update();
            audio_pll_clock_update();

            TEST_ASSERT_EQUAL_UINT32(sysclk_before, get_system_core_clock());
            TEST_ASSERT_EQUAL_UINT32(apb1_before, get_apb1_core_clock());
            TEST_ASSERT_EQUAL_UINT32(apb2_before, get_apb2_core_clock());
            TEST_ASSERT_EQUAL_UINT32(audio_before, get_audio_pll_clock_type());

            audio_pll_clock_config(AUDIO_PLL_DISABLE);
        }

    } // namespace

    void all() {
        LOGI(TAG, "Starting the tests on the system and audio PLL clock driver");

        // Preserve whatever clock config was active on entry (almost certainly the real
        // production config set up in system_init()) so every test suite that runs after
        // this one - UART baud rates, I2C timing, PWM/timer frequencies - isn't left
        // running against a clock state this suite happened to leave behind.
        const system_clock_t original_clock = get_system_core_clock_type();
        const audio_clock_t  original_audio = get_audio_pll_clock_type();

        RUN_TEST(default_boot_state_is_correct);
        RUN_TEST(each_system_clock_preset_configures_correctly);
        RUN_TEST(direct_to_pll_round_trips_succeed);
        RUN_TEST(reconfiguring_same_preset_is_idempotent);
        RUN_TEST(each_audio_clock_preset_configures_correctly);
        RUN_TEST(audio_clock_frequency_is_source_independent);
        RUN_TEST(audio_clock_works_with_direct_sysclk_sources);
        RUN_TEST(update_functions_are_read_only_and_consistent);

        system_core_clock_config(original_clock);
        audio_pll_clock_config(original_audio);

        LOGI(TAG, "Done with all tests on the system and audio PLL clock driver");
    }

} // namespace test::clock
