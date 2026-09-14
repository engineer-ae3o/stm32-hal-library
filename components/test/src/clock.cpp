#include "stm32f411xe.h"
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
            bool           expected_on_hse;
        };

        struct audio_clock_case_t {
            audio_clock_t clock;
            uint32_t      expected_audio;
        };

        // Expected values hand-derived from the PLLM/PLLN/PLLP presets in clock.c.
        // The sequence is deliberately not a strict HSE<->HSI alternation: most steps
        // do cross sources (to exercise the PLLSRC-staleness class of bug), but a few
        // consecutive same source steps are mixed in so same source frequency changes,
        // PLL->DIRECT, and DIRECT->PLL transitions all get covered too, not just handoffs.
        constexpr auto SYSTEM_CLOCK_CASES = std::array<system_clock_case_t, 15>{{
            {
                .clock           = HSI_PLL_DIRECT,
                .expected_sysclk = HSI_VALUE_Hz,
                .expected_apb1   = HSI_VALUE_Hz,
                .expected_apb2   = HSI_VALUE_Hz,
                .expected_on_hse = false,
            },
            {
                .clock           = HSE_PLL_100MHz,
                .expected_sysclk = 100'000'000UL,
                .expected_apb1   = 50'000'000UL,
                .expected_apb2   = 100'000'000UL,
                .expected_on_hse = true,
            },
            {
                // same source freq step
                .clock           = HSE_PLL_84MHz,
                .expected_sysclk = 84'000'000UL,
                .expected_apb1   = 42'000'000UL,
                .expected_apb2   = 84'000'000UL,
                .expected_on_hse = true,
            },
            {
                // same source PLL->direct
                .clock           = HSE_PLL_DIRECT,
                .expected_sysclk = HSE_VALUE_Hz,
                .expected_apb1   = HSE_VALUE_Hz,
                .expected_apb2   = HSE_VALUE_Hz,
                .expected_on_hse = true,
            },
            {
                // same source direct->PLL
                .clock           = HSE_PLL_96MHz,
                .expected_sysclk = 96'000'000UL,
                .expected_apb1   = 48'000'000UL,
                .expected_apb2   = 96'000'000UL,
                .expected_on_hse = true,
            },
            {
                .clock           = HSI_PLL_100MHz,
                .expected_sysclk = 100'000'000UL,
                .expected_apb1   = 50'000'000UL,
                .expected_apb2   = 100'000'000UL,
                .expected_on_hse = false,
            },
            {
                // same source freq step
                .clock           = HSI_PLL_48MHz,
                .expected_sysclk = 48'000'000UL,
                .expected_apb1   = 48'000'000UL,
                .expected_apb2   = 48'000'000UL,
                .expected_on_hse = false,
            },
            {
                // same source PLL->direct
                .clock           = HSI_PLL_DIRECT,
                .expected_sysclk = HSI_VALUE_Hz,
                .expected_apb1   = HSI_VALUE_Hz,
                .expected_apb2   = HSI_VALUE_Hz,
                .expected_on_hse = false,
            },
            {
                // same source direct->PLL
                .clock           = HSI_PLL_96MHz,
                .expected_sysclk = 96'000'000UL,
                .expected_apb1   = 48'000'000UL,
                .expected_apb2   = 96'000'000UL,
                .expected_on_hse = false,
            },
            {
                .clock           = HSE_PLL_64MHz,
                .expected_sysclk = 64'000'000UL,
                .expected_apb1   = 32'000'000UL,
                .expected_apb2   = 64'000'000UL,
                .expected_on_hse = true,
            },
            {
                .clock           = HSI_PLL_84MHz,
                .expected_sysclk = 84'000'000UL,
                .expected_apb1   = 42'000'000UL,
                .expected_apb2   = 84'000'000UL,
                .expected_on_hse = false,
            },
            {
                .clock           = HSE_PLL_48MHz,
                .expected_sysclk = 48'000'000UL,
                .expected_apb1   = 48'000'000UL,
                .expected_apb2   = 48'000'000UL,
                .expected_on_hse = true,
            },
            {
                .clock           = HSI_PLL_64MHz,
                .expected_sysclk = 64'000'000UL,
                .expected_apb1   = 32'000'000UL,
                .expected_apb2   = 64'000'000UL,
                .expected_on_hse = false,
            },
            {
                .clock           = HSE_PLL_MATCH_HSI,
                .expected_sysclk = HSI_VALUE_Hz,
                .expected_apb1   = HSI_VALUE_Hz,
                .expected_apb2   = HSI_VALUE_Hz,
                .expected_on_hse = true,
            },
            {
                .clock           = HSI_PLL_MATCH_HSE,
                .expected_sysclk = HSE_VALUE_Hz,
                .expected_apb1   = HSE_VALUE_Hz,
                .expected_apb2   = HSE_VALUE_Hz,
                .expected_on_hse = false,
            },
        }};

        // plln/pllr in each preset are chosen so the audio PLL's VCO is normalized to 1MHz
        // regardless of whether HSE or HSI feeds it - so the resulting frequency should be
        // identical no matter which oscillator is currently driving the main SYSCLK.
        constexpr std::array<audio_clock_case_t, 4> AUDIO_CLOCK_CASES{{
            {.clock = AUDIO_PLL_76_8MHz, .expected_audio = 76'800'000UL},
            {.clock = AUDIO_PLL_135_5MHz, .expected_audio = 135'500'000UL},
            {.clock = AUDIO_PLL_151MHz, .expected_audio = 151'000'000UL},
            {.clock = AUDIO_PLL_172MHz, .expected_audio = 172'000'000UL},
        }};

        void assert_system_clock(
            system_clock_t expected_type, uint32_t expected_sysclk, uint32_t expected_apb1, uint32_t expected_apb2, bool expected_on_hse) {
            TEST_ASSERT_EQUAL(expected_type, get_system_core_clock_type());
            TEST_ASSERT_EQUAL_UINT32(expected_sysclk, get_system_core_clock());
            TEST_ASSERT_EQUAL_UINT32(expected_apb1, get_apb1_core_clock());
            TEST_ASSERT_EQUAL_UINT32(expected_apb2, get_apb2_core_clock());
            TEST_ASSERT_EQUAL(expected_on_hse, is_sysclk_on_hse());

            TEST_ASSERT_TRUE(get_system_core_clock() <= MAX_SYSTEM_CLOCK_Hz);
            TEST_ASSERT_TRUE(get_apb1_core_clock() <= MAX_APB1_CLOCK_Hz);
            TEST_ASSERT_TRUE(get_apb2_core_clock() <= MAX_APB2_CLOCK_Hz);
        }

        void assert_audio_clock(audio_clock_t expected_type, uint32_t expected_freq) {
            TEST_ASSERT_EQUAL(expected_type, get_audio_pll_clock_type());
            TEST_ASSERT_EQUAL_UINT32(expected_freq, get_audio_pll_clock());
            TEST_ASSERT_TRUE(get_audio_pll_clock() <= MAX_AUDIO_PLL_CLOCK_Hz);
        }

        void each_system_clock_preset_configures_correctly() {
            for (const auto& test_case : SYSTEM_CLOCK_CASES) {
                system_core_clock_config(test_case.clock);
                assert_system_clock(
                    test_case.clock, test_case.expected_sysclk, test_case.expected_apb1, test_case.expected_apb2, test_case.expected_on_hse);
            }
        }

        // Explicit round trips, in addition to the sweep above, to pin down the specific
        // PLL-active-and-must-be-disabled-first path discussed while fixing the original bug.
        void direct_to_pll_round_trips_succeed() {
            system_core_clock_config(HSE_PLL_DIRECT);
            assert_system_clock(HSE_PLL_DIRECT, HSE_VALUE_Hz, HSE_VALUE_Hz, HSE_VALUE_Hz, true);

            system_core_clock_config(HSE_PLL_100MHz);
            assert_system_clock(HSE_PLL_100MHz, 100'000'000UL, 50'000'000UL, 100'000'000UL, true);

            system_core_clock_config(HSE_PLL_DIRECT);
            assert_system_clock(HSE_PLL_DIRECT, HSE_VALUE_Hz, HSE_VALUE_Hz, HSE_VALUE_Hz, true);

            system_core_clock_config(HSI_PLL_DIRECT);
            assert_system_clock(HSI_PLL_DIRECT, HSI_VALUE_Hz, HSI_VALUE_Hz, HSI_VALUE_Hz, false);

            system_core_clock_config(HSI_PLL_100MHz);
            assert_system_clock(HSI_PLL_100MHz, 100'000'000UL, 50'000'000UL, 100'000'000UL, false);

            system_core_clock_config(HSI_PLL_DIRECT);
            assert_system_clock(HSI_PLL_DIRECT, HSI_VALUE_Hz, HSI_VALUE_Hz, HSI_VALUE_Hz, false);
        }

        // Reconfiguring to the preset that's already active must not hang or corrupt state,
        // even though the PLL is already on and already selected as SYSCLK when this runs.
        void reconfiguring_same_preset_is_idempotent() {
            system_core_clock_config(HSE_PLL_100MHz);
            assert_system_clock(HSE_PLL_100MHz, 100'000'000UL, 50'000'000UL, 100'000'000UL, true);

            system_core_clock_config(HSE_PLL_100MHz);
            assert_system_clock(HSE_PLL_100MHz, 100'000'000UL, 50'000'000UL, 100'000'000UL, true);
        }

        // Direct regression test for the PLLCFGR staleness class of bug: the audio PLL is
        // left disabled here (the common case), which is exactly the condition under which
        // audio_pll_clock_config_preset's PLLSRC resync gets skipped entirely. If
        // is_sysclk_on_hse() ever goes back to trusting a stale PLLSRC bit, this is the
        // sequence that will catch it - HSE-PLL-sourced, then switched straight to HSI direct,
        // with nothing in between to incidentally resync PLLCFGR.
        void is_sysclk_on_hse_survives_a_stale_pllcfgr() {
            audio_pll_clock_config(AUDIO_PLL_DISABLE);

            system_core_clock_config(HSE_PLL_100MHz);
            TEST_ASSERT_TRUE(is_sysclk_on_hse());

            system_core_clock_config(HSI_PLL_DIRECT);
            TEST_ASSERT_FALSE_MESSAGE(is_sysclk_on_hse(), "is_sysclk_on_hse reported HSE from a stale PLLCFGR after switching to HSI direct");

            // And the mirror direction, for symmetry.
            system_core_clock_config(HSI_PLL_100MHz);
            TEST_ASSERT_FALSE(is_sysclk_on_hse());

            system_core_clock_config(HSE_PLL_DIRECT);
            TEST_ASSERT_TRUE_MESSAGE(is_sysclk_on_hse(), "is_sysclk_on_hse reported HSI from a stale PLLCFGR after switching to HSE direct");
        }

        // The MATCH_ presets are the sharpest edge case: HSE_PLL_MATCH_HSI is genuinely
        // HSE-sourced despite landing on the HSI frequency, and HSI_PLL_MATCH_HSE is the
        // mirror. A frequency-based (rather than source-based) implementation would get
        // these backwards.
        void match_presets_report_their_actual_source_not_their_frequency() {
            system_core_clock_config(HSE_PLL_MATCH_HSI);
            assert_system_clock(HSE_PLL_MATCH_HSI, HSI_VALUE_Hz, HSI_VALUE_Hz, HSI_VALUE_Hz, true);

            system_core_clock_config(HSI_PLL_MATCH_HSE);
            assert_system_clock(HSI_PLL_MATCH_HSE, HSE_VALUE_Hz, HSE_VALUE_Hz, HSE_VALUE_Hz, false);
        }

        void each_audio_clock_preset_configures_correctly() {
            // Hold SYSCLK on a known PLL-driven preset for this pass; source-independence
            // is verified separately below.
            system_core_clock_config(HSE_PLL_100MHz);

            for (const auto& test_case : AUDIO_CLOCK_CASES) {
                audio_pll_clock_config(test_case.clock);
                assert_audio_clock(test_case.clock, test_case.expected_audio);
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
            assert_system_clock(HSE_PLL_DIRECT, HSE_VALUE_Hz, HSE_VALUE_Hz, HSE_VALUE_Hz, true);

            system_core_clock_config(HSI_PLL_DIRECT);
            audio_pll_clock_config(AUDIO_PLL_76_8MHz);
            assert_audio_clock(AUDIO_PLL_76_8MHz, 76'800'000U);
            assert_system_clock(HSI_PLL_DIRECT, HSI_VALUE_Hz, HSI_VALUE_Hz, HSI_VALUE_Hz, false);

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

        // Preserve whatever clock config was active on entry.
        const system_clock_t original_clock = get_system_core_clock_type();
        const audio_clock_t  original_audio = get_audio_pll_clock_type();

        RUN_TEST(each_system_clock_preset_configures_correctly);
        RUN_TEST(direct_to_pll_round_trips_succeed);
        RUN_TEST(reconfiguring_same_preset_is_idempotent);
        RUN_TEST(is_sysclk_on_hse_survives_a_stale_pllcfgr);
        RUN_TEST(match_presets_report_their_actual_source_not_their_frequency);
        RUN_TEST(each_audio_clock_preset_configures_correctly);
        RUN_TEST(audio_clock_frequency_is_source_independent);
        RUN_TEST(audio_clock_works_with_direct_sysclk_sources);
        RUN_TEST(update_functions_are_read_only_and_consistent);

        system_core_clock_config(original_clock);
        audio_pll_clock_config(original_audio);

        LOGI(TAG, "Done with all tests on the system and audio PLL clock driver");
    }

} // namespace test::clock
