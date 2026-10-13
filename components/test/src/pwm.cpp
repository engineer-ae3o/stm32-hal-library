#include "stm32f411xe.h"
#include "Unity/unity.h"

#include "drivers/src/timer_internals.h"
#include "drivers/timer_types.h"
#include "drivers/gpio_types.h"
#include "drivers/pwm_types.h"
#include "test/loopback.hpp"
#include "drivers/timer.h"
#include "utils/common.h"
#include "utils/clock.h"
#include "utils/board.h"
#include "drivers/pwm.h"
#include "test/pwm.hpp"
#include "utils/tick.h"
#include "utils/err.h"
#include "utils/log.h"

#include <array>
#include <cstdint>
#include <utility>


namespace test::pwm {

    namespace {

        constexpr const char* TAG = "PWM_Test";

        TIM_TypeDef* const TIM_ADV   = TIM1;  // Advanced, 16-bit
        TIM_TypeDef* const TIM_GP32  = TIM2;  // General purpose, 32-bit
        TIM_TypeDef* const TIM_GP16  = TIM3;  // General purpose, 16-bit
        TIM_TypeDef* const TIM_LITE2 = TIM9;  // Lite, 2 channels
        TIM_TypeDef* const TIM_LITE1 = TIM10; // Lite, 1 channel

        // Helpers
        inline void reset_pwm(TIM_TypeDef* handle) {
            TEST_ASSERT_EQUAL(HAL_OK, pwm_deinit(handle));
            TEST_ASSERT_EQUAL(HAL_OK, timer_reset(handle));
        }

        inline uint32_t get_gpio_moder(GPIO_TypeDef* port, gpio_pin_t pin) {
            return (port->MODER >> (std::to_underlying(pin) * 2U)) & 0b11U;
        }

        inline uint32_t get_gpio_afr(GPIO_TypeDef* port, gpio_pin_t pin) {
            const uint32_t p = std::to_underlying(pin);
            return (p < 8U) ? ((port->AFR[0] >> (p * 4U)) & 0xFU)
                            : ((port->AFR[1] >> ((p - 8U) * 4U)) & 0xFU);
        }

        // Per channel register layout. Mirrors the hardware's fixed CCMRx/CCER/CR2 bit
        // positions for each channel. DOesn' copy the driver's logic, just the wiring
        // any correct implementation has to target.
        struct channel_regs_t {
            volatile uint32_t* ccmr;
            // Bit masks/positions
            uint32_t ccxs_pos;
            uint32_t ocxpe;
            uint32_t ocxm_pos;
            uint32_t ocxm_mask;
            uint32_t ccxe;
            uint32_t ccxp;
            uint32_t ccxne;
            uint32_t ccxnp;
            uint32_t oisx;
            uint32_t oisxn;
        };

        channel_regs_t get_channel_regs(TIM_TypeDef*    handle,
                                        timer_channel_t channel) {
            switch (channel) {
                case TIMER_CHANNEL_1:
                    return {
                        .ccmr      = &handle->CCMR1,
                        .ccxs_pos  = TIM_CCMR1_CC1S_Pos,
                        .ocxpe     = TIM_CCMR1_OC1PE,
                        .ocxm_pos  = TIM_CCMR1_OC1M_Pos,
                        .ocxm_mask = TIM_CCMR1_OC1M,
                        .ccxe      = TIM_CCER_CC1E,
                        .ccxp      = TIM_CCER_CC1P,
                        .ccxne     = TIM_CCER_CC1NE,
                        .ccxnp     = TIM_CCER_CC1NP,
                        .oisx      = TIM_CR2_OIS1,
                        .oisxn     = TIM_CR2_OIS1N,
                    };
                case TIMER_CHANNEL_2:
                    return {
                        .ccmr      = &handle->CCMR1,
                        .ccxs_pos  = TIM_CCMR1_CC2S_Pos,
                        .ocxpe     = TIM_CCMR1_OC2PE,
                        .ocxm_pos  = TIM_CCMR1_OC2M_Pos,
                        .ocxm_mask = TIM_CCMR1_OC2M,
                        .ccxe      = TIM_CCER_CC2E,
                        .ccxp      = TIM_CCER_CC2P,
                        .ccxne     = TIM_CCER_CC2NE,
                        .ccxnp     = TIM_CCER_CC2NP,
                        .oisx      = TIM_CR2_OIS2,
                        .oisxn     = TIM_CR2_OIS2N,
                    };
                case TIMER_CHANNEL_3:
                    return {
                        .ccmr      = &handle->CCMR2,
                        .ccxs_pos  = TIM_CCMR2_CC3S_Pos,
                        .ocxpe     = TIM_CCMR2_OC3PE,
                        .ocxm_pos  = TIM_CCMR2_OC3M_Pos,
                        .ocxm_mask = TIM_CCMR2_OC3M,
                        .ccxe      = TIM_CCER_CC3E,
                        .ccxp      = TIM_CCER_CC3P,
                        .ccxne     = TIM_CCER_CC3NE,
                        .ccxnp     = TIM_CCER_CC3NP,
                        .oisx      = TIM_CR2_OIS3,
                        .oisxn     = TIM_CR2_OIS3N,
                    };
                case TIMER_CHANNEL_4:
                default:
                    return {
                        .ccmr      = &handle->CCMR2,
                        .ccxs_pos  = TIM_CCMR2_CC4S_Pos,
                        .ocxpe     = TIM_CCMR2_OC4PE,
                        .ocxm_pos  = TIM_CCMR2_OC4M_Pos,
                        .ocxm_mask = TIM_CCMR2_OC4M,
                        .ccxe      = TIM_CCER_CC4E,
                        .ccxp      = TIM_CCER_CC4P,
                        .ccxne     = 0,
                        .ccxnp     = 0,
                        .oisx      = TIM_CR2_OIS4,
                        .oisxn     = 0,
                    };
            }
        }

        volatile uint32_t* get_ccr(TIM_TypeDef*    handle,
                                   timer_channel_t channel) {
            switch (channel) {
                case TIMER_CHANNEL_1:
                    return &handle->CCR1;
                case TIMER_CHANNEL_2:
                    return &handle->CCR2;
                case TIMER_CHANNEL_3:
                    return &handle->CCR3;
                case TIMER_CHANNEL_4:
                    return &handle->CCR4;
                default:
                    TEST_FAIL();
                    return nullptr;
            }
        }

        // Independent decoding of the DTG byte, from the reference manual's
        // documented encoding ranges. Returns the dead time in ticks.
        uint32_t decode_dtg_ticks(uint8_t dtg) {
            if ((dtg & 0x80U) == 0) {
                return dtg; // Range 1: step 1
            } else if ((dtg & 0xC0U) == 0x80U) {
                return (64U + (dtg & 0x3FU)) * 2U; // Range 2: step 2
            } else if ((dtg & 0xE0U) == 0xC0U) {
                return (32U + (dtg & 0x1FU)) * 8U; // Range 3: step 8
            } else {
                return (32U + (dtg & 0x1FU)) * 16U; // Range 4: step 16
            }
        }

        volatile bool s_brk_fired = false;
        void          brk_cb(void*, timer_channel_t, timer_count_dir_t) {
            s_brk_fired = true;
        }

        template<typename predicate>
        bool wait_until(predicate pred) {
            volatile uint32_t timeout = TIMEOUT;
            while (!pred() && timeout) {
                timeout -= 1;
            }
            return pred();
        }

        constexpr timer_channel_config_t make_channel(timer_channel_t channel,
                                                      board_pin_t     pin,
                                                      bool invert = false,
                                                      bool idle   = false) {
            return {.output_polarity   = invert,
                    .output_idle_state = idle,
                    .channel           = channel,
                    .gpio_pin          = pin};
        }

        // TESTS
        void timer_init_rejects_invalid_arguments() {
            pwm_gp_timer_config_t config{};
            config.pwm_mode       = PWM_MODE_1;
            config.pwm_count_mode = OUTPUT_COMPARE_EDGE_LEFT_ALIGNED;
            config.channels[0] =
                make_channel(TIMER_CHANNEL_1, BOARD_TIM3_CH1_PA6);
            config.num_channels = 1;

            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG,
                              pwm_gp_timer_init(nullptr, &config));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG,
                              pwm_gp_timer_init(TIM_GP16, nullptr));

            auto zero_channels         = config;
            zero_channels.num_channels = 0;
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG,
                              pwm_gp_timer_init(TIM_GP16, &zero_channels));
        }

        void timer_init_rejects_too_many_channels_per_timer_class() {
            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM_GP16, true));
            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM_LITE2, true));
            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM_LITE1, true));

            pwm_gp_timer_config_t config{};
            config.pwm_mode       = PWM_MODE_1;
            config.pwm_count_mode = OUTPUT_COMPARE_EDGE_LEFT_ALIGNED;
            for (size_t i = 0; i < MAX_TIM2_CHANNELS; i++) {
                config.channels[i] = make_channel(
                    static_cast<timer_channel_t>(i), BOARD_TIM3_CH1_PA6);
            }

            config.num_channels = MAX_TIM1_CHANNELS + 1;
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG,
                              pwm_gp_timer_init(TIM_GP16, &config));

            config.num_channels = MAX_TIM9_CHANNELS + 1;
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG,
                              pwm_gp_timer_init(TIM_LITE2, &config));

            config.num_channels = MAX_TIM10_CHANNELS + 1;
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG,
                              pwm_gp_timer_init(TIM_LITE1, &config));

            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM_GP16, false));
            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM_LITE2, false));
            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM_LITE1, false));
        }

        void timer_init_rejects_non_upcounting_mode_on_the_lite_timers() {
            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM_LITE1, true));

            pwm_gp_timer_config_t config{};
            config.pwm_mode = PWM_MODE_1;
            config.channels[0] =
                make_channel(TIMER_CHANNEL_1, BOARD_TIM10_CH1_PB8);
            config.num_channels = 1;

            for (const auto mode : {OUTPUT_COMPARE_EDGE_RIGHT_ALIGNED,
                                    OUTPUT_COMPARE_CENTER_ALIGNED_MODE_1,
                                    OUTPUT_COMPARE_CENTER_ALIGNED_MODE_2,
                                    OUTPUT_COMPARE_CENTER_ALIGNED_MODE_3}) {
                config.pwm_count_mode = mode;
                TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG,
                                  pwm_gp_timer_init(TIM_LITE1, &config));
            }

            config.pwm_count_mode = OUTPUT_COMPARE_EDGE_LEFT_ALIGNED;
            TEST_ASSERT_EQUAL(HAL_OK, pwm_gp_timer_init(TIM_LITE1, &config));

            reset_pwm(TIM_LITE1);
            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM_LITE1, false));
        }

        void timer_init_rejects_an_unknown_channel_without_corrupting_state() {
            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM_GP16, true));
            reset_pwm(TIM_GP16);

            pwm_gp_timer_config_t config{};
            config.pwm_mode       = PWM_MODE_1;
            config.pwm_count_mode = OUTPUT_COMPARE_EDGE_LEFT_ALIGNED;
            config.channels[0]    = make_channel(
                static_cast<timer_channel_t>(0xFF), BOARD_TIM3_CH1_PA6);
            config.num_channels = 1;

            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG,
                              pwm_gp_timer_init(TIM_GP16, &config));
            TEST_ASSERT_EQUAL_UINT32(
                0,
                TIM_GP16
                    ->CCMR1); // Never written to, since the rejection happens before writeback

            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM_GP16, false));
        }

        void timer_init_programs_each_main_channels_registers() {
            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM_GP16, true));

            pwm_gp_timer_config_t config{};
            config.pwm_mode       = PWM_MODE_2;
            config.pwm_count_mode = OUTPUT_COMPARE_EDGE_LEFT_ALIGNED;

            config.channels[0] = make_channel(
                TIMER_CHANNEL_1, BOARD_TIM3_CH1_PA6, /*invert=*/false);
            config.channels[1] = make_channel(
                TIMER_CHANNEL_2, BOARD_TIM3_CH2_PA7, /*invert=*/true);
            config.channels[2] = make_channel(
                TIMER_CHANNEL_3, BOARD_TIM3_CH3_PB0, /*invert=*/false);
            config.channels[3] = make_channel(
                TIMER_CHANNEL_4, BOARD_TIM3_CH4_PB1, /*invert=*/true);
            config.num_channels = 4;

            TEST_ASSERT_EQUAL(HAL_OK, pwm_gp_timer_init(TIM_GP16, &config));

            for (const auto& ch_cfg : config.channels) {
                const auto regs = get_channel_regs(TIM_GP16, ch_cfg.channel);
                TEST_ASSERT_EQUAL_UINT32(0,
                                         (*regs.ccmr >> regs.ccxs_pos) &
                                             0b11U); // CCxS == 00: output
                TEST_ASSERT_TRUE(*regs.ccmr & regs.ocxpe);
                TEST_ASSERT_EQUAL_UINT32(std::to_underlying(PWM_MODE_2),
                                         (*regs.ccmr >> regs.ocxm_pos) &
                                             0b111U);
                TEST_ASSERT_TRUE(TIM_GP16->CCER & regs.ccxe);
                TEST_ASSERT_EQUAL(
                    ch_cfg.output_polarity,
                    static_cast<bool>(TIM_GP16->CCER & regs.ccxp));
            }

            TEST_ASSERT_TRUE(TIM_GP16->CR1 & TIM_CR1_ARPE);
            TEST_ASSERT_EQUAL_UINT32(
                std::to_underlying(OUTPUT_COMPARE_EDGE_LEFT_ALIGNED),
                TIM_GP16->CR1 & (TIM_CR1_CMS | TIM_CR1_DIR));

            reset_pwm(TIM_GP16);
            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM_GP16, false));
        }

        void timer_init_configures_the_gpio_pin() {
            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM_GP16, true));

            pwm_gp_timer_config_t config{};
            config.pwm_mode       = PWM_MODE_1;
            config.pwm_count_mode = OUTPUT_COMPARE_EDGE_LEFT_ALIGNED;
            config.channels[0] =
                make_channel(TIMER_CHANNEL_1, BOARD_TIM3_CH1_PA6);
            config.num_channels = 1;

            TEST_ASSERT_EQUAL(HAL_OK, pwm_gp_timer_init(TIM_GP16, &config));

            TEST_ASSERT_TRUE(RCC->AHB1ENR & RCC_AHB1ENR_GPIOAEN);
            TEST_ASSERT_EQUAL_UINT32(
                0b10U,
                get_gpio_moder(GPIOA, GPIO_PIN_6)); // Alternate function mode
            TEST_ASSERT_EQUAL_UINT32(
                2U,
                get_gpio_afr(GPIOA, GPIO_PIN_6)); // AF2, per BOARD_TIM3_CH1_PA6

            reset_pwm(TIM_GP16);
            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM_GP16, false));
        }

        void
        advanced_timer_init_rejects_invalid_arguments_and_non_advanced_timers() {
            pwm_advanced_timer_config_t config{};
            config.pwm_mode       = PWM_MODE_1;
            config.pwm_count_mode = OUTPUT_COMPARE_EDGE_LEFT_ALIGNED;
            config.wp_level =
                WP_OFF; // Never anything else yet. See note above the test suite
            config.channels[0] =
                make_channel(TIMER_CHANNEL_1, BOARD_TIM1_CH1_PA8);
            config.num_channels = 1;

            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG,
                              pwm_advanced_timer_init(nullptr, &config));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG,
                              pwm_advanced_timer_init(TIM_ADV, nullptr));

            auto zero_channels         = config;
            zero_channels.num_channels = 0;
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG,
                              pwm_advanced_timer_init(TIM_ADV, &zero_channels));

            auto too_many         = config;
            too_many.num_channels = MAX_TIM1_CHANNELS + 1;
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG,
                              pwm_advanced_timer_init(TIM_ADV, &too_many));

            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM_GP32, true));
            TEST_ASSERT_EQUAL(HAL_ERR_NOT_SUPPORTED,
                              pwm_advanced_timer_init(
                                  TIM_GP32, &config)); // not an advanced timer
            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM_GP32, false));
        }

        void advanced_timer_init_programs_main_and_complementary_channels() {
            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM_ADV, true));

            pwm_advanced_timer_config_t config{};
            config.pwm_mode       = PWM_MODE_1;
            config.pwm_count_mode = OUTPUT_COMPARE_EDGE_LEFT_ALIGNED;
            config.wp_level       = WP_OFF;
            config.use_complementary_channels = true;

            config.channels[0] = make_channel(
                TIMER_CHANNEL_1, BOARD_TIM1_CH1_PA8, /*invert=*/false);
            config.channels[1] = make_channel(
                TIMER_CHANNEL_2, BOARD_TIM1_CH2_PA9, /*invert=*/true);
            config.channels[2] = make_channel(
                TIMER_CHANNEL_3, BOARD_TIM1_CH3_PA10, /*invert=*/false);
            config.channels[3] = make_channel(
                TIMER_CHANNEL_4, BOARD_TIM1_CH4_PA11, /*invert=*/true);
            config.num_channels = 4;

            config.complementary_channels[0] = {.output_polarity   = true,
                                                .output_idle_state = true,
                                                .gpio_pin =
                                                    BOARD_TIM1_CH1N_PA7};
            config.complementary_channels[1] = {.output_polarity   = false,
                                                .output_idle_state = false,
                                                .gpio_pin =
                                                    BOARD_TIM1_CH2N_PB0};
            config.complementary_channels[2] = {.output_polarity   = true,
                                                .output_idle_state = true,
                                                .gpio_pin =
                                                    BOARD_TIM1_CH3N_PB1};

            TEST_ASSERT_EQUAL(HAL_OK,
                              pwm_advanced_timer_init(TIM_ADV, &config));

            for (const auto& ch_cfg : config.channels) {
                const auto regs = get_channel_regs(TIM_ADV, ch_cfg.channel);
                TEST_ASSERT_TRUE(TIM_ADV->CCER & regs.ccxe);
                TEST_ASSERT_EQUAL(ch_cfg.output_polarity,
                                  static_cast<bool>(TIM_ADV->CCER & regs.ccxp));
                TEST_ASSERT_EQUAL(ch_cfg.output_idle_state,
                                  static_cast<bool>(TIM_ADV->CR2 & regs.oisx));
            }

            // Channels 0-2 have complementary outputs
            for (size_t i = 0; i < 3; i++) {
                const auto  channel = static_cast<timer_channel_t>(i);
                const auto  regs    = get_channel_regs(TIM_ADV, channel);
                const auto& comp    = config.complementary_channels[i];
                TEST_ASSERT_TRUE(TIM_ADV->CCER & regs.ccxne);
                TEST_ASSERT_EQUAL(
                    comp.output_polarity,
                    static_cast<bool>(TIM_ADV->CCER & regs.ccxnp));
                TEST_ASSERT_EQUAL(comp.output_idle_state,
                                  static_cast<bool>(TIM_ADV->CR2 & regs.oisxn));
            }

            // Channel 3 has no complementary output: its N bits must stay clear
            TEST_ASSERT_FALSE(TIM_ADV->CCER & TIM_CCER_CC4NP);

            reset_pwm(TIM_ADV);
            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM_ADV, false));
        }

        void advanced_timer_init_programs_ossr_ossi_repetition_and_clk_div() {
            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM_ADV, true));

            pwm_advanced_timer_config_t config{};
            config.pwm_mode       = PWM_MODE_1;
            config.pwm_count_mode = OUTPUT_COMPARE_CENTER_ALIGNED_MODE_1;
            config.wp_level       = WP_OFF;
            config.ossr           = true;
            config.ossi           = true;
            config.clk_div        = TIM_CLK_DIV_4;
            config.repetition_cnt = 7;
            config.channels[0] =
                make_channel(TIMER_CHANNEL_1, BOARD_TIM1_CH1_PA8);
            config.num_channels = 1;

            TEST_ASSERT_EQUAL(HAL_OK,
                              pwm_advanced_timer_init(TIM_ADV, &config));

            TEST_ASSERT_TRUE(TIM_ADV->BDTR & TIM_BDTR_OSSR);
            TEST_ASSERT_TRUE(TIM_ADV->BDTR & TIM_BDTR_OSSI);
            TEST_ASSERT_EQUAL_UINT32(7, TIM_ADV->RCR & TIM_RCR_REP);
            TEST_ASSERT_EQUAL_UINT32(std::to_underlying(TIM_CLK_DIV_4),
                                     (TIM_ADV->CR1 & TIM_CR1_CKD) >>
                                         TIM_CR1_CKD_Pos);
            TEST_ASSERT_EQUAL_UINT32(
                std::to_underlying(OUTPUT_COMPARE_CENTER_ALIGNED_MODE_1),
                TIM_ADV->CR1 & TIM_CR1_CMS);

            reset_pwm(TIM_ADV);
            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM_ADV, false));
        }

        void
        advanced_timer_init_dead_time_lands_in_the_correct_encoding_range() {
            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM_ADV, true));
            const uint32_t timer_freq_hz = timer_get_frequency_hz(TIM_ADV);

            struct range_t {
                uint32_t target_ticks;
                uint32_t step;
            };
            // One representative target per documented range, well clear of the boundaries
            constexpr auto RANGES = std::array<range_t, 4>{{
                {.target_ticks = 50, .step = 1},   // Range 1
                {.target_ticks = 200, .step = 2},  // Range 2
                {.target_ticks = 400, .step = 8},  // Range 3
                {.target_ticks = 800, .step = 16}, // Range 4
            }};

            for (const auto& range : RANGES) {
                reset_pwm(TIM_ADV);

                constexpr uint64_t DENOM =
                    1'000'000'000ULL; // clk_div == TIM_CLK_DIV_1
                const uint32_t dead_time_ns = (uint32_t)ceil_div_u64(
                    (uint64_t)range.target_ticks * DENOM, timer_freq_hz);

                pwm_advanced_timer_config_t config{};
                config.pwm_mode       = PWM_MODE_1;
                config.pwm_count_mode = OUTPUT_COMPARE_EDGE_LEFT_ALIGNED;
                config.wp_level       = WP_OFF;
                config.clk_div        = TIM_CLK_DIV_1;
                config.dead_time_ns   = dead_time_ns;
                config.channels[0] =
                    make_channel(TIMER_CHANNEL_1, BOARD_TIM1_CH1_PA8);
                config.num_channels = 1;

                TEST_ASSERT_EQUAL(HAL_OK,
                                  pwm_advanced_timer_init(TIM_ADV, &config));

                const uint8_t dtg =
                    (TIM_ADV->BDTR & TIM_BDTR_DTG) >> TIM_BDTR_DTG_Pos;
                const uint32_t expected_ticks = (uint32_t)ceil_div_u64(
                    (uint64_t)dead_time_ns * timer_freq_hz, DENOM);
                const uint32_t decoded_ticks = decode_dtg_ticks(dtg);

                // Encoding rounds UP to the nearest representable value for its range's step size
                TEST_ASSERT_TRUE(decoded_ticks >= expected_ticks);
                TEST_ASSERT_TRUE(decoded_ticks < expected_ticks + range.step);
            }

            reset_pwm(TIM_ADV);
            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM_ADV, false));
        }

        void advanced_timer_init_dead_time_zero_leaves_dtg_untouched() {
            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM_ADV, true));
            reset_pwm(TIM_ADV);

            pwm_advanced_timer_config_t config{};
            config.pwm_mode       = PWM_MODE_1;
            config.pwm_count_mode = OUTPUT_COMPARE_EDGE_LEFT_ALIGNED;
            config.wp_level       = WP_OFF;
            config.dead_time_ns   = 0;
            config.channels[0] =
                make_channel(TIMER_CHANNEL_1, BOARD_TIM1_CH1_PA8);
            config.num_channels = 1;

            TEST_ASSERT_EQUAL(HAL_OK,
                              pwm_advanced_timer_init(TIM_ADV, &config));
            TEST_ASSERT_EQUAL_UINT32(0, TIM_ADV->BDTR & TIM_BDTR_DTG);

            reset_pwm(TIM_ADV);
            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM_ADV, false));
        }

        void
        advanced_timer_init_rejects_dead_time_beyond_the_max_encodable_range() {
            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM_ADV, true));
            reset_pwm(TIM_ADV);
            const uint32_t timer_freq_hz = timer_get_frequency_hz(TIM_ADV);

            // One tick past range 4's ceiling of 1008
            constexpr uint64_t DENOM = 1'000'000'000ULL;
            const uint32_t     dead_time_ns =
                (uint32_t)ceil_div_u64(1009ULL * DENOM, timer_freq_hz);

            pwm_advanced_timer_config_t config{};
            config.pwm_mode       = PWM_MODE_1;
            config.pwm_count_mode = OUTPUT_COMPARE_EDGE_LEFT_ALIGNED;
            config.wp_level       = WP_OFF;
            config.clk_div        = TIM_CLK_DIV_1;
            config.dead_time_ns   = dead_time_ns;
            config.channels[0] =
                make_channel(TIMER_CHANNEL_1, BOARD_TIM1_CH1_PA8);
            config.num_channels = 1;

            TEST_ASSERT_EQUAL(HAL_ERR_NOT_SUPPORTED,
                              pwm_advanced_timer_init(TIM_ADV, &config));
            TEST_ASSERT_EQUAL_UINT32(
                0, TIM_ADV->BDTR); // Rejected before BDTR is ever written

            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM_ADV, false));
        }

        void
        advanced_timer_init_configures_the_break_input_and_fires_the_callback() {
            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM_ADV, true));
            reset_pwm(TIM_ADV);

            pwm_advanced_timer_config_t config{};
            config.pwm_mode       = PWM_MODE_1;
            config.pwm_count_mode = OUTPUT_COMPARE_EDGE_LEFT_ALIGNED;
            config.wp_level       = WP_OFF;
            config.channels[0] =
                make_channel(TIMER_CHANNEL_1, BOARD_TIM1_CH1_PA8);
            config.num_channels = 1;

            config.break_input.use_break_input = true;
            config.break_input.active_low      = false;
            config.break_input.auto_rearm      = true;
            config.break_input.gpio_pin        = BOARD_TIM1_BKIN_PA6;
            config.break_input.callback        = {.cb = brk_cb, .arg = nullptr};

            s_brk_fired = false;
            TEST_ASSERT_EQUAL(HAL_OK,
                              pwm_advanced_timer_init(TIM_ADV, &config));

            TEST_ASSERT_TRUE(TIM_ADV->BDTR & TIM_BDTR_BKE);
            TEST_ASSERT_TRUE(
                TIM_ADV->BDTR &
                TIM_BDTR_BKP); // active_low == false -> BKP set (active high)
            TEST_ASSERT_TRUE(TIM_ADV->BDTR & TIM_BDTR_AOE);
            TEST_ASSERT_TRUE(TIM_ADV->DIER & TIM_DIER_BIE);
            TEST_ASSERT_TRUE(RCC->AHB1ENR & RCC_AHB1ENR_GPIOAEN);
            TEST_ASSERT_EQUAL_UINT32(
                1U,
                get_gpio_afr(GPIOA,
                             GPIO_PIN_6)); // AF1, per BOARD_TIM1_BKIN_PA6

            // Simulate a break event with the BG bit in the event generation register.
            TIM_ADV->EGR = TIM_EGR_BG;

            TEST_ASSERT_TRUE_MESSAGE(wait_until([]() {
                                         return s_brk_fired;
                                     }),
                                     "Break event callback never fired");
            TEST_ASSERT_FALSE(TIM_ADV->SR & TIM_SR_BIF);

            reset_pwm(TIM_ADV);
            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM_ADV, false));
        }

        void advanced_timer_init_active_low_break_clears_bkp() {
            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM_ADV, true));
            reset_pwm(TIM_ADV);

            pwm_advanced_timer_config_t config{};
            config.pwm_mode       = PWM_MODE_1;
            config.pwm_count_mode = OUTPUT_COMPARE_EDGE_LEFT_ALIGNED;
            config.wp_level       = WP_OFF;
            config.channels[0] =
                make_channel(TIMER_CHANNEL_1, BOARD_TIM1_CH1_PA8);
            config.num_channels = 1;

            config.break_input.use_break_input = true;
            config.break_input.active_low      = true;
            config.break_input.gpio_pin        = BOARD_TIM1_BKIN_PA6;
            config.break_input.callback        = {.cb = brk_cb, .arg = nullptr};

            TEST_ASSERT_EQUAL(HAL_OK,
                              pwm_advanced_timer_init(TIM_ADV, &config));
            TEST_ASSERT_FALSE(TIM_ADV->BDTR & TIM_BDTR_BKP);

            reset_pwm(TIM_ADV);
            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM_ADV, false));
        }

        void start_rejects_invalid_arguments() {
            auto*    bogus    = reinterpret_cast<TIM_TypeDef*>(1);
            uint32_t max_duty = 0;
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG,
                              pwm_start(nullptr, 1000, &max_duty));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG,
                              pwm_start(bogus, 0, &max_duty));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG,
                              pwm_start(bogus, 1000, nullptr));
        }

        void start_rejects_a_frequency_above_the_timer_clock() {
            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM_GP16, true));
            reset_pwm(TIM_GP16);
            const uint32_t timer_freq_hz = timer_get_frequency_hz(TIM_GP16);

            uint32_t max_duty = 0;
            TEST_ASSERT_EQUAL(
                HAL_ERR_NOT_SUPPORTED,
                pwm_start(TIM_GP16, timer_freq_hz + 1, &max_duty));

            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM_GP16, false));
        }

        void
        start_edge_aligned_hits_the_requested_frequency_on_16_and_32_bit_timers() {
            for (auto* const handle : {TIM_GP16, TIM_GP32}) {
                TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(handle, true));
                reset_pwm(handle);

                constexpr uint32_t REQUESTED_HZ = 1000;
                uint32_t           max_duty     = 0;
                TEST_ASSERT_EQUAL(HAL_OK,
                                  pwm_start(handle, REQUESTED_HZ, &max_duty));

                const uint32_t timer_freq_hz = timer_get_frequency_hz(handle);
                const uint64_t psc_plus_1    = (uint64_t)handle->PSC + 1;
                const uint64_t arr_plus_1    = (uint64_t)handle->ARR + 1;
                const uint64_t achieved_hz =
                    timer_freq_hz / (psc_plus_1 * arr_plus_1);

                constexpr uint64_t tolerance = (REQUESTED_HZ / 100) + 1;
                TEST_ASSERT_TRUE(achieved_hz >= (REQUESTED_HZ - tolerance));
                TEST_ASSERT_TRUE(achieved_hz <= (REQUESTED_HZ + tolerance));

                TEST_ASSERT_EQUAL_UINT32(
                    handle->ARR + 1, max_duty); // Edge aligned: max == ARR + 1

                TEST_ASSERT_TRUE(handle->CR1 & TIM_CR1_CEN);
                TEST_ASSERT_FALSE(handle->SR & TIM_SR_UIF);

                TEST_ASSERT_EQUAL(HAL_OK, pwm_freeze_timer(handle));
                reset_pwm(handle);
                TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(handle, false));
            }
        }

        void start_center_aligned_uses_the_doubled_period_formula() {
            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM_GP16, true));
            reset_pwm(TIM_GP16);
            TIM_GP16->CR1 |= std::to_underlying(
                OUTPUT_COMPARE_CENTER_ALIGNED_MODE_1); // Fake the mode directly; pwm_start only reads CMS

            constexpr uint32_t REQUESTED_HZ = 500;
            uint32_t           max_duty     = 0;
            TEST_ASSERT_EQUAL(HAL_OK,
                              pwm_start(TIM_GP16, REQUESTED_HZ, &max_duty));

            const uint32_t timer_freq_hz = timer_get_frequency_hz(TIM_GP16);
            const uint64_t psc_plus_1    = (uint64_t)TIM_GP16->PSC + 1;
            const uint64_t arr           = TIM_GP16->ARR;
            const uint64_t achieved_hz =
                timer_freq_hz / (2ULL * psc_plus_1 * arr);

            constexpr uint64_t tolerance = (REQUESTED_HZ / 100) + 1;
            TEST_ASSERT_TRUE(achieved_hz >= (REQUESTED_HZ - tolerance));
            TEST_ASSERT_TRUE(achieved_hz <= (REQUESTED_HZ + tolerance));

            TEST_ASSERT_EQUAL_UINT32(
                TIM_GP16->ARR, max_duty); // Center aligned: max == ARR, no +1

            TEST_ASSERT_EQUAL(HAL_OK, pwm_freeze_timer(TIM_GP16));
            reset_pwm(TIM_GP16);
            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM_GP16, false));
        }

        void set_duty_cycle_rejects_invalid_arguments() {
            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM_GP16, true));
            reset_pwm(TIM_GP16);
            TIM_GP16->ARR =
                999; // Edge aligned by default (CMS == 0): max duty == 1000

            TEST_ASSERT_EQUAL(
                HAL_ERR_INVALID_ARG,
                pwm_set_duty_cycle(nullptr, TIMER_CHANNEL_1, 500));
            TEST_ASSERT_EQUAL(
                HAL_ERR_INVALID_ARG,
                pwm_set_duty_cycle(TIM_GP16, TIMER_CHANNEL_1, 999 + 2));
            TEST_ASSERT_EQUAL(
                HAL_ERR_INVALID_ARG,
                pwm_set_duty_cycle(
                    TIM_GP16, static_cast<timer_channel_t>(0xFF), 500));

            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM_GP16, false));
        }

        void set_duty_cycle_edge_aligned_boundary_is_arr_plus_one() {
            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM_GP16, true));
            reset_pwm(TIM_GP16);
            TIM_GP16->ARR = 999; // CMS == 0: edge aligned, max == 1000

            TEST_ASSERT_EQUAL(
                HAL_OK, pwm_set_duty_cycle(TIM_GP16, TIMER_CHANNEL_1, 1000));
            TEST_ASSERT_EQUAL(
                HAL_ERR_INVALID_ARG,
                pwm_set_duty_cycle(TIM_GP16, TIMER_CHANNEL_1, 1001));

            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM_GP16, false));
        }

        void set_duty_cycle_center_aligned_boundary_is_arr() {
            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM_GP16, true));
            reset_pwm(TIM_GP16);
            TIM_GP16->ARR = 999;
            TIM_GP16->CR1 |= std::to_underlying(
                OUTPUT_COMPARE_CENTER_ALIGNED_MODE_1); // CMS != 0: center aligned, max == ARR == 999

            TEST_ASSERT_EQUAL(
                HAL_OK, pwm_set_duty_cycle(TIM_GP16, TIMER_CHANNEL_1, 999));
            TEST_ASSERT_EQUAL(
                HAL_ERR_INVALID_ARG,
                pwm_set_duty_cycle(TIM_GP16, TIMER_CHANNEL_1, 1000));

            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM_GP16, false));
        }

        void set_duty_cycle_edge_aligned_off_by_one_at_the_32_bit_ceiling() {
            // On a 32 bit timer with ARR at its absolute max, edge aligned max duty is clamped
            // to ARR itself (not ARR + 1, which would wrap to 0). See get_max_duty_cycle's comment.
            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM_GP32, true));
            reset_pwm(TIM_GP32);
            TIM_GP32->ARR = UINT32_MAX;

            TEST_ASSERT_EQUAL(
                HAL_OK,
                pwm_set_duty_cycle(TIM_GP32, TIMER_CHANNEL_1, UINT32_MAX));

            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM_GP32, false));
        }

        void set_duty_cycle_writes_the_correct_compare_register() {
            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM_GP16, true));
            reset_pwm(TIM_GP16);
            TIM_GP16->ARR = 999;

            for (const auto channel : {TIMER_CHANNEL_1,
                                       TIMER_CHANNEL_2,
                                       TIMER_CHANNEL_3,
                                       TIMER_CHANNEL_4}) {
                TEST_ASSERT_EQUAL(HAL_OK,
                                  pwm_set_duty_cycle(TIM_GP16, channel, 250));
                TEST_ASSERT_EQUAL_UINT32(250, *get_ccr(TIM_GP16, channel));
            }

            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM_GP16, false));
        }

        void freeze_and_unfreeze_toggle_cen_and_moe_on_the_advanced_timer() {
            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM_ADV, true));
            reset_pwm(TIM_ADV);

            TEST_ASSERT_EQUAL(HAL_OK, pwm_unfreeze_timer(TIM_ADV));
            TEST_ASSERT_TRUE(TIM_ADV->CR1 & TIM_CR1_CEN);
            TEST_ASSERT_TRUE(TIM_ADV->BDTR & TIM_BDTR_MOE);

            TEST_ASSERT_EQUAL(HAL_OK, pwm_freeze_timer(TIM_ADV));
            TEST_ASSERT_FALSE(TIM_ADV->CR1 & TIM_CR1_CEN);
            TEST_ASSERT_FALSE(TIM_ADV->BDTR & TIM_BDTR_MOE);

            reset_pwm(TIM_ADV);
            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM_ADV, false));
        }

        void freeze_and_unfreeze_toggle_only_cen_on_general_purpose_timers() {
            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM_GP16, true));
            reset_pwm(TIM_GP16);

            TEST_ASSERT_EQUAL(HAL_OK, pwm_unfreeze_timer(TIM_GP16));
            TEST_ASSERT_TRUE(TIM_GP16->CR1 & TIM_CR1_CEN);

            TEST_ASSERT_EQUAL(HAL_OK, pwm_freeze_timer(TIM_GP16));
            TEST_ASSERT_FALSE(TIM_GP16->CR1 & TIM_CR1_CEN);

            reset_pwm(TIM_GP16);
            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM_GP16, false));
        }

        void pause_and_resume_channel_reject_invalid_arguments() {
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG,
                              pwm_pause_channel(nullptr, TIMER_CHANNEL_1));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG,
                              pwm_resume_channel(nullptr, TIMER_CHANNEL_1));

            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM_GP16, true));
            TEST_ASSERT_EQUAL(
                HAL_ERR_INVALID_ARG,
                pwm_pause_channel(TIM_GP16, static_cast<timer_channel_t>(4)));
            TEST_ASSERT_EQUAL(
                HAL_ERR_INVALID_ARG,
                pwm_resume_channel(TIM_GP16, static_cast<timer_channel_t>(4)));
            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM_GP16, false));
        }

        void
        pause_and_resume_channel_toggle_only_the_main_output_on_general_purpose_timers() {
            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM_GP16, true));
            reset_pwm(TIM_GP16);
            TIM_GP16->CCER |=
                TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;

            for (const auto channel : {TIMER_CHANNEL_1,
                                       TIMER_CHANNEL_2,
                                       TIMER_CHANNEL_3,
                                       TIMER_CHANNEL_4}) {
                const auto regs = get_channel_regs(TIM_GP16, channel);
                TEST_ASSERT_EQUAL(HAL_OK, pwm_pause_channel(TIM_GP16, channel));
                TEST_ASSERT_FALSE(TIM_GP16->CCER & regs.ccxe);

                TEST_ASSERT_EQUAL(HAL_OK,
                                  pwm_resume_channel(TIM_GP16, channel));
                TEST_ASSERT_TRUE(TIM_GP16->CCER & regs.ccxe);
            }

            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM_GP16, false));
        }

        void
        pause_and_resume_channel_also_toggle_the_complementary_output_on_the_advanced_timer() {
            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM_ADV, true));
            reset_pwm(TIM_ADV);
            TIM_ADV->CCER = TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC4E;

            TEST_ASSERT_EQUAL(HAL_OK,
                              pwm_pause_channel(TIM_ADV, TIMER_CHANNEL_1));
            TEST_ASSERT_EQUAL(
                HAL_OK,
                pwm_pause_complementary_channel(TIM_ADV, TIMER_CHANNEL_1));
            TEST_ASSERT_FALSE(TIM_ADV->CCER & (TIM_CCER_CC1E | TIM_CCER_CC1NE));

            TEST_ASSERT_EQUAL(HAL_OK,
                              pwm_resume_channel(TIM_ADV, TIMER_CHANNEL_1));
            TEST_ASSERT_EQUAL(
                HAL_OK,
                pwm_resume_complementary_channel(TIM_ADV, TIMER_CHANNEL_1));
            TEST_ASSERT_TRUE(TIM_ADV->CCER & (TIM_CCER_CC1E | TIM_CCER_CC1NE));

            // Channel 4 has no complementary output: pausing it must not touch bits outside CC4E
            TEST_ASSERT_EQUAL(HAL_OK,
                              pwm_pause_channel(TIM_ADV, TIMER_CHANNEL_4));
            TEST_ASSERT_FALSE(TIM_ADV->CCER & TIM_CCER_CC4E);
            TEST_ASSERT_TRUE(TIM_ADV->CCER &
                             TIM_CCER_CC1E); // untouched by the channel-3 call

            // Channel 4 has no complementary output, so *_complementary* API should fail with it
            TEST_ASSERT_EQUAL(
                HAL_ERR_INVALID_ARG,
                pwm_pause_complementary_channel(TIM_ADV, TIMER_CHANNEL_4));
            TEST_ASSERT_EQUAL(
                HAL_ERR_INVALID_ARG,
                pwm_resume_complementary_channel(TIM_ADV, TIMER_CHANNEL_4));

            reset_pwm(TIM_ADV);
            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM_ADV, false));
        }

        void
        advanced_timer_init_rejects_reinit_once_locked_but_recovers_after_a_peripheral_reset() {
            // TIM1_BDTR's LOCK bits are write once until a reset in hardware. Software write can
            // clear them once set, including pwm_deinit()'s own BDTR clear, which is a no-op against
            // the hardware's frozen bits. The one recoverable path is a peripheral level reset via
            // RCC_APBxRSTR: toggling TIMxRST reinitializes every TIMx register, BDTR included,
            // without resetting the rest of the MCU. This is the only test in the suite that pokes
            // RCC reset directly, so it's kept last among the TIM1 tests even though it cleans up
            // fully after itself.
            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM_ADV, true));
            reset_pwm(TIM_ADV);

            // Reset TIM1 since the LOCK bits have been written to it in previous tests
            timer_reset(TIM1);

            pwm_advanced_timer_config_t config{};
            config.pwm_mode       = PWM_MODE_1;
            config.pwm_count_mode = OUTPUT_COMPARE_EDGE_LEFT_ALIGNED;
            config.wp_level       = WP_LOCK_LEVEL_3;
            config.channels[0] =
                make_channel(TIMER_CHANNEL_1, BOARD_TIM1_CH1_PA8);
            config.num_channels = 1;

            TEST_ASSERT_EQUAL(HAL_OK,
                              pwm_advanced_timer_init(TIM_ADV, &config));
            TEST_ASSERT_EQUAL_UINT32(std::to_underlying(WP_LOCK_LEVEL_3),
                                     (TIM_ADV->BDTR & TIM_BDTR_LOCK) >>
                                         TIM_BDTR_LOCK_Pos);

            // Once locked, reinit must refuse; regardless of what the new request asks for
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_STATE,
                              pwm_advanced_timer_init(TIM_ADV, &config));

            auto no_lock     = config;
            no_lock.wp_level = WP_OFF;
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_STATE,
                              pwm_advanced_timer_init(TIM_ADV, &no_lock));

            // pwm_timer_init shares the same lock check when called on an advanced timer
            pwm_gp_timer_config_t gp_config{};
            gp_config.pwm_mode       = PWM_MODE_1;
            gp_config.pwm_count_mode = OUTPUT_COMPARE_EDGE_LEFT_ALIGNED;
            gp_config.channels[0] =
                make_channel(TIMER_CHANNEL_1, BOARD_TIM1_CH1_PA8);
            gp_config.num_channels = 1;
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_STATE,
                              pwm_gp_timer_init(TIM_ADV, &gp_config));

            // Recover via a TIM1 local peripheral reset
            timer_reset(TIM1);

            TEST_ASSERT_EQUAL_UINT32(
                0, TIM_ADV->BDTR); // LOCK (and everything else) is clear again
            TEST_ASSERT_EQUAL(
                HAL_OK,
                pwm_advanced_timer_init(TIM_ADV, &no_lock)); // Usable again

            // Reset one final time
            reset_pwm(TIM_ADV);
            timer_reset(TIM1);

            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM_ADV, false));
        }


        // PWM input mode. Hardware loopback: TIM5's PWM output (PA0) feeds TIM3's channel 1 pin (PB4).

        namespace lb = test::loopback;

        constexpr uint32_t INPUT_TICK_HZ =
            1'000'000U; // 1us per timer tick in most of the tests below
        constexpr size_t INPUT_LOG_N = 16;

        struct input_log_t {
            volatile uint32_t events;
            volatile uint32_t stored;

            std::array<timer_channel_t, INPUT_LOG_N> channel;
            std::array<uint32_t, INPUT_LOG_N>        cycles;
        };

        input_log_t s_in_log;

        void reset_input_log() {
            s_in_log.events = 0;
            s_in_log.stored = 0;
        }

        void input_cb(void*, timer_channel_t channel, timer_count_dir_t) {
            const uint32_t i = s_in_log.stored;
            if (i < INPUT_LOG_N) {
                s_in_log.channel[i] = channel;
                s_in_log.cycles[i]  = DWT->CYCCNT;
                s_in_log.stored     = i + 1;
            }
            s_in_log.events = s_in_log.events + 1;
        }

        pwm_input_config_t make_input_config(uint32_t   prescaler,
                                             uint32_t   filter_ns = 0,
                                             timer_cb_t callback  = {}) {
            pwm_input_config_t cfg{};
            cfg.pull            = TIMER_PULL_NONE;
            cfg.clk_div         = TIM_CLK_DIV_1;
            cfg.count_mode      = TIMER_COUNTER_DIR_UP;
            cfg.timer_prescaler = static_cast<uint16_t>(prescaler);
            cfg.filter_ns       = filter_ns;
            cfg.gpio_pin        = lb::IN_A;
            cfg.callback        = callback;
            return cfg;
        }

        // Stop the measurement, then read the last captured period and pulse width. Stopping first means the read
        // can never race a capture
        pwm_input_data_t stop_and_read() {
            TEST_ASSERT_EQUAL(HAL_OK, pwm_input_stop(lb::MEAS));
            pwm_input_data_t data{};
            TEST_ASSERT_EQUAL(HAL_OK, pwm_input_get_data(lb::MEAS, &data));
            return data;
        }

        void input_init_rejects_invalid_arguments() {
            lb::reset_fixture();
            const auto good =
                make_input_config(lb::prescaler_for_1us(lb::MEAS));

            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG,
                              pwm_input_init(nullptr, &good));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG,
                              pwm_input_init(lb::MEAS, nullptr));

            auto bad            = good;
            bad.timer_prescaler = 0;
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG,
                              pwm_input_init(lb::MEAS, &bad));

            bad            = good;
            bad.count_mode = TIMER_COUNTER_DIR_NONE;
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG,
                              pwm_input_init(lb::MEAS, &bad));

            // Timers with a single channel can't do PWM input
            TEST_ASSERT_EQUAL(HAL_ERR_NOT_SUPPORTED,
                              pwm_input_init(TIM10, &good));
            TEST_ASSERT_EQUAL(HAL_ERR_NOT_SUPPORTED,
                              pwm_input_init(TIM11, &good));

            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM1, true));
            TEST_ASSERT_EQUAL(HAL_OK, timer_reset(TIM1));
            TIM1->BDTR |= TIM_BDTR_LOCK;
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_STATE,
                              pwm_input_init(TIM1, &good));
            TEST_ASSERT_EQUAL(HAL_OK, timer_reset(TIM1));
            TEST_ASSERT_EQUAL(HAL_OK, timx_clk_enable(TIM1, false));
        }

        void input_get_data_and_frequency_reject_invalid_arguments() {
            lb::reset_fixture();
            pwm_input_data_t data{};
            uint32_t         hz = 0;

            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG,
                              pwm_input_get_data(nullptr, &data));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG,
                              pwm_input_get_data(lb::MEAS, nullptr));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG,
                              pwm_input_get_timer_frequency(nullptr, &hz));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG,
                              pwm_input_get_timer_frequency(lb::MEAS, nullptr));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, pwm_input_start(nullptr));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, pwm_input_stop(nullptr));
        }

        void input_init_programs_slave_reset_mode_and_both_capture_channels() {
            lb::reset_fixture();

            auto cfg    = make_input_config(7, 0, {input_cb, nullptr});
            cfg.clk_div = TIM_CLK_DIV_2;
            TEST_ASSERT_EQUAL(HAL_OK, pwm_input_init(lb::MEAS, &cfg));

            TIM_TypeDef* const t = lb::MEAS;

            // Channel 1 captures TI1 (period), channel 2 captures the same TI1 indirectly (pulse width)
            TEST_ASSERT_EQUAL_UINT32(
                0b01U, (t->CCMR1 & TIM_CCMR1_CC1S) >> TIM_CCMR1_CC1S_Pos);
            TEST_ASSERT_EQUAL_UINT32(
                0b10U, (t->CCMR1 & TIM_CCMR1_CC2S) >> TIM_CCMR1_CC2S_Pos);
            TEST_ASSERT_EQUAL_UINT32(0U,
                                     t->CCMR1 &
                                         (TIM_CCMR1_IC1F | TIM_CCMR1_IC2F |
                                          TIM_CCMR1_IC1PSC | TIM_CCMR1_IC2PSC));

            // Channel 1 on the rising edge, channel 2 on the falling edge
            TEST_ASSERT_EQUAL_UINT32(
                TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC2P,
                t->CCER & (TIM_CCER_CC1E | TIM_CCER_CC1P | TIM_CCER_CC1NP |
                           TIM_CCER_CC2E | TIM_CCER_CC2P | TIM_CCER_CC2NP));

            // Triggered by the filtered TI1 (TI1FP1), and the trigger resets the counter
            TEST_ASSERT_EQUAL_UINT32(
                0b101U, (t->SMCR & TIM_SMCR_TS) >> TIM_SMCR_TS_Pos);
            TEST_ASSERT_EQUAL_UINT32(
                0b100U, (t->SMCR & TIM_SMCR_SMS) >> TIM_SMCR_SMS_Pos);

            TEST_ASSERT_EQUAL_UINT32(TIM_DIER_CC1IE | TIM_DIER_CC2IE,
                                     t->DIER &
                                         (TIM_DIER_CC1IE | TIM_DIER_CC2IE));
            TEST_ASSERT_TRUE(NVIC->ISER[TIM3_IRQn >> 5] &
                             (1UL << (TIM3_IRQn & 0x1FU)));

            TEST_ASSERT_EQUAL_UINT32(6U, t->PSC);
            TEST_ASSERT_EQUAL_UINT32(UINT16_MAX, t->ARR);
            TEST_ASSERT_EQUAL_UINT32(std::to_underlying(TIM_CLK_DIV_2),
                                     (t->CR1 & TIM_CR1_CKD) >> TIM_CR1_CKD_Pos);
            TEST_ASSERT_FALSE(t->CR1 & TIM_CR1_CEN);

            // Without a callback no capture interrupt is enabled
            cfg = make_input_config(7);
            TEST_ASSERT_EQUAL(HAL_OK, pwm_input_init(lb::MEAS, &cfg));
            TEST_ASSERT_EQUAL_UINT32(
                0U, t->DIER & (TIM_DIER_CC1IE | TIM_DIER_CC2IE));

            // The 32 bit timer gets the full reload value
            cfg          = make_input_config(1);
            cfg.gpio_pin = lb::GEN_A;
            TEST_ASSERT_EQUAL(HAL_OK, pwm_input_init(lb::GEN, &cfg));
            TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, lb::GEN->ARR);
        }

        void input_init_programs_the_requested_filter() {
            lb::reset_fixture();

            const auto cfg = make_input_config(1, 500);
            TEST_ASSERT_EQUAL(HAL_OK, pwm_input_init(lb::MEAS, &cfg));

            uint32_t code = 0;
            TEST_ASSERT_EQUAL(HAL_OK,
                              timer_filter_ns_to_ic_code(
                                  lb::MEAS, TIM_CLK_DIV_1, 500, &code));
            TEST_ASSERT_TRUE(code != 0U);
            TEST_ASSERT_EQUAL_UINT32(
                code, (lb::MEAS->CCMR1 & TIM_CCMR1_IC1F) >> TIM_CCMR1_IC1F_Pos);
            TEST_ASSERT_EQUAL_UINT32(
                code, (lb::MEAS->CCMR1 & TIM_CCMR1_IC2F) >> TIM_CCMR1_IC2F_Pos);
        }

        void input_init_configures_the_gpio_pin() {
            lb::reset_fixture();

            struct pull_case_t {
                timer_pull_t pull;
                uint32_t     pupdr;
            };
            constexpr std::array<pull_case_t, 3> pulls = {{
                {.pull = TIMER_PULL_NONE, .pupdr = 0b00},
                {.pull = TIMER_USE_PULLUP, .pupdr = 0b01},
                {.pull = TIMER_USE_PULLDOWN, .pupdr = 0b10},
            }};

            for (const auto& p : pulls) {
                auto cfg = make_input_config(1);
                cfg.pull = p.pull;
                TEST_ASSERT_EQUAL(HAL_OK, pwm_input_init(lb::MEAS, &cfg));
                TEST_ASSERT_EQUAL_UINT32(0b10U, lb::moder(lb::IN_A));
                TEST_ASSERT_EQUAL_UINT32(lb::IN_A.af, lb::afr(lb::IN_A));
                TEST_ASSERT_EQUAL_UINT32(p.pupdr, lb::pupdr(lb::IN_A));
            }
        }

        void input_init_replaces_the_output_configuration_of_the_same_timer() {
            lb::reset_fixture();

            // PWM output on TIM3 channel 1 leaves output compare mode bits in CCMR1. They overlay the input
            // capture filter and prescaler fields, so they must not survive into the input configuration
            pwm_gp_timer_config_t out{};
            out.pwm_mode       = PWM_MODE_1;
            out.pwm_count_mode = OUTPUT_COMPARE_EDGE_LEFT_ALIGNED;
            out.channels[0]    = make_channel(TIMER_CHANNEL_1, lb::IN_A);
            out.num_channels   = 1;
            TEST_ASSERT_EQUAL(HAL_OK, pwm_gp_timer_init(lb::MEAS, &out));
            TEST_ASSERT_TRUE(lb::MEAS->CCMR1 & TIM_CCMR1_OC1M);

            const auto cfg = make_input_config(1);
            TEST_ASSERT_EQUAL(HAL_OK, pwm_input_init(lb::MEAS, &cfg));
            TEST_ASSERT_EQUAL_UINT32(0U,
                                     lb::MEAS->CCMR1 &
                                         (TIM_CCMR1_IC1F | TIM_CCMR1_IC2F |
                                          TIM_CCMR1_IC1PSC | TIM_CCMR1_IC2PSC));
            TEST_ASSERT_EQUAL_UINT32(0b01U,
                                     (lb::MEAS->CCMR1 & TIM_CCMR1_CC1S) >>
                                         TIM_CCMR1_CC1S_Pos);
            TEST_ASSERT_FALSE(lb::MEAS->CCER & TIM_CCER_CC1P);
        }

        void input_timer_frequency_reports_the_tick_rate() {
            lb::reset_fixture();

            const uint32_t clk = timer_get_frequency_hz(lb::MEAS);
            const std::array<uint32_t, 4> prescalers = {
                1, 2, 4, clk / INPUT_TICK_HZ};

            for (const uint32_t prescaler : prescalers) {
                const auto cfg = make_input_config(prescaler);
                TEST_ASSERT_EQUAL(HAL_OK, pwm_input_init(lb::MEAS, &cfg));

                uint32_t hz = 0;
                TEST_ASSERT_EQUAL(HAL_OK,
                                  pwm_input_get_timer_frequency(lb::MEAS, &hz));
                TEST_ASSERT_EQUAL_UINT32(
                    clk / prescaler,
                    hz); // Ticks per second, not the timer's input clock
            }

            uint32_t   hz  = 0;
            const auto cfg = make_input_config(lb::prescaler_for_1us(lb::MEAS));
            TEST_ASSERT_EQUAL(HAL_OK, pwm_input_init(lb::MEAS, &cfg));
            TEST_ASSERT_EQUAL(HAL_OK,
                              pwm_input_get_timer_frequency(lb::MEAS, &hz));
            TEST_ASSERT_EQUAL_UINT32(INPUT_TICK_HZ, hz);
        }

        void input_start_and_stop_run_and_freeze_the_counter() {
            lb::reset_fixture();
            const auto cfg = make_input_config(lb::prescaler_for_1us(lb::MEAS));
            TEST_ASSERT_EQUAL(HAL_OK, pwm_input_init(lb::MEAS, &cfg));
            TEST_ASSERT_FALSE(lb::MEAS->CR1 & TIM_CR1_CEN);

            TEST_ASSERT_EQUAL(HAL_OK, pwm_input_start(lb::MEAS));
            TEST_ASSERT_TRUE(lb::MEAS->CR1 & TIM_CR1_CEN);
            delay_us(100);
            TEST_ASSERT_TRUE(lb::MEAS->CNT > 0U);

            TEST_ASSERT_EQUAL(HAL_OK, pwm_input_stop(lb::MEAS));
            TEST_ASSERT_FALSE(lb::MEAS->CR1 & TIM_CR1_CEN);
            const uint32_t held = lb::MEAS->CNT;
            delay_us(100);
            TEST_ASSERT_EQUAL_UINT32(held, lb::MEAS->CNT);
        }

        void input_measures_period_and_pulse_width_of_the_generated_pwm() {
            struct pwm_case_t {
                uint32_t hz;
                uint32_t duty_percent;
            };
            const std::array<pwm_case_t, 5> cases = {{
                {.hz = 1'000, .duty_percent = 25},
                {.hz = 1'000, .duty_percent = 75},
                {.hz = 5'000, .duty_percent = 50},
                {.hz = 200, .duty_percent = 10},
                {.hz = 20'000, .duty_percent = 50},
            }};

            for (const auto& c : cases) {
                lb::reset_fixture();
                uint32_t compare = 0;
                uint32_t max     = 0;
                TEST_ASSERT_EQUAL(HAL_OK,
                                  lb::generator_pwm_start(
                                      c.hz, c.duty_percent, &compare, &max));

                reset_input_log();
                const auto cfg = make_input_config(
                    lb::prescaler_for_1us(lb::MEAS), 0, {input_cb, nullptr});
                TEST_ASSERT_EQUAL(HAL_OK, pwm_input_init(lb::MEAS, &cfg));
                TEST_ASSERT_EQUAL(HAL_OK, pwm_input_start(lb::MEAS));

                // Six events is three full cycles: the first one after the start measures a partial cycle and is discarded
                TEST_ASSERT_TRUE_MESSAGE(lb::wait_for(
                                             []() {
                                                 return s_in_log.stored >= 6;
                                             },
                                             500),
                                         "No PWM input capture events");
                const auto data = stop_and_read();

                const uint64_t gen_clk = timer_get_frequency_hz(lb::GEN);
                const uint32_t expected_period =
                    (uint32_t)((uint64_t)max * INPUT_TICK_HZ / gen_clk);
                const uint32_t expected_pulse =
                    (uint32_t)((uint64_t)compare * INPUT_TICK_HZ / gen_clk);

                TEST_ASSERT_UINT32_WITHIN(2U, expected_period, data.period);
                TEST_ASSERT_UINT32_WITHIN(2U, expected_pulse, data.pulse);
            }
        }

        void
        input_capture_callbacks_alternate_between_rising_and_falling_edge_channels() {
            lb::reset_fixture();
            uint32_t compare = 0;
            uint32_t max     = 0;
            TEST_ASSERT_EQUAL(
                HAL_OK,
                lb::generator_pwm_start(
                    1'000, 25, &compare, &max)); // 250us high, 750us low

            reset_input_log();
            const auto cfg = make_input_config(
                lb::prescaler_for_1us(lb::MEAS), 0, {input_cb, nullptr});
            TEST_ASSERT_EQUAL(HAL_OK, pwm_input_init(lb::MEAS, &cfg));
            TEST_ASSERT_EQUAL(HAL_OK, pwm_input_start(lb::MEAS));
            TEST_ASSERT_TRUE(lb::wait_for(
                []() {
                    return s_in_log.stored >= 10;
                },
                500));
            TEST_ASSERT_EQUAL(HAL_OK, pwm_input_stop(lb::MEAS));

            for (uint32_t i = 1; i < 10; i++) {
                TEST_ASSERT_NOT_EQUAL(s_in_log.channel[i - 1],
                                      s_in_log.channel[i]);
            }

            // Channel 1 (rising edge) fires 250us before channel 2 (falling edge)
            const uint32_t sysclk = get_system_core_clock();
            for (uint32_t i = 0; i + 1 < 10; i++) {
                if (s_in_log.channel[i] == TIMER_CHANNEL_1) {
                    TEST_ASSERT_EQUAL(TIMER_CHANNEL_2, s_in_log.channel[i + 1]);
                    const uint32_t gap_us = lb::cycles_to_us32(
                        s_in_log.cycles[i + 1] - s_in_log.cycles[i], sysclk);
                    TEST_ASSERT_UINT32_WITHIN(25U, 250U, gap_us);
                }
            }
        }

        void input_get_data_is_self_consistent_over_many_reads() {
            lb::reset_fixture();
            uint32_t compare = 0;
            uint32_t max     = 0;
            TEST_ASSERT_EQUAL(
                HAL_OK, lb::generator_pwm_start(20'000, 50, &compare, &max));

            // Prescaler 1: full resolution, and a 50us period still fits the 16 bit counter
            const auto cfg = make_input_config(1);
            TEST_ASSERT_EQUAL(HAL_OK, pwm_input_init(lb::MEAS, &cfg));
            TEST_ASSERT_EQUAL(HAL_OK, pwm_input_start(lb::MEAS));
            delay_ms(5); // Plenty of complete cycles first

            const uint32_t expected_period =
                timer_get_frequency_hz(lb::MEAS) / 20'000U;
            const uint32_t expected_pulse = expected_period / 2U;

            // Reads landing at every phase of the cycle, including right on the capture edges
            for (uint32_t i = 0; i < 5'000; i++) {
                pwm_input_data_t data{};
                TEST_ASSERT_EQUAL(HAL_OK, pwm_input_get_data(lb::MEAS, &data));
                TEST_ASSERT_UINT32_WITHIN(2U, expected_period, data.period);
                TEST_ASSERT_UINT32_WITHIN(2U, expected_pulse, data.pulse);
                TEST_ASSERT_TRUE(data.pulse <= data.period);
            }
        }

        // PWM input against the CPU's cycle counter, across the supported system clocks. The generator's PWM
        // frequency and the measurer's tick rate both come from the timer clock arithmetic, so a wrong timer clock
        // would cancel out between them. The CPU's cycle counter doesn't share that arithmetic
        struct input_sweep_t {
            hal_err_t err;
            uint32_t  period_ticks;
            uint32_t  pulse_ticks;
            uint32_t  tick_hz;
            uint32_t  avg_period_us;
        };

        hal_err_t measure_input_at_current_clock(input_sweep_t* out) {
            TRY(lb::reset_fixture_hal());
            uint32_t compare = 0;
            uint32_t max     = 0;
            TRY(lb::generator_pwm_start(1'000, 50, &compare, &max));

            reset_input_log();
            const auto cfg = make_input_config(
                lb::prescaler_for_1us(lb::MEAS), 0, {input_cb, nullptr});
            TRY(pwm_input_init(lb::MEAS, &cfg));
            TRY(pwm_input_get_timer_frequency(lb::MEAS, &out->tick_hz));
            TRY(pwm_input_start(lb::MEAS));
            if (!lb::wait_for(
                    []() {
                        return s_in_log.stored >= INPUT_LOG_N;
                    },
                    100)) {
                return HAL_ERR_TIMEOUT;
            }
            TRY(pwm_input_stop(lb::MEAS));

            pwm_input_data_t data{};
            TRY(pwm_input_get_data(lb::MEAS, &data));
            out->period_ticks = data.period;
            out->pulse_ticks  = data.pulse;

            // Average spacing of the rising edge (channel 1) events by the cycle counter
            uint32_t first = 0;
            uint32_t last  = 0;
            uint32_t seen  = 0;
            for (uint32_t i = 0; i < INPUT_LOG_N; i++) {
                if (s_in_log.channel[i] == TIMER_CHANNEL_1) {
                    if (seen == 0) {
                        first = s_in_log.cycles[i];
                    }
                    last = s_in_log.cycles[i];
                    seen += 1;
                }
            }
            if (seen < 3) {
                return HAL_ERR_TIMEOUT;
            }
            out->avg_period_us = lb::cycles_to_us32(
                (last - first) / (seen - 1U), get_system_core_clock());
            return HAL_OK;
        }

        void
        input_period_agrees_with_the_cpu_cycle_counter_across_system_clocks() {
            const system_clock_t original = get_system_core_clock_type();

            std::array<input_sweep_t, lb::CLOCK_SWEEP.size()> results{};
            for (size_t i = 0; i < lb::CLOCK_SWEEP.size(); i++) {
                lb::switch_clock(lb::pick_preset(lb::CLOCK_SWEEP[i], original));
                results[i].err = measure_input_at_current_clock(&results[i]);
                (void)pwm_input_deinit(lb::MEAS);
                lb::generator_stop();
            }
            lb::switch_clock(
                original); // Back to the original clock before anything gets asserted
            lb::reset_fixture();

            for (const auto& r : results) {
                TEST_ASSERT_EQUAL(HAL_OK, r.err);
                TEST_ASSERT_EQUAL_UINT32(INPUT_TICK_HZ, r.tick_hz);
                TEST_ASSERT_UINT32_WITHIN(
                    2U, 1'000U, r.period_ticks); // 1 kHz at 1us ticks
                TEST_ASSERT_UINT32_WITHIN(2U, 500U, r.pulse_ticks);
                TEST_ASSERT_UINT32_WITHIN(10U, 1'000U, r.avg_period_us); // 1%
            }
        }

        void input_deinit_clears_the_slave_mode_and_stops_the_timer() {
            lb::reset_fixture();
            const auto cfg = make_input_config(
                lb::prescaler_for_1us(lb::MEAS), 0, {input_cb, nullptr});
            TEST_ASSERT_EQUAL(HAL_OK, pwm_input_init(lb::MEAS, &cfg));
            TEST_ASSERT_EQUAL(HAL_OK, pwm_input_start(lb::MEAS));

            TEST_ASSERT_EQUAL(HAL_OK, pwm_input_deinit(lb::MEAS));
            TEST_ASSERT_FALSE(lb::MEAS->CR1 & TIM_CR1_CEN);
            TEST_ASSERT_EQUAL_UINT32(
                0U,
                lb::MEAS->SMCR &
                    (TIM_SMCR_SMS |
                     TIM_SMCR_TS)); // A leftover reset mode would break the next user
            TEST_ASSERT_EQUAL_UINT32(
                0U,
                lb::MEAS->CCER &
                    (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC2P));
            TEST_ASSERT_EQUAL_UINT32(
                0U, lb::MEAS->CCMR1 & (TIM_CCMR1_CC1S | TIM_CCMR1_CC2S));
            TEST_ASSERT_EQUAL_UINT32(
                0U, lb::MEAS->DIER & (TIM_DIER_CC1IE | TIM_DIER_CC2IE));
            TEST_ASSERT_FALSE(NVIC->ISER[TIM3_IRQn >> 5] &
                              (1UL << (TIM3_IRQn & 0x1FU)));
        }

    } // namespace

    void all() {
        LOGI(TAG, "Starting the tests on the PWM driver");
        UNITY_BEGIN();

        RUN_TEST(timer_init_rejects_invalid_arguments);
        RUN_TEST(timer_init_rejects_too_many_channels_per_timer_class);
        RUN_TEST(timer_init_rejects_non_upcounting_mode_on_the_lite_timers);
        RUN_TEST(
            timer_init_rejects_an_unknown_channel_without_corrupting_state);
        RUN_TEST(timer_init_programs_each_main_channels_registers);
        RUN_TEST(timer_init_configures_the_gpio_pin);
        RUN_TEST(
            advanced_timer_init_rejects_invalid_arguments_and_non_advanced_timers);
        RUN_TEST(advanced_timer_init_programs_main_and_complementary_channels);
        RUN_TEST(advanced_timer_init_programs_ossr_ossi_repetition_and_clk_div);
        RUN_TEST(
            advanced_timer_init_dead_time_lands_in_the_correct_encoding_range);
        RUN_TEST(advanced_timer_init_dead_time_zero_leaves_dtg_untouched);
        RUN_TEST(
            advanced_timer_init_rejects_dead_time_beyond_the_max_encodable_range);
        RUN_TEST(
            advanced_timer_init_configures_the_break_input_and_fires_the_callback);
        RUN_TEST(advanced_timer_init_active_low_break_clears_bkp);
        RUN_TEST(start_rejects_invalid_arguments);
        RUN_TEST(start_rejects_a_frequency_above_the_timer_clock);
        RUN_TEST(
            start_edge_aligned_hits_the_requested_frequency_on_16_and_32_bit_timers);
        RUN_TEST(start_center_aligned_uses_the_doubled_period_formula);
        RUN_TEST(set_duty_cycle_rejects_invalid_arguments);
        RUN_TEST(set_duty_cycle_edge_aligned_boundary_is_arr_plus_one);
        RUN_TEST(set_duty_cycle_center_aligned_boundary_is_arr);
        RUN_TEST(set_duty_cycle_edge_aligned_off_by_one_at_the_32_bit_ceiling);
        RUN_TEST(set_duty_cycle_writes_the_correct_compare_register);
        RUN_TEST(freeze_and_unfreeze_toggle_cen_and_moe_on_the_advanced_timer);
        RUN_TEST(freeze_and_unfreeze_toggle_only_cen_on_general_purpose_timers);
        RUN_TEST(pause_and_resume_channel_reject_invalid_arguments);
        RUN_TEST(
            pause_and_resume_channel_toggle_only_the_main_output_on_general_purpose_timers);
        RUN_TEST(
            pause_and_resume_channel_also_toggle_the_complementary_output_on_the_advanced_timer);
        RUN_TEST(
            advanced_timer_init_rejects_reinit_once_locked_but_recovers_after_a_peripheral_reset);

        RUN_TEST(input_init_rejects_invalid_arguments);
        RUN_TEST(input_get_data_and_frequency_reject_invalid_arguments);
        RUN_TEST(
            input_init_programs_slave_reset_mode_and_both_capture_channels);
        RUN_TEST(input_init_programs_the_requested_filter);
        RUN_TEST(input_init_configures_the_gpio_pin);
        RUN_TEST(
            input_init_replaces_the_output_configuration_of_the_same_timer);
        RUN_TEST(input_timer_frequency_reports_the_tick_rate);
        RUN_TEST(input_start_and_stop_run_and_freeze_the_counter);
        RUN_TEST(input_measures_period_and_pulse_width_of_the_generated_pwm);
        RUN_TEST(
            input_capture_callbacks_alternate_between_rising_and_falling_edge_channels);
        RUN_TEST(input_get_data_is_self_consistent_over_many_reads);
        RUN_TEST(input_deinit_clears_the_slave_mode_and_stops_the_timer);
        RUN_TEST(
            input_period_agrees_with_the_cpu_cycle_counter_across_system_clocks);

        // Leave the loopback timers and clocks the way they were found
        TRY_THEN_FAIL(pwm_input_deinit(lb::MEAS));
        lb::generator_stop();
        TRY_THEN_FAIL(timx_clk_enable(lb::MEAS, false));
        TRY_THEN_FAIL(timx_clk_enable(lb::GEN, false));

        UNITY_END();
        LOGI(TAG, "Done with all tests on the PWM driver");
    }

} // namespace test::pwm
