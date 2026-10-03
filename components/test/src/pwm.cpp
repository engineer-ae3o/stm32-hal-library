#include "stm32f411xe.h"
#include "Unity/unity.h"

#include "drivers/timer_internals.h"
#include "drivers/gpio_types.h"
#include "drivers/pwm_types.h"
#include "drivers/timer.h"
#include "utils/common.h"
#include "utils/board.h"
#include "drivers/pwm.h"
#include "test/pwm.hpp"
#include "utils/err.h"
#include "utils/log.h"

#include <array>
#include <cstdint>
#include <utility>


namespace test::pwm {

    namespace {

        constexpr const char* TAG = "PWM_Test";

        TIM_TypeDef* const TIM_ADV   = TIM1;  // advanced, 16-bit
        TIM_TypeDef* const TIM_GP32  = TIM2;  // general purpose, 32-bit
        TIM_TypeDef* const TIM_GP16  = TIM3;  // general purpose, 16-bit
        TIM_TypeDef* const TIM_LITE2 = TIM9;  // lite, 2 channels
        TIM_TypeDef* const TIM_LITE1 = TIM10; // lite, 1 channel

        // Helpers
        inline void reset_pwm(TIM_TypeDef* handle) {
            TEST_ASSERT_EQUAL(HAL_OK, pwm_deinit(handle));
        }

        inline uint32_t get_gpio_moder(GPIO_TypeDef* port, gpio_pin_t pin) {
            return (port->MODER >> (std::to_underlying(pin) * 2U)) & 0b11U;
        }

        inline uint32_t get_gpio_afr(GPIO_TypeDef* port, gpio_pin_t pin) {
            const uint32_t p = std::to_underlying(pin);
            return (p < 8U) ? ((port->AFR[0] >> (p * 4U)) & 0xFU) : ((port->AFR[1] >> ((p - 8U) * 4U)) & 0xFU);
        }

        // Per-channel register layout. Mirrors the hardware's fixed CCMRx/CCER/CR2 bit
        // positions for each channel -- not a copy of the driver's logic, just the wiring
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

        channel_regs_t get_channel_regs(TIM_TypeDef* handle, pwm_channel_t channel) {
            switch (channel) {
                case PWM_CHANNEL_1:
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
                case PWM_CHANNEL_2:
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
                case PWM_CHANNEL_3:
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
                case PWM_CHANNEL_4:
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

        volatile uint32_t* get_ccr(TIM_TypeDef* handle, pwm_channel_t channel) {
            switch (channel) {
                case PWM_CHANNEL_1:
                    return &handle->CCR1;
                case PWM_CHANNEL_2:
                    return &handle->CCR2;
                case PWM_CHANNEL_3:
                    return &handle->CCR3;
                default:
                    return &handle->CCR4;
            }
        }

        // Independent decode of the DTG byte, from the reference manual's documented
        // encoding ranges -- not a mirror of pwm.c's encoder. Returns the dead time in ticks.
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
        void          brk_cb(void*) {
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

        constexpr pwm_channel_config_t make_channel(pwm_channel_t channel, board_pin_t pin, bool invert = false, bool idle = false) {
            return {.invert_output = invert, .output_idle_state = idle, .channel = channel, .gpio_pin = pin};
        }

        // TESTS

        void timer_init_rejects_invalid_arguments() {
            pwm_timer_config_t config{};
            config.pwm_mode       = PWM_MODE_1;
            config.pwm_count_mode = PWM_EDGE_ALIGNED_UPCOUNTING;
            config.channels[0]    = make_channel(PWM_CHANNEL_1, BOARD_TIM3_CH1_PA6);
            config.num_channels   = 1;

            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, pwm_timer_init(nullptr, &config));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, pwm_timer_init(TIM_GP16, nullptr));

            auto zero_channels         = config;
            zero_channels.num_channels = 0;
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, pwm_timer_init(TIM_GP16, &zero_channels));
        }

        void timer_init_rejects_too_many_channels_per_timer_class() {
            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM_GP16, true));
            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM_LITE2, true));
            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM_LITE1, true));

            pwm_timer_config_t config{};
            config.pwm_mode       = PWM_MODE_1;
            config.pwm_count_mode = PWM_EDGE_ALIGNED_UPCOUNTING;
            for (size_t i = 0; i < MAX_TIM2_CHANNELS; i++) {
                config.channels[i] = make_channel(static_cast<pwm_channel_t>(i), BOARD_TIM3_CH1_PA6);
            }

            config.num_channels = MAX_TIM1_CHANNELS + 1;
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, pwm_timer_init(TIM_GP16, &config));

            config.num_channels = MAX_TIM9_CHANNELS + 1;
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, pwm_timer_init(TIM_LITE2, &config));

            config.num_channels = MAX_TIM10_CHANNELS + 1;
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, pwm_timer_init(TIM_LITE1, &config));

            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM_GP16, false));
            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM_LITE2, false));
            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM_LITE1, false));
        }

        void timer_init_rejects_non_upcounting_mode_on_the_lite_timers() {
            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM_LITE1, true));

            pwm_timer_config_t config{};
            config.pwm_mode     = PWM_MODE_1;
            config.channels[0]  = make_channel(PWM_CHANNEL_1, BOARD_TIM10_CH1_PB8);
            config.num_channels = 1;

            for (const auto mode : {PWM_EDGE_ALIGNED_DOWNCOUNTING, PWM_CENTER_ALIGNED_MODE_1, PWM_CENTER_ALIGNED_MODE_2, PWM_CENTER_ALIGNED_MODE_3}) {
                config.pwm_count_mode = mode;
                TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, pwm_timer_init(TIM_LITE1, &config));
            }

            config.pwm_count_mode = PWM_EDGE_ALIGNED_UPCOUNTING;
            TEST_ASSERT_EQUAL(HAL_OK, pwm_timer_init(TIM_LITE1, &config));

            reset_pwm(TIM_LITE1);
            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM_LITE1, false));
        }

        void timer_init_rejects_an_unknown_channel_without_corrupting_state() {
            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM_GP16, true));
            reset_pwm(TIM_GP16);

            pwm_timer_config_t config{};
            config.pwm_mode       = PWM_MODE_1;
            config.pwm_count_mode = PWM_EDGE_ALIGNED_UPCOUNTING;
            config.channels[0]    = make_channel(static_cast<pwm_channel_t>(0xFF), BOARD_TIM3_CH1_PA6);
            config.num_channels   = 1;

            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, pwm_timer_init(TIM_GP16, &config));
            TEST_ASSERT_EQUAL_UINT32(0, TIM_GP16->CCMR1); // never written to, since the rejection happens before writeback

            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM_GP16, false));
        }

        void timer_init_programs_each_main_channels_registers() {
            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM_GP16, true));

            pwm_timer_config_t config{};
            config.pwm_mode       = PWM_MODE_2;
            config.pwm_count_mode = PWM_EDGE_ALIGNED_UPCOUNTING;

            config.channels[0]  = make_channel(PWM_CHANNEL_1, BOARD_TIM3_CH1_PA6, /*invert=*/false);
            config.channels[1]  = make_channel(PWM_CHANNEL_2, BOARD_TIM3_CH2_PA7, /*invert=*/true);
            config.channels[2]  = make_channel(PWM_CHANNEL_3, BOARD_TIM3_CH3_PB0, /*invert=*/false);
            config.channels[3]  = make_channel(PWM_CHANNEL_4, BOARD_TIM3_CH4_PB1, /*invert=*/true);
            config.num_channels = 4;

            TEST_ASSERT_EQUAL(HAL_OK, pwm_timer_init(TIM_GP16, &config));

            for (const auto& ch_cfg : config.channels) {
                const auto regs = get_channel_regs(TIM_GP16, ch_cfg.channel);
                TEST_ASSERT_EQUAL_UINT32(0, (*regs.ccmr >> regs.ccxs_pos) & 0b11U); // CCxS == 00: output
                TEST_ASSERT_TRUE(*regs.ccmr & regs.ocxpe);
                TEST_ASSERT_EQUAL_UINT32(std::to_underlying(PWM_MODE_2), (*regs.ccmr >> regs.ocxm_pos) & 0b111U);
                TEST_ASSERT_TRUE(TIM_GP16->CCER & regs.ccxe);
                TEST_ASSERT_EQUAL(ch_cfg.invert_output, static_cast<bool>(TIM_GP16->CCER & regs.ccxp));
            }

            TEST_ASSERT_TRUE(TIM_GP16->CR1 & TIM_CR1_ARPE);
            TEST_ASSERT_TRUE(TIM_GP16->CR1 & TIM_CR1_URS);
            TEST_ASSERT_EQUAL_UINT32(std::to_underlying(PWM_EDGE_ALIGNED_UPCOUNTING), TIM_GP16->CR1 & (TIM_CR1_CMS | TIM_CR1_DIR));

            reset_pwm(TIM_GP16);
            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM_GP16, false));
        }

        void timer_init_clears_residual_state_before_reconfiguring() {
            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM_GP16, true));
            reset_pwm(TIM_GP16);

            // Pollute with bits the next config won't set, to catch an |= where a plain = was needed
            TIM_GP16->CCMR1 = 0xFFFFU;
            TIM_GP16->CCER  = 0xFFFFU;
            TIM_GP16->CR1   = 0xFFFFU;

            pwm_timer_config_t config{};
            config.pwm_mode       = PWM_MODE_1;
            config.pwm_count_mode = PWM_EDGE_ALIGNED_UPCOUNTING;
            config.channels[0]    = make_channel(PWM_CHANNEL_1, BOARD_TIM3_CH1_PA6);
            config.num_channels   = 1;

            TEST_ASSERT_EQUAL(HAL_OK, pwm_timer_init(TIM_GP16, &config));

            // Only channel 0's bits and the expected CR1 bits should be set -- nothing left over
            TEST_ASSERT_EQUAL_UINT32(0, TIM_GP16->CCMR1 & (TIM_CCMR1_CC2S | TIM_CCMR1_OC2PE | TIM_CCMR1_OC2M));
            TEST_ASSERT_EQUAL_UINT32(0, TIM_GP16->CCER & ~(TIM_CCER_CC1E | TIM_CCER_CC1P));
            TEST_ASSERT_EQUAL_UINT32(0, TIM_GP16->CCMR2);

            reset_pwm(TIM_GP16);
            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM_GP16, false));
        }

        void timer_init_configures_the_gpio_pin() {
            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM_GP16, true));

            pwm_timer_config_t config{};
            config.pwm_mode       = PWM_MODE_1;
            config.pwm_count_mode = PWM_EDGE_ALIGNED_UPCOUNTING;
            config.channels[0]    = make_channel(PWM_CHANNEL_1, BOARD_TIM3_CH1_PA6);
            config.num_channels   = 1;

            TEST_ASSERT_EQUAL(HAL_OK, pwm_timer_init(TIM_GP16, &config));

            TEST_ASSERT_TRUE(RCC->AHB1ENR & RCC_AHB1ENR_GPIOAEN);
            TEST_ASSERT_EQUAL_UINT32(0b10U, get_gpio_moder(GPIOA, GPIO_PIN_6)); // alternate function mode
            TEST_ASSERT_EQUAL_UINT32(2U, get_gpio_afr(GPIOA, GPIO_PIN_6));      // AF2, per BOARD_TIM3_CH1_PA6

            reset_pwm(TIM_GP16);
            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM_GP16, false));
        }

        void advanced_timer_init_rejects_invalid_arguments_and_non_advanced_timers() {
            pwm_advanced_timer_config_t config{};
            config.pwm_mode       = PWM_MODE_1;
            config.pwm_count_mode = PWM_EDGE_ALIGNED_UPCOUNTING;
            config.wp_level       = WP_OFF; // never anything else -- see note above the test suite
            config.channels[0]    = make_channel(PWM_CHANNEL_1, BOARD_TIM1_CH1_PA8);
            config.num_channels   = 1;

            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, pwm_advanced_timer_init(nullptr, &config));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, pwm_advanced_timer_init(TIM_ADV, nullptr));

            auto zero_channels         = config;
            zero_channels.num_channels = 0;
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, pwm_advanced_timer_init(TIM_ADV, &zero_channels));

            auto too_many         = config;
            too_many.num_channels = MAX_TIM1_CHANNELS + 1;
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, pwm_advanced_timer_init(TIM_ADV, &too_many));

            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM_GP32, true));
            TEST_ASSERT_EQUAL(HAL_ERR_NOT_SUPPORTED, pwm_advanced_timer_init(TIM_GP32, &config)); // not an advanced timer
            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM_GP32, false));
        }

        void advanced_timer_init_programs_main_and_complementary_channels() {
            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM_ADV, true));

            pwm_advanced_timer_config_t config{};
            config.pwm_mode                   = PWM_MODE_1;
            config.pwm_count_mode             = PWM_EDGE_ALIGNED_UPCOUNTING;
            config.wp_level                   = WP_OFF;
            config.use_complementary_channels = true;

            config.channels[0]  = make_channel(PWM_CHANNEL_1, BOARD_TIM1_CH1_PA8, /*invert=*/false);
            config.channels[1]  = make_channel(PWM_CHANNEL_2, BOARD_TIM1_CH2_PA9, /*invert=*/true);
            config.channels[2]  = make_channel(PWM_CHANNEL_3, BOARD_TIM1_CH3_PA10, /*invert=*/false);
            config.channels[3]  = make_channel(PWM_CHANNEL_4, BOARD_TIM1_CH4_PA11, /*invert=*/true);
            config.num_channels = 4;

            config.complementary_channels[0] = {.invert_output = true, .output_idle_state = true, .gpio_pin = BOARD_TIM1_CH1N_PA7};
            config.complementary_channels[1] = {.invert_output = false, .output_idle_state = false, .gpio_pin = BOARD_TIM1_CH2N_PB0};
            config.complementary_channels[2] = {.invert_output = true, .output_idle_state = true, .gpio_pin = BOARD_TIM1_CH3N_PB1};

            TEST_ASSERT_EQUAL(HAL_OK, pwm_advanced_timer_init(TIM_ADV, &config));

            for (const auto& ch_cfg : config.channels) {
                const auto regs = get_channel_regs(TIM_ADV, ch_cfg.channel);
                TEST_ASSERT_TRUE(TIM_ADV->CCER & regs.ccxe);
                TEST_ASSERT_EQUAL(ch_cfg.invert_output, static_cast<bool>(TIM_ADV->CCER & regs.ccxp));
                TEST_ASSERT_EQUAL(ch_cfg.output_idle_state, static_cast<bool>(TIM_ADV->CR2 & regs.oisx));
            }

            // Channels 0-2 have complementary outputs
            for (size_t i = 0; i < 3; i++) {
                const auto  channel = static_cast<pwm_channel_t>(i);
                const auto  regs    = get_channel_regs(TIM_ADV, channel);
                const auto& comp    = config.complementary_channels[i];
                TEST_ASSERT_TRUE(TIM_ADV->CCER & regs.ccxne);
                TEST_ASSERT_EQUAL(comp.invert_output, static_cast<bool>(TIM_ADV->CCER & regs.ccxnp));
                TEST_ASSERT_EQUAL(comp.output_idle_state, static_cast<bool>(TIM_ADV->CR2 & regs.oisxn));
            }

            // Channel 3 has no complementary output: its N-bits must stay clear
            TEST_ASSERT_FALSE(TIM_ADV->CCER & TIM_CCER_CC4NP);

            reset_pwm(TIM_ADV);
            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM_ADV, false));
        }

        void advanced_timer_init_programs_ossr_ossi_repetition_and_clk_div() {
            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM_ADV, true));

            pwm_advanced_timer_config_t config{};
            config.pwm_mode       = PWM_MODE_1;
            config.pwm_count_mode = PWM_CENTER_ALIGNED_MODE_1;
            config.wp_level       = WP_OFF;
            config.ossr           = true;
            config.ossi           = true;
            config.clk_div        = TIM_CLK_DIV_4;
            config.repetition_cnt = 7;
            config.channels[0]    = make_channel(PWM_CHANNEL_1, BOARD_TIM1_CH1_PA8);
            config.num_channels   = 1;

            TEST_ASSERT_EQUAL(HAL_OK, pwm_advanced_timer_init(TIM_ADV, &config));

            TEST_ASSERT_TRUE(TIM_ADV->BDTR & TIM_BDTR_OSSR);
            TEST_ASSERT_TRUE(TIM_ADV->BDTR & TIM_BDTR_OSSI);
            TEST_ASSERT_EQUAL_UINT32(7, TIM_ADV->RCR & TIM_RCR_REP);
            TEST_ASSERT_EQUAL_UINT32(std::to_underlying(TIM_CLK_DIV_4), (TIM_ADV->CR1 & TIM_CR1_CKD) >> TIM_CR1_CKD_Pos);
            TEST_ASSERT_EQUAL_UINT32(std::to_underlying(PWM_CENTER_ALIGNED_MODE_1), TIM_ADV->CR1 & TIM_CR1_CMS);

            reset_pwm(TIM_ADV);
            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM_ADV, false));
        }

        void advanced_timer_init_dead_time_lands_in_the_correct_encoding_range() {
            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM_ADV, true));
            const uint32_t timer_freq_hz = timer_get_frequency_hz(TIM_ADV);

            struct range_t {
                uint32_t target_ticks;
                uint32_t step;
            };
            // One representative target per documented range, well clear of the boundaries
            constexpr std::array<range_t, 4> RANGES = {{
                {50, 1},   // Range 1
                {200, 2},  // Range 2
                {400, 8},  // Range 3
                {800, 16}, // Range 4
            }};

            for (const auto& range : RANGES) {
                reset_pwm(TIM_ADV);

                constexpr uint64_t DENOM        = 1'000'000'000ULL; // clk_div == TIM_CLK_DIV_1
                const uint32_t     dead_time_ns = (uint32_t)ceil_div_u64((uint64_t)range.target_ticks * DENOM, timer_freq_hz);

                pwm_advanced_timer_config_t config{};
                config.pwm_mode       = PWM_MODE_1;
                config.pwm_count_mode = PWM_EDGE_ALIGNED_UPCOUNTING;
                config.wp_level       = WP_OFF;
                config.clk_div        = TIM_CLK_DIV_1;
                config.dead_time_ns   = dead_time_ns;
                config.channels[0]    = make_channel(PWM_CHANNEL_1, BOARD_TIM1_CH1_PA8);
                config.num_channels   = 1;

                TEST_ASSERT_EQUAL(HAL_OK, pwm_advanced_timer_init(TIM_ADV, &config));

                const uint8_t  dtg            = (TIM_ADV->BDTR & TIM_BDTR_DTG) >> TIM_BDTR_DTG_Pos;
                const uint32_t expected_ticks = (uint32_t)ceil_div_u64((uint64_t)dead_time_ns * timer_freq_hz, DENOM);
                const uint32_t decoded_ticks  = decode_dtg_ticks(dtg);

                // Encoding rounds UP to the nearest representable value for its range's step size
                TEST_ASSERT_TRUE(decoded_ticks >= expected_ticks);
                TEST_ASSERT_TRUE(decoded_ticks < expected_ticks + range.step);
            }

            reset_pwm(TIM_ADV);
            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM_ADV, false));
        }

        void advanced_timer_init_dead_time_zero_leaves_dtg_untouched() {
            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM_ADV, true));
            reset_pwm(TIM_ADV);

            pwm_advanced_timer_config_t config{};
            config.pwm_mode       = PWM_MODE_1;
            config.pwm_count_mode = PWM_EDGE_ALIGNED_UPCOUNTING;
            config.wp_level       = WP_OFF;
            config.dead_time_ns   = 0;
            config.channels[0]    = make_channel(PWM_CHANNEL_1, BOARD_TIM1_CH1_PA8);
            config.num_channels   = 1;

            TEST_ASSERT_EQUAL(HAL_OK, pwm_advanced_timer_init(TIM_ADV, &config));
            TEST_ASSERT_EQUAL_UINT32(0, TIM_ADV->BDTR & TIM_BDTR_DTG);

            reset_pwm(TIM_ADV);
            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM_ADV, false));
        }

        void advanced_timer_init_rejects_dead_time_beyond_the_max_encodable_range() {
            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM_ADV, true));
            reset_pwm(TIM_ADV);
            const uint32_t timer_freq_hz = timer_get_frequency_hz(TIM_ADV);

            // One tick past range 4's ceiling of 1008
            constexpr uint64_t DENOM        = 1'000'000'000ULL;
            const uint32_t     dead_time_ns = (uint32_t)ceil_div_u64(1009ULL * DENOM, timer_freq_hz);

            pwm_advanced_timer_config_t config{};
            config.pwm_mode       = PWM_MODE_1;
            config.pwm_count_mode = PWM_EDGE_ALIGNED_UPCOUNTING;
            config.wp_level       = WP_OFF;
            config.clk_div        = TIM_CLK_DIV_1;
            config.dead_time_ns   = dead_time_ns;
            config.channels[0]    = make_channel(PWM_CHANNEL_1, BOARD_TIM1_CH1_PA8);
            config.num_channels   = 1;

            TEST_ASSERT_EQUAL(HAL_ERR_NOT_SUPPORTED, pwm_advanced_timer_init(TIM_ADV, &config));
            TEST_ASSERT_EQUAL_UINT32(0, TIM_ADV->BDTR); // rejected before BDTR is ever written

            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM_ADV, false));
        }

        void advanced_timer_init_configures_the_break_input_and_fires_the_callback() {
            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM_ADV, true));

            pwm_advanced_timer_config_t config{};
            config.pwm_mode       = PWM_MODE_1;
            config.pwm_count_mode = PWM_EDGE_ALIGNED_UPCOUNTING;
            config.wp_level       = WP_OFF;
            config.channels[0]    = make_channel(PWM_CHANNEL_1, BOARD_TIM1_CH1_PA8);
            config.num_channels   = 1;

            config.break_input.use_break_input = true;
            config.break_input.active_low      = false;
            config.break_input.auto_rearm      = true;
            config.break_input.gpio_pin        = BOARD_TIM1_BKIN_PA6;
            config.break_input.callback        = brk_cb;
            config.break_input.user            = nullptr;

            s_brk_fired = false;
            TEST_ASSERT_EQUAL(HAL_OK, pwm_advanced_timer_init(TIM_ADV, &config));

            TEST_ASSERT_TRUE(TIM_ADV->BDTR & TIM_BDTR_BKE);
            TEST_ASSERT_TRUE(TIM_ADV->BDTR & TIM_BDTR_BKP); // active_low == false -> BKP set (active high)
            TEST_ASSERT_TRUE(TIM_ADV->BDTR & TIM_BDTR_AOE);
            TEST_ASSERT_TRUE(TIM_ADV->DIER & TIM_DIER_BIE);
            TEST_ASSERT_TRUE(RCC->AHB1ENR & RCC_AHB1ENR_GPIOAEN);
            TEST_ASSERT_EQUAL_UINT32(1U, get_gpio_afr(GPIOA, GPIO_PIN_6)); // AF1, per BOARD_TIM1_BKIN_PA6

            // Force the break condition and the NVIC pend directly: there's no safe way to
            // drive the physical BKIN pin from here, so this exercises the ISR dispatch path
            // (TIM1_BRK_TIM9_IRQn -> timer_isr_helper -> s_advanced_timer_cb.break_input_cb),
            // not the analog/digital break detection itself.
            TIM_ADV->SR |= TIM_SR_BIF;
            NVIC_SetPendingIRQ(TIM1_BRK_TIM9_IRQn);

            TEST_ASSERT_TRUE_MESSAGE(wait_until([]() {
                                         return s_brk_fired;
                                     }),
                                     "Break event callback never fired");
            TEST_ASSERT_FALSE(TIM_ADV->SR & TIM_SR_BIF);

            reset_pwm(TIM_ADV);
            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM_ADV, false));
        }

        void advanced_timer_init_active_low_break_clears_bkp() {
            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM_ADV, true));

            pwm_advanced_timer_config_t config{};
            config.pwm_mode       = PWM_MODE_1;
            config.pwm_count_mode = PWM_EDGE_ALIGNED_UPCOUNTING;
            config.wp_level       = WP_OFF;
            config.channels[0]    = make_channel(PWM_CHANNEL_1, BOARD_TIM1_CH1_PA8);
            config.num_channels   = 1;

            config.break_input.use_break_input = true;
            config.break_input.active_low      = true;
            config.break_input.gpio_pin        = BOARD_TIM1_BKIN_PA6;
            config.break_input.callback        = brk_cb;

            TEST_ASSERT_EQUAL(HAL_OK, pwm_advanced_timer_init(TIM_ADV, &config));
            TEST_ASSERT_FALSE(TIM_ADV->BDTR & TIM_BDTR_BKP);

            reset_pwm(TIM_ADV);
            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM_ADV, false));
        }

        void deinit_forwards_the_invalid_handle_error() {
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, pwm_deinit(reinterpret_cast<TIM_TypeDef*>(1)));
        }

        void start_rejects_invalid_arguments() {
            auto*    bogus    = reinterpret_cast<TIM_TypeDef*>(1);
            uint32_t max_duty = 0;
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, pwm_start(nullptr, 1000, &max_duty));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, pwm_start(bogus, 0, &max_duty));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, pwm_start(bogus, 1000, nullptr));
        }

        void start_rejects_when_already_running() {
            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM_GP16, true));
            reset_pwm(TIM_GP16);
            TIM_GP16->CR1 |= TIM_CR1_CEN; // fake a running timer directly

            uint32_t max_duty = 0;
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_STATE, pwm_start(TIM_GP16, 1000, &max_duty));

            TIM_GP16->CR1 &= ~TIM_CR1_CEN;
            reset_pwm(TIM_GP16);
            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM_GP16, false));
        }

        void start_rejects_a_frequency_above_the_timer_clock() {
            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM_GP16, true));
            reset_pwm(TIM_GP16);
            const uint32_t timer_freq_hz = timer_get_frequency_hz(TIM_GP16);

            uint32_t max_duty = 0;
            TEST_ASSERT_EQUAL(HAL_ERR_NOT_SUPPORTED, pwm_start(TIM_GP16, timer_freq_hz + 1, &max_duty));

            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM_GP16, false));
        }

        void start_edge_aligned_hits_the_requested_frequency_on_16_and_32_bit_timers() {
            for (auto* const handle : {TIM_GP16, TIM_GP32}) {
                TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(handle, true));
                reset_pwm(handle);

                constexpr uint32_t REQUESTED_HZ = 1000;
                uint32_t           max_duty     = 0;
                TEST_ASSERT_EQUAL(HAL_OK, pwm_start(handle, REQUESTED_HZ, &max_duty));

                const uint32_t timer_freq_hz = timer_get_frequency_hz(handle);
                const uint64_t psc_plus_1    = (uint64_t)handle->PSC + 1;
                const uint64_t arr_plus_1    = (uint64_t)handle->ARR + 1;
                const uint64_t achieved_hz   = timer_freq_hz / (psc_plus_1 * arr_plus_1);

                const uint64_t tolerance = REQUESTED_HZ / 100 + 1;
                TEST_ASSERT_TRUE(achieved_hz >= REQUESTED_HZ - tolerance);
                TEST_ASSERT_TRUE(achieved_hz <= REQUESTED_HZ + tolerance);

                TEST_ASSERT_EQUAL_UINT32(handle->ARR + 1, max_duty); // edge aligned: max == ARR + 1

                TEST_ASSERT_TRUE(handle->CR1 & TIM_CR1_CEN);
                TEST_ASSERT_FALSE(handle->SR & TIM_SR_UIF);

                TEST_ASSERT_EQUAL(HAL_OK, pwm_freeze_timer(handle));
                reset_pwm(handle);
                TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(handle, false));
            }
        }

        void start_center_aligned_uses_the_doubled_period_formula() {
            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM_GP16, true));
            reset_pwm(TIM_GP16);
            TIM_GP16->CR1 |= std::to_underlying(PWM_CENTER_ALIGNED_MODE_1); // fake the mode directly; pwm_start only reads CMS

            constexpr uint32_t REQUESTED_HZ = 500;
            uint32_t           max_duty     = 0;
            TEST_ASSERT_EQUAL(HAL_OK, pwm_start(TIM_GP16, REQUESTED_HZ, &max_duty));

            const uint32_t timer_freq_hz = timer_get_frequency_hz(TIM_GP16);
            const uint64_t psc_plus_1    = (uint64_t)TIM_GP16->PSC + 1;
            const uint64_t arr           = TIM_GP16->ARR;
            const uint64_t achieved_hz   = timer_freq_hz / (2ULL * psc_plus_1 * arr);

            const uint64_t tolerance = REQUESTED_HZ / 100 + 1;
            TEST_ASSERT_TRUE(achieved_hz >= REQUESTED_HZ - tolerance);
            TEST_ASSERT_TRUE(achieved_hz <= REQUESTED_HZ + tolerance);

            TEST_ASSERT_EQUAL_UINT32(TIM_GP16->ARR, max_duty); // center aligned: max == ARR, no +1

            TEST_ASSERT_EQUAL(HAL_OK, pwm_freeze_timer(TIM_GP16));
            reset_pwm(TIM_GP16);
            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM_GP16, false));
        }

        void start_zeroes_all_four_compare_registers() {
            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM_GP16, true));
            reset_pwm(TIM_GP16);

            TIM_GP16->CCR1 = 111;
            TIM_GP16->CCR2 = 222;
            TIM_GP16->CCR3 = 333;
            TIM_GP16->CCR4 = 444;

            uint32_t max_duty = 0;
            TEST_ASSERT_EQUAL(HAL_OK, pwm_start(TIM_GP16, 1000, &max_duty));

            TEST_ASSERT_EQUAL_UINT32(0, TIM_GP16->CCR1);
            TEST_ASSERT_EQUAL_UINT32(0, TIM_GP16->CCR2);
            TEST_ASSERT_EQUAL_UINT32(0, TIM_GP16->CCR3);
            TEST_ASSERT_EQUAL_UINT32(0, TIM_GP16->CCR4);

            TEST_ASSERT_EQUAL(HAL_OK, pwm_freeze_timer(TIM_GP16));
            reset_pwm(TIM_GP16);
            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM_GP16, false));
        }

        void set_duty_cycle_rejects_invalid_arguments() {
            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM_GP16, true));
            reset_pwm(TIM_GP16);
            TIM_GP16->ARR = 999; // edge aligned by default (CMS == 0): max duty == 1000

            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, pwm_set_duty_cycle(nullptr, PWM_CHANNEL_1, 500));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, pwm_set_duty_cycle(TIM_GP16, PWM_CHANNEL_1, 1001));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, pwm_set_duty_cycle(TIM_GP16, static_cast<pwm_channel_t>(0xFF), 500));

            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM_GP16, false));
        }

        void set_duty_cycle_edge_aligned_boundary_is_arr_plus_one() {
            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM_GP16, true));
            reset_pwm(TIM_GP16);
            TIM_GP16->ARR = 999; // CMS == 0: edge aligned, max == 1000

            TEST_ASSERT_EQUAL(HAL_OK, pwm_set_duty_cycle(TIM_GP16, PWM_CHANNEL_1, 1000));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, pwm_set_duty_cycle(TIM_GP16, PWM_CHANNEL_1, 1001));

            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM_GP16, false));
        }

        void set_duty_cycle_center_aligned_boundary_is_arr() {
            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM_GP16, true));
            reset_pwm(TIM_GP16);
            TIM_GP16->ARR = 999;
            TIM_GP16->CR1 |= std::to_underlying(PWM_CENTER_ALIGNED_MODE_1); // CMS != 0: center aligned, max == 999

            TEST_ASSERT_EQUAL(HAL_OK, pwm_set_duty_cycle(TIM_GP16, PWM_CHANNEL_1, 999));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, pwm_set_duty_cycle(TIM_GP16, PWM_CHANNEL_1, 1000));

            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM_GP16, false));
        }

        void set_duty_cycle_edge_aligned_off_by_one_at_the_32_bit_ceiling() {
            // On a 32-bit timer with ARR at its absolute max, edge-aligned max duty is clamped
            // to ARR itself (not ARR + 1, which would wrap to 0). See get_max_duty_cycle's comment.
            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM_GP32, true));
            reset_pwm(TIM_GP32);
            TIM_GP32->ARR = UINT32_MAX;

            TEST_ASSERT_EQUAL(HAL_OK, pwm_set_duty_cycle(TIM_GP32, PWM_CHANNEL_1, UINT32_MAX));

            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM_GP32, false));
        }

        void set_duty_cycle_writes_the_correct_compare_register() {
            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM_GP16, true));
            reset_pwm(TIM_GP16);
            TIM_GP16->ARR = 999;

            for (const auto channel : {PWM_CHANNEL_1, PWM_CHANNEL_2, PWM_CHANNEL_3, PWM_CHANNEL_4}) {
                TEST_ASSERT_EQUAL(HAL_OK, pwm_set_duty_cycle(TIM_GP16, channel, 250));
                TEST_ASSERT_EQUAL_UINT32(250, *get_ccr(TIM_GP16, channel));
            }

            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM_GP16, false));
        }

        void freeze_and_unfreeze_reject_invalid_arguments_and_states() {
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, pwm_freeze_timer(nullptr));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, pwm_unfreeze_timer(nullptr));

            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM_GP16, true));
            reset_pwm(TIM_GP16);

            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_STATE, pwm_freeze_timer(TIM_GP16)); // not running yet

            TIM_GP16->CR1 |= TIM_CR1_CEN;
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_STATE, pwm_unfreeze_timer(TIM_GP16)); // already running

            TIM_GP16->CR1 &= ~TIM_CR1_CEN;
            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM_GP16, false));
        }

        void freeze_and_unfreeze_toggle_cen_and_moe_on_the_advanced_timer() {
            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM_ADV, true));
            reset_pwm(TIM_ADV);

            TEST_ASSERT_EQUAL(HAL_OK, pwm_unfreeze_timer(TIM_ADV));
            TEST_ASSERT_TRUE(TIM_ADV->CR1 & TIM_CR1_CEN);
            TEST_ASSERT_TRUE(TIM_ADV->BDTR & TIM_BDTR_MOE);

            TEST_ASSERT_EQUAL(HAL_OK, pwm_freeze_timer(TIM_ADV));
            TEST_ASSERT_FALSE(TIM_ADV->CR1 & TIM_CR1_CEN);
            TEST_ASSERT_FALSE(TIM_ADV->BDTR & TIM_BDTR_MOE);

            reset_pwm(TIM_ADV);
            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM_ADV, false));
        }

        void freeze_and_unfreeze_toggle_only_cen_on_general_purpose_timers() {
            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM_GP16, true));
            reset_pwm(TIM_GP16);

            TEST_ASSERT_EQUAL(HAL_OK, pwm_unfreeze_timer(TIM_GP16));
            TEST_ASSERT_TRUE(TIM_GP16->CR1 & TIM_CR1_CEN);

            TEST_ASSERT_EQUAL(HAL_OK, pwm_freeze_timer(TIM_GP16));
            TEST_ASSERT_FALSE(TIM_GP16->CR1 & TIM_CR1_CEN);

            reset_pwm(TIM_GP16);
            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM_GP16, false));
        }

        void pause_and_resume_channel_reject_invalid_arguments() {
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, pwm_pause_channel(nullptr, PWM_CHANNEL_1));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, pwm_resume_channel(nullptr, PWM_CHANNEL_1));

            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM_GP16, true));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, pwm_pause_channel(TIM_GP16, static_cast<pwm_channel_t>(4)));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, pwm_resume_channel(TIM_GP16, static_cast<pwm_channel_t>(4)));
            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM_GP16, false));
        }

        void pause_and_resume_channel_toggle_only_the_main_output_on_general_purpose_timers() {
            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM_GP16, true));
            reset_pwm(TIM_GP16);
            TIM_GP16->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;

            for (const auto channel : {PWM_CHANNEL_1, PWM_CHANNEL_2, PWM_CHANNEL_3, PWM_CHANNEL_4}) {
                const auto regs = get_channel_regs(TIM_GP16, channel);
                TEST_ASSERT_EQUAL(HAL_OK, pwm_pause_channel(TIM_GP16, channel));
                TEST_ASSERT_FALSE(TIM_GP16->CCER & regs.ccxe);

                TEST_ASSERT_EQUAL(HAL_OK, pwm_resume_channel(TIM_GP16, channel));
                TEST_ASSERT_TRUE(TIM_GP16->CCER & regs.ccxe);
            }

            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM_GP16, false));
        }

        void pause_and_resume_channel_also_toggle_the_complementary_output_on_the_advanced_timer() {
            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM_ADV, true));
            reset_pwm(TIM_ADV);
            TIM_ADV->CCER = TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC4E;

            TEST_ASSERT_EQUAL(HAL_OK, pwm_pause_channel(TIM_ADV, PWM_CHANNEL_1));
            TEST_ASSERT_FALSE(TIM_ADV->CCER & (TIM_CCER_CC1E | TIM_CCER_CC1NE));

            TEST_ASSERT_EQUAL(HAL_OK, pwm_resume_channel(TIM_ADV, PWM_CHANNEL_1));
            TEST_ASSERT_TRUE(TIM_ADV->CCER & (TIM_CCER_CC1E | TIM_CCER_CC1NE));

            // Channel 3 has no complementary output: pausing it must not touch bits outside CC4E
            TEST_ASSERT_EQUAL(HAL_OK, pwm_pause_channel(TIM_ADV, PWM_CHANNEL_4));
            TEST_ASSERT_FALSE(TIM_ADV->CCER & TIM_CCER_CC4E);
            TEST_ASSERT_TRUE(TIM_ADV->CCER & TIM_CCER_CC1E); // untouched by the channel-3 call

            reset_pwm(TIM_ADV);
            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM_ADV, false));
        }

        void advanced_timer_init_rejects_reinit_once_locked_but_recovers_after_a_peripheral_reset() {
            // TIM1_BDTR's LOCK bits are write-once-until-reset in hardware -- no software write can
            // clear them once set, including pwm_deinit()'s own BDTR clear, which is a no-op against
            // hardware-frozen bits. The one recoverable path is a peripheral-level reset via
            // RCC_APB2RSTR: toggling TIM1RST reinitializes every TIM1 register, BDTR included,
            // without resetting the rest of the MCU. This is the only test in the suite that pokes
            // RCC reset directly, so it's kept last among the TIM1 tests even though it cleans up
            // fully after itself.
            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM_ADV, true));
            reset_pwm(TIM_ADV);

            pwm_advanced_timer_config_t config{};
            config.pwm_mode       = PWM_MODE_1;
            config.pwm_count_mode = PWM_EDGE_ALIGNED_UPCOUNTING;
            config.wp_level       = WP_LOCK_LEVEL_1;
            config.channels[0]    = make_channel(PWM_CHANNEL_1, BOARD_TIM1_CH1_PA8);
            config.num_channels   = 1;

            TEST_ASSERT_EQUAL(HAL_OK, pwm_advanced_timer_init(TIM_ADV, &config));
            TEST_ASSERT_EQUAL_UINT32(std::to_underlying(WP_LOCK_LEVEL_1), (TIM_ADV->BDTR & TIM_BDTR_LOCK) >> TIM_BDTR_LOCK_Pos);

            // Once locked, reinit must refuse -- regardless of what the new request asks for
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_STATE, pwm_advanced_timer_init(TIM_ADV, &config));

            auto no_lock     = config;
            no_lock.wp_level = WP_OFF;
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_STATE, pwm_advanced_timer_init(TIM_ADV, &no_lock));

            // pwm_timer_init shares the same lock check when called on an advanced timer
            pwm_timer_config_t gp_config{};
            gp_config.pwm_mode       = PWM_MODE_1;
            gp_config.pwm_count_mode = PWM_EDGE_ALIGNED_UPCOUNTING;
            gp_config.channels[0]    = make_channel(PWM_CHANNEL_1, BOARD_TIM1_CH1_PA8);
            gp_config.num_channels   = 1;
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_STATE, pwm_timer_init(TIM_ADV, &gp_config));

            // Recover via a TIM1-local peripheral reset
            RCC->APB2RSTR |= RCC_APB2RSTR_TIM1RST;
            RCC->APB2RSTR &= ~RCC_APB2RSTR_TIM1RST;

            TEST_ASSERT_EQUAL_UINT32(0, TIM_ADV->BDTR);                            // LOCK (and everything else) is clear again
            TEST_ASSERT_EQUAL(HAL_OK, pwm_advanced_timer_init(TIM_ADV, &no_lock)); // usable again

            reset_pwm(TIM_ADV);
            TEST_ASSERT_EQUAL(HAL_OK, timer_clock_enable(TIM_ADV, false));
        }

    } // namespace

    void all() {
        LOGI(TAG, "Starting the tests on the PWM driver");
        UNITY_BEGIN();

        RUN_TEST(timer_init_rejects_invalid_arguments);
        RUN_TEST(timer_init_rejects_too_many_channels_per_timer_class);
        RUN_TEST(timer_init_rejects_non_upcounting_mode_on_the_lite_timers);
        RUN_TEST(timer_init_rejects_an_unknown_channel_without_corrupting_state);
        RUN_TEST(timer_init_programs_each_main_channels_registers);
        RUN_TEST(timer_init_clears_residual_state_before_reconfiguring);
        RUN_TEST(timer_init_configures_the_gpio_pin);

        RUN_TEST(advanced_timer_init_rejects_invalid_arguments_and_non_advanced_timers);
        RUN_TEST(advanced_timer_init_programs_main_and_complementary_channels);
        RUN_TEST(advanced_timer_init_programs_ossr_ossi_repetition_and_clk_div);
        RUN_TEST(advanced_timer_init_dead_time_lands_in_the_correct_encoding_range);
        RUN_TEST(advanced_timer_init_dead_time_zero_leaves_dtg_untouched);
        RUN_TEST(advanced_timer_init_rejects_dead_time_beyond_the_max_encodable_range);
        RUN_TEST(advanced_timer_init_configures_the_break_input_and_fires_the_callback);
        RUN_TEST(advanced_timer_init_active_low_break_clears_bkp);

        RUN_TEST(deinit_forwards_the_invalid_handle_error);

        RUN_TEST(start_rejects_invalid_arguments);
        RUN_TEST(start_rejects_when_already_running);
        RUN_TEST(start_rejects_a_frequency_above_the_timer_clock);
        RUN_TEST(start_edge_aligned_hits_the_requested_frequency_on_16_and_32_bit_timers);
        RUN_TEST(start_center_aligned_uses_the_doubled_period_formula);
        RUN_TEST(start_zeroes_all_four_compare_registers);

        RUN_TEST(set_duty_cycle_rejects_invalid_arguments);
        RUN_TEST(set_duty_cycle_edge_aligned_boundary_is_arr_plus_one);
        RUN_TEST(set_duty_cycle_center_aligned_boundary_is_arr);
        RUN_TEST(set_duty_cycle_edge_aligned_off_by_one_at_the_32_bit_ceiling);
        RUN_TEST(set_duty_cycle_writes_the_correct_compare_register);

        RUN_TEST(freeze_and_unfreeze_reject_invalid_arguments_and_states);
        RUN_TEST(freeze_and_unfreeze_toggle_cen_and_moe_on_the_advanced_timer);
        RUN_TEST(freeze_and_unfreeze_toggle_only_cen_on_general_purpose_timers);

        RUN_TEST(pause_and_resume_channel_reject_invalid_arguments);
        RUN_TEST(pause_and_resume_channel_toggle_only_the_main_output_on_general_purpose_timers);
        RUN_TEST(pause_and_resume_channel_also_toggle_the_complementary_output_on_the_advanced_timer);

        RUN_TEST(advanced_timer_init_rejects_reinit_once_locked_but_recovers_after_a_peripheral_reset);

        UNITY_END();
        LOGI(TAG, "Done with all tests on the PWM driver");
    }

} // namespace test::pwm
