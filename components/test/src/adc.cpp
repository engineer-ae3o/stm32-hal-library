#include "stm32f411xe.h"
#include "Unity/unity.h"

#include "drivers/adc_types.h"
#include "utils/common.h"
#include "drivers/adc.h"
#include "utils/board.h"
#include "utils/tick.h"
#include "utils/err.h"
#include "utils/log.h"

#include <array>
#include <cstdint>


namespace test::adc {

    namespace {

        constexpr const char* TAG = "ADC_Test";

        constexpr uint32_t BOUNDED_WAIT_ITERS = 10U * TIMEOUT_CYCLES;

        // Bring ADC1 to a known-good, powered, 12-bit right-aligned baseline before each test
        void reset_to_baseline() {
            adc_deconfigure(ADC1);
            adc_clk_configure(ADC_CLK_PRESCALER_4);

            const adc_config_t config{
                .alignment       = ADC_RIGHT_ALIGN,
                .resolution      = ADC_RES_12_BITS,
                .sampling_cycles = ADC_SAMPLE_28_CYCLES,
            };
            adc_configure(ADC1, &config);
        }

        volatile bool s_injected_done = false;
        void          injected_done_cb(void* arg) {
            UNUSED(arg);
            s_injected_done = true;
        }

        volatile bool s_cont_done       = false;
        volatile bool s_cont_buf_1_used = false;
        void          cont_done_cb(void* arg, bool is_buf_1_in_use) {
            UNUSED(arg);
            s_cont_buf_1_used = is_buf_1_in_use;
            s_cont_done       = true;
        }

        volatile bool s_wdg_triggered = false;
        void          wdg_cb(void* arg) {
            UNUSED(arg);
            s_wdg_triggered = true;
        }

        template<typename Predicate>
        bool wait_until(Predicate pred) {
            uint32_t timeout = BOUNDED_WAIT_ITERS;
            while (!pred() && --timeout) {
            }
            return pred();
        }

    } // namespace

    void clk_enable_rejects_unknown_handles() {
        TEST_ASSERT_EQUAL(HAL_OK, adcx_clk_enable(ADC1, true));
        TEST_ASSERT_TRUE(RCC->APB2ENR & RCC_APB2ENR_ADC1EN);

        const uint32_t before = RCC->APB2ENR;
        TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, adcx_clk_enable(nullptr, true));
        TEST_ASSERT_EQUAL_UINT32(before, RCC->APB2ENR);

        TEST_ASSERT_EQUAL(HAL_OK, adcx_clk_enable(ADC1, true)); // leave it enabled for every other test
    }

    void power_on_toggles_adon() {
        TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, adc_power_on(nullptr, true));

        TEST_ASSERT_EQUAL(HAL_OK, adc_power_on(ADC1, true));
        TEST_ASSERT_TRUE(ADC1->CR2 & ADC_CR2_ADON);

        TEST_ASSERT_EQUAL(HAL_OK, adc_power_on(ADC1, false));
        TEST_ASSERT_FALSE(ADC1->CR2 & ADC_CR2_ADON);

        adc_power_on(ADC1, true);
    }

    void configure_covers_every_alignment_resolution_and_sample_time() {
        TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, adc_configure(nullptr, nullptr));

        constexpr std::array<adc_resolution_t, 4>    RESOLUTIONS{ADC_RES_6_BITS, ADC_RES_8_BITS, ADC_RES_10_BITS, ADC_RES_12_BITS};
        constexpr std::array<adc_sample_cycles_t, 8> SAMPLE_TIMES{
            ADC_SAMPLE_3_CYCLES,
            ADC_SAMPLE_15_CYCLES,
            ADC_SAMPLE_28_CYCLES,
            ADC_SAMPLE_56_CYCLES,
            ADC_SAMPLE_84_CYCLES,
            ADC_SAMPLE_112_CYCLES,
            ADC_SAMPLE_144_CYCLES,
            ADC_SAMPLE_480_CYCLES,
        };

        for (const auto res : RESOLUTIONS) {
            for (const auto time : SAMPLE_TIMES) {
                const adc_config_t config{.alignment = ADC_LEFT_ALIGN, .resolution = res, .sampling_cycles = time};
                TEST_ASSERT_EQUAL(HAL_OK, adc_configure(ADC1, &config));

                TEST_ASSERT_TRUE(ADC1->CR2 & ADC_CR2_ALIGN);
                TEST_ASSERT_EQUAL_UINT32(static_cast<uint32_t>(res), (ADC1->CR1 & ADC_CR1_RES) >> ADC_CR1_RES_Pos);

                // Every external channel must carry the same sampling time
                TEST_ASSERT_EQUAL_UINT32(static_cast<uint32_t>(time), (ADC1->SMPR1 >> ADC_SMPR1_SMP10_Pos) & 0b111UL);
                TEST_ASSERT_EQUAL_UINT32(static_cast<uint32_t>(time), (ADC1->SMPR1 >> ADC_SMPR1_SMP15_Pos) & 0b111UL);
                TEST_ASSERT_EQUAL_UINT32(static_cast<uint32_t>(time), (ADC1->SMPR2 >> ADC_SMPR2_SMP0_Pos) & 0b111UL);
                TEST_ASSERT_EQUAL_UINT32(static_cast<uint32_t>(time), (ADC1->SMPR2 >> ADC_SMPR2_SMP9_Pos) & 0b111UL);
            }
        }

        const adc_config_t right_align_config{.alignment = ADC_RIGHT_ALIGN, .resolution = ADC_RES_12_BITS, .sampling_cycles = ADC_SAMPLE_28_CYCLES};
        TEST_ASSERT_EQUAL(HAL_OK, adc_configure(ADC1, &right_align_config));
        TEST_ASSERT_FALSE(ADC1->CR2 & ADC_CR2_ALIGN);

        reset_to_baseline();
    }

    void deconfigure_powers_down_and_clears_state() {
        TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, adc_deconfigure(nullptr));

        TEST_ASSERT_EQUAL(HAL_OK, adc_deconfigure(ADC1));
        TEST_ASSERT_FALSE(ADC1->CR2 & ADC_CR2_ADON);
        TEST_ASSERT_EQUAL_UINT32(0, ADC1->CR1 & ADC_CR1_RES);
        TEST_ASSERT_EQUAL_UINT32(0, ADC1->SMPR1);
        TEST_ASSERT_EQUAL_UINT32(0, ADC1->SMPR2);

        reset_to_baseline();
    }

    void clk_configure_sweeps_every_prescaler() {
        constexpr std::array<adc_prescaler_t, 4> PRESCALERS{ADC_CLK_PRESCALER_2, ADC_CLK_PRESCALER_4, ADC_CLK_PRESCALER_6, ADC_CLK_PRESCALER_8};
        for (const auto presc : PRESCALERS) {
            adc_clk_configure(presc);
            TEST_ASSERT_EQUAL_UINT32(static_cast<uint32_t>(presc), (ADC->CCR & ADC_CCR_ADCPRE) >> ADC_CCR_ADCPRE_Pos);
        }
        adc_clk_configure(ADC_CLK_PRESCALER_4);
    }

    void nvic_irq_enable_toggles_the_adc_line() {
        auto irq_is_enabled = []() {
            return (NVIC->ISER[static_cast<uint32_t>(ADC_IRQn) >> 5] & (1UL << (static_cast<uint32_t>(ADC_IRQn) & 0x1FU))) != 0;
        };

        adc_enable_nvic_irq(true);
        TEST_ASSERT_TRUE(irq_is_enabled());

        adc_enable_nvic_irq(false);
        TEST_ASSERT_FALSE(irq_is_enabled());
    }

    void temp_sensor_power_toggles_tsvrefe() {
        adc_power_on_temp_sensor(true);
        TEST_ASSERT_TRUE(ADC->CCR & ADC_CCR_TSVREFE);

        adc_power_on_temp_sensor(false);
        TEST_ASSERT_FALSE(ADC->CCR & ADC_CCR_TSVREFE);
    }

    void oneshot_regular_group_completes_within_12_bit_range() {
        reset_to_baseline();

        uint16_t raw = 0;
        TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, adc_regular_group_get_oneshot(nullptr, ADC_CHANNEL_0, &raw));
        TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, adc_regular_group_get_oneshot(ADC1, ADC_CHANNEL_0, nullptr));

        TEST_ASSERT_EQUAL(HAL_OK, adc_regular_group_get_oneshot(ADC1, ADC_CHANNEL_0, &raw));
        TEST_ASSERT_TRUE(raw <= 0x0FFFU);
    }

    void internal_channel_readings_complete_and_manage_their_own_enables() {
        reset_to_baseline();

        uint16_t raw = 0;

        TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, adc_get_v_bat(nullptr, &raw));
        TEST_ASSERT_EQUAL(HAL_OK, adc_get_v_bat(ADC1, &raw));
        TEST_ASSERT_TRUE(raw <= (0x0FFFU * VBAT_DIVIDER_RATIO));
        // VBATE must be switched back off once the reading is done
        TEST_ASSERT_FALSE(ADC->CCR & ADC_CCR_VBATE);

        TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, adc_get_temperature(nullptr, &raw));
        TEST_ASSERT_EQUAL(HAL_OK, adc_get_temperature(ADC1, &raw));
        TEST_ASSERT_TRUE(raw <= 0x0FFFU);
        // The temperature sensor/VREFINT stay latched on afterwards - they share the TSVREFE bit
        TEST_ASSERT_TRUE(ADC->CCR & ADC_CCR_TSVREFE);

        TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, adc_get_v_ref_internal(nullptr, &raw));
        TEST_ASSERT_EQUAL(HAL_OK, adc_get_v_ref_internal(ADC1, &raw));
        TEST_ASSERT_TRUE(raw <= 0x0FFFU);

        adc_power_on_temp_sensor(false);
    }

    // Small helper to keep the invalid-resolution/null-arg checks above self-contained
    float& voltage_dummy_unused_guard() {
        static float dummy = 0.0F;
        return dummy;
    }

    void voltage_and_temperature_math_are_internally_consistent() {
        reset_to_baseline();

        float vdda = 0.0F;
        TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, adc_get_vdda(nullptr, &vdda));
        TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, adc_get_vdda(ADC1, nullptr));
        TEST_ASSERT_EQUAL(HAL_OK, adc_get_vdda(ADC1, &vdda));
        // VDDA on this board is a regulated supply - sanity-bound it rather than assert an exact value
        TEST_ASSERT_TRUE(vdda > 1.5F && vdda < 4.0F);

        float temp_c = 0.0F;
        TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, adc_get_temp_celsius(nullptr, &temp_c));
        TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, adc_get_temp_celsius(ADC1, nullptr));
        TEST_ASSERT_EQUAL(HAL_OK, adc_get_temp_celsius(ADC1, &temp_c));
        // Plausible bench/board temperature bound, not a calibrated accuracy check
        TEST_ASSERT_TRUE(temp_c > -40.0F && temp_c < 125.0F);

        // adc_get_value_right_aligned is pure math over its raw_data argument - verify it
        // directly against hand-computed expected values for every resolution
        struct case_t {
            adc_resolution_t res;
            uint16_t         raw;
            uint32_t         full_scale;
        };
        constexpr std::array<case_t, 4> CASES{{
            {ADC_RES_6_BITS, 63, 64},
            {ADC_RES_8_BITS, 255, 256},
            {ADC_RES_10_BITS, 1023, 1024},
            {ADC_RES_12_BITS, 4095, 4096},
        }};

        float measured_vdda = 0.0F;
        TEST_ASSERT_EQUAL(HAL_OK, adc_get_vdda(ADC1, &measured_vdda));

        for (const auto& c : CASES) {
            float voltage = 0.0F;
            TEST_ASSERT_EQUAL(HAL_OK, adc_get_value_right_aligned(ADC1, c.raw, c.res, &voltage));
            const float expected = (measured_vdda * static_cast<float>(c.raw)) / static_cast<float>(c.full_scale);
            TEST_ASSERT_FLOAT_WITHIN(0.001F, expected, voltage);
        }

        TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, adc_get_value_right_aligned(nullptr, 100, ADC_RES_12_BITS, &voltage_dummy_unused_guard()));
    }

    void value_right_aligned_rejects_unknown_resolution() {
        reset_to_baseline();

        float      voltage          = 0.0F;
        const auto bogus_resolution = static_cast<adc_resolution_t>(0xFF);
        TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, adc_get_value_right_aligned(ADC1, 100, bogus_resolution, &voltage));
        TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, adc_get_value_right_aligned(ADC1, 100, ADC_RES_12_BITS, nullptr));
    }

    void regular_group_channel_sequence_registers_are_placed_correctly() {
        reset_to_baseline();

        // Exercises the SQR3/SQR2 boundary at 6/7 channels, the SQR2/SQR1 boundary at
        // 12/13 channels, and both extremes (1 and the maximum of 16)
        constexpr std::array<size_t, 6> COUNTS{1, 6, 7, 12, 13, MAX_REGULAR_CHANNELS};

        for (const auto count : COUNTS) {
            std::array<adc_channels_t, MAX_REGULAR_CHANNELS> sequence{};
            for (size_t i = 0; i < count; i++) {
                sequence[i] = static_cast<adc_channels_t>(i % 16);
            }

            uint16_t                      buffer = 0;
            const adc_continuous_config_t config{
                .channels         = {sequence.data(), count},
                .trigger          = RG_TRIGGER_SOFTWARE,
                .trigger_polarity = RISING_EDGE,
                .buffer_1         = &buffer,
                .buffer_2         = nullptr,
                .buffer_size      = 1,
                .priority         = DMA_PRIORITY_LOW,
                .circular_mode    = DMA_MODE_NO_CIRCULAR,
                .callbacks        = {},
            };

            TEST_ASSERT_EQUAL(HAL_OK, adc_regular_group_cont_start_conv(ADC1, &config));

            constexpr uint32_t BIT_WIDTH = 5;
            constexpr uint32_t MASK      = (1U << BIT_WIDTH) - 1U;

            for (size_t i = 0; i < count; i++) {
                const uint32_t expected = static_cast<uint32_t>(sequence[i]) & MASK;
                uint32_t       actual   = 0;
                if (i >= 12) {
                    actual = (ADC1->SQR1 >> ((i - 12) * BIT_WIDTH)) & MASK;
                } else if (i >= 6) {
                    actual = (ADC1->SQR2 >> ((i - 6) * BIT_WIDTH)) & MASK;
                } else {
                    actual = (ADC1->SQR3 >> (i * BIT_WIDTH)) & MASK;
                }
                TEST_ASSERT_EQUAL_UINT32(expected, actual);
            }

            TEST_ASSERT_EQUAL_UINT32(count - 1, (ADC1->SQR1 & ADC_SQR1_L) >> ADC_SQR1_L_Pos);
            TEST_ASSERT_EQUAL(count > 1, (ADC1->CR1 & ADC_CR1_SCAN) != 0);

            TEST_ASSERT_EQUAL(HAL_OK, adc_regular_group_cont_end_conv(ADC1));
        }

        reset_to_baseline();
    }

    void continuous_conversion_arg_guards() {
        reset_to_baseline();

        constexpr adc_channels_t channel = ADC_CHANNEL_0;
        std::array<uint16_t, 4>  buffer  = {};

        const adc_continuous_config_t base{
            .channels         = {.channels_sequence = &channel, .num_of_channels = 1},
            .trigger          = RG_TRIGGER_SOFTWARE,
            .trigger_polarity = RISING_EDGE,
            .buffer_1         = buffer.data(),
            .buffer_2         = nullptr,
            .buffer_size      = buffer.size(),
            .priority         = DMA_PRIORITY_LOW,
            .circular_mode    = DMA_MODE_NO_CIRCULAR,
            .callbacks        = {},
        };

        TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, adc_regular_group_cont_start_conv(nullptr, &base));
        TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, adc_regular_group_cont_start_conv(ADC1, nullptr));

        auto no_channels     = base;
        no_channels.channels = {.channels_sequence = nullptr, .num_of_channels = 1};
        TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, adc_regular_group_cont_start_conv(ADC1, &no_channels));

        auto zero_count     = base;
        zero_count.channels = {.channels_sequence = &channel, .num_of_channels = 0};
        TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, adc_regular_group_cont_start_conv(ADC1, &zero_count));

        auto too_many                     = base;
        too_many.channels.num_of_channels = MAX_REGULAR_CHANNELS + 1;
        TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, adc_regular_group_cont_start_conv(ADC1, &too_many));

        auto no_buffer     = base;
        no_buffer.buffer_1 = nullptr;
        TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, adc_regular_group_cont_start_conv(ADC1, &no_buffer));

        auto missing_second_buffer          = base;
        missing_second_buffer.circular_mode = DMA_MODE_DOUBLE_BUFFER;
        missing_second_buffer.buffer_2      = nullptr;
        TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, adc_regular_group_cont_start_conv(ADC1, &missing_second_buffer));
    }

    void continuous_conversion_fills_the_buffer_via_dma_and_calls_back() {
        reset_to_baseline();

        constexpr auto          CHANNELS = std::array{ADC_CHANNEL_0, ADC_CHANNEL_1};
        std::array<uint16_t, 8> buffer{};
        buffer.fill(0xFFFF);

        s_cont_done = false;

        const adc_continuous_config_t config{
            .channels         = {.channels_sequence = CHANNELS.data(), .num_of_channels = CHANNELS.size()},
            .trigger          = RG_TRIGGER_SOFTWARE,
            .trigger_polarity = RISING_EDGE,
            .buffer_1         = buffer.data(),
            .buffer_2         = nullptr,
            .buffer_size      = static_cast<uint16_t>(buffer.size()),
            .priority         = DMA_PRIORITY_LOW,
            .circular_mode    = DMA_MODE_NO_CIRCULAR,
            .callbacks =
                {
                    .on_buffer_full       = cont_done_cb,
                    .on_transfer_error    = nullptr,
                    .on_direct_mode_error = nullptr,
                    .on_data_overrun      = nullptr,
                    .user                 = nullptr,
                },
        };

        adc_enable_nvic_irq(true);
        TEST_ASSERT_EQUAL(HAL_OK, adc_regular_group_cont_start_conv(ADC1, &config));

        TEST_ASSERT_TRUE_MESSAGE(wait_until([]() {
                                     return s_cont_done;
                                 }),
                                 "Continuous DMA conversion never completed");

        for (const auto sample : buffer) {
            TEST_ASSERT_TRUE(sample <= 0x0FFFU);
        }

        adc_enable_nvic_irq(false);
        reset_to_baseline();
    }

    void injected_group_covers_every_channel_count_and_offset_slot() {
        reset_to_baseline();

        constexpr std::array<size_t, 4> COUNTS{1, 2, 3, 4};

        for (const auto count : COUNTS) {
            std::array<adc_channels_t, MAX_INJECTED_CHANNELS> channels{};
            for (size_t i = 0; i < count; i++) {
                channels[i] = static_cast<adc_channels_t>(i);
            }

            adc_injected_group_config_t config{};
            config.channels         = {channels.data(), count};
            config.trigger          = JG_TRIGGER_SOFTWARE;
            config.trigger_polarity = RISING_EDGE;
            for (size_t i = 0; i < count; i++) {
                config.offsets[i] = static_cast<uint16_t>(10 * (i + 1));
            }
            config.on_conv_complete = nullptr;
            config.arg              = nullptr;

            TEST_ASSERT_EQUAL(HAL_OK, adc_injected_group_start_conv(ADC1, &config));

            TEST_ASSERT_TRUE_MESSAGE(wait_until([]() {
                                         return (ADC1->SR & ADC_SR_JEOC) != 0;
                                     }),
                                     "Injected conversion never finished");

            std::array<uint16_t, MAX_INJECTED_CHANNELS> results{};
            TEST_ASSERT_EQUAL(HAL_OK, adc_injected_group_get_result(ADC1, results.data(), count));
            for (size_t i = 0; i < count; i++) {
                TEST_ASSERT_TRUE(results[i] <= 0x0FFFU);
            }

            ADC1->SR &= ~ADC_SR_JEOC;
        }

        reset_to_baseline();
    }

    void injected_group_result_before_completion_is_not_done() {
        reset_to_baseline();
        ADC1->SR &= ~ADC_SR_JEOC;

        std::array<uint16_t, 1> result{};
        TEST_ASSERT_EQUAL(HAL_ERR_NOT_DONE, adc_injected_group_get_result(ADC1, result.data(), 1));
    }

    void injected_group_arg_guards() {
        reset_to_baseline();

        constexpr adc_channels_t    CH[1] = {ADC_CHANNEL_0};
        adc_injected_group_config_t config{};
        config.channels = {CH, 1};
        config.trigger  = JG_TRIGGER_SOFTWARE;

        TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, adc_injected_group_start_conv(nullptr, &config));
        TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, adc_injected_group_start_conv(ADC1, nullptr));

        auto no_channels     = config;
        no_channels.channels = {nullptr, 1};
        TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, adc_injected_group_start_conv(ADC1, &no_channels));

        auto too_many                     = config;
        too_many.channels.num_of_channels = MAX_INJECTED_CHANNELS + 1;
        TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, adc_injected_group_start_conv(ADC1, &too_many));

        std::array<uint16_t, 1> result{};
        TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, adc_injected_group_get_result(ADC1, nullptr, 1));
        TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, adc_injected_group_get_result(ADC1, result.data(), 0));
        TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, adc_injected_group_get_result(ADC1, result.data(), MAX_INJECTED_CHANNELS + 1));
    }

    void injected_group_invokes_the_completion_callback_via_the_isr() {
        reset_to_baseline();

        constexpr adc_channels_t    CH[1] = {ADC_CHANNEL_0};
        adc_injected_group_config_t config{};
        config.channels         = {CH, 1};
        config.trigger          = JG_TRIGGER_SOFTWARE;
        config.trigger_polarity = RISING_EDGE;
        config.on_conv_complete = injected_done_cb;
        config.arg              = nullptr;

        s_injected_done = false;
        adc_enable_nvic_irq(true);

        TEST_ASSERT_EQUAL(HAL_OK, adc_injected_group_start_conv(ADC1, &config));
        TEST_ASSERT_TRUE_MESSAGE(wait_until([]() {
                                     return s_injected_done;
                                 }),
                                 "Injected-group completion callback never fired");

        // The isr helper must have cleared both the flag and the interrupt enable behind us
        TEST_ASSERT_FALSE(ADC1->SR & ADC_SR_JEOC);
        TEST_ASSERT_FALSE(ADC1->CR1 & ADC_CR1_JEOCIE);

        adc_enable_nvic_irq(false);
        reset_to_baseline();
    }

    void analog_watchdog_validates_thresholds_and_monitor_flags() {
        reset_to_baseline();

        adc_analog_wdg_config_t config{};
        config.min_adc_value            = 100;
        config.max_adc_value            = 50; // inverted on purpose
        config.on_thresholds_violated   = wdg_cb;
        config.monitor_regular_channels = true;

        TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, adc_analog_wdg_start(ADC1, &config));

        config.min_adc_value = 50;
        config.max_adc_value = 100;
        TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, adc_analog_wdg_start(nullptr, &config));
        TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, adc_analog_wdg_start(ADC1, nullptr));

        auto no_callback                   = config;
        no_callback.on_thresholds_violated = nullptr;
        TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, adc_analog_wdg_start(ADC1, &no_callback));

        auto no_monitor_target                      = config;
        no_monitor_target.monitor_regular_channels  = false;
        no_monitor_target.monitor_injected_channels = false;
        TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, adc_analog_wdg_start(ADC1, &no_monitor_target));

        struct combo_t {
            bool     regular;
            bool     injected;
            uint32_t expected_bits;
        };
        constexpr std::array<combo_t, 3> COMBOS{{
            {true, true, ADC_CR1_AWDEN | ADC_CR1_JAWDEN},
            {false, true, ADC_CR1_JAWDEN},
            {true, false, ADC_CR1_AWDEN},
        }};

        for (const auto& combo : COMBOS) {
            auto c                      = config;
            c.monitor_regular_channels  = combo.regular;
            c.monitor_injected_channels = combo.injected;
            c.min_adc_value             = 0x1234; // deliberately > 12 bits to verify masking
            c.max_adc_value             = 0xFFFF;

            TEST_ASSERT_EQUAL(HAL_OK, adc_analog_wdg_start(ADC1, &c));
            TEST_ASSERT_EQUAL_UINT32(combo.expected_bits, ADC1->CR1 & (ADC_CR1_AWDEN | ADC_CR1_JAWDEN));
            TEST_ASSERT_TRUE(ADC1->CR1 & ADC_CR1_AWDIE);
            TEST_ASSERT_EQUAL_UINT32(c.max_adc_value & 0xFFFU, ADC1->HTR);
            TEST_ASSERT_EQUAL_UINT32(c.min_adc_value & 0xFFFU, ADC1->LTR);

            TEST_ASSERT_EQUAL(HAL_OK, adc_analog_wdg_stop(ADC1));
            TEST_ASSERT_EQUAL_UINT32(0, ADC1->CR1 & (ADC_CR1_AWDEN | ADC_CR1_JAWDEN | ADC_CR1_AWDIE));
        }

        TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, adc_analog_wdg_stop(nullptr));

        reset_to_baseline();
    }

    void all() {
        LOGI(TAG, "Starting the tests on the ADC driver");

        RUN_TEST(clk_enable_rejects_unknown_handles);
        RUN_TEST(power_on_toggles_adon);
        RUN_TEST(configure_covers_every_alignment_resolution_and_sample_time);
        RUN_TEST(deconfigure_powers_down_and_clears_state);
        RUN_TEST(clk_configure_sweeps_every_prescaler);
        RUN_TEST(nvic_irq_enable_toggles_the_adc_line);
        RUN_TEST(temp_sensor_power_toggles_tsvrefe);
        RUN_TEST(oneshot_regular_group_completes_within_12_bit_range);
        RUN_TEST(internal_channel_readings_complete_and_manage_their_own_enables);
        RUN_TEST(voltage_and_temperature_math_are_internally_consistent);
        RUN_TEST(value_right_aligned_rejects_unknown_resolution);
        RUN_TEST(regular_group_channel_sequence_registers_are_placed_correctly);
        RUN_TEST(continuous_conversion_arg_guards);
        RUN_TEST(continuous_conversion_fills_the_buffer_via_dma_and_calls_back);
        RUN_TEST(injected_group_covers_every_channel_count_and_offset_slot);
        RUN_TEST(injected_group_result_before_completion_is_not_done);
        RUN_TEST(injected_group_arg_guards);
        RUN_TEST(injected_group_invokes_the_completion_callback_via_the_isr);
        RUN_TEST(analog_watchdog_validates_thresholds_and_monitor_flags);

        LOGI(TAG, "Done with all tests on the ADC driver");
    }

} // namespace test::adc
