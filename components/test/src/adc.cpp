#include "drivers/dma_types.h"
#include "stm32f411xe.h"
#include "Unity/unity.h"

#include "drivers/adc_types.h"
#include "utils/common.h"
#include "drivers/adc.h"
#include "utils/board.h"
#include "test/adc.hpp"
#include "utils/err.h"
#include "utils/log.h"

#include <array>
#include <cstdint>
#include <utility>


namespace test::adc {

    namespace {

        constexpr const char* TAG = "ADC_Test";

        // Helpers
        void reset_to_baseline() {
            adc_clk_configure(ADC_CLK_PRESCALER_4);
            constexpr adc_config_t config = {
                .alignment       = ADC_RIGHT_ALIGN,
                .resolution      = ADC_RES_12_BITS,
                .sampling_cycles = ADC_SAMPLE_28_CYCLES,
            };
            TEST_ASSERT_EQUAL(HAL_OK, adc_configure(ADC1, &config));
        }

        volatile bool s_injected_done = false;
        void          injected_done_cb(void*) {
            s_injected_done = true;
        }

        volatile bool    s_cont_done         = false;
        volatile uint8_t s_filled_buffer_idx = 0;
        void             cont_done_cb(void*, uint8_t filled_buffer_idx) {
            s_filled_buffer_idx = filled_buffer_idx;
            s_cont_done         = true;
        }

        volatile bool s_wdg_triggered = false;
        void          wdg_cb(void*) {
            s_wdg_triggered = true;
        }

        template<typename predicate>
        bool wait_until(predicate pred) {
            uint32_t timeout = TIMEOUT;
            while (!pred() && --timeout);
            return pred();
        }

        // TESTS
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

            constexpr auto RESOLUTIONS = std::array{
                ADC_RES_6_BITS,
                ADC_RES_8_BITS,
                ADC_RES_10_BITS,
                ADC_RES_12_BITS,
            };
            constexpr auto SAMPLE_TIMES = std::array{
                ADC_SAMPLE_3_CYCLES,
                ADC_SAMPLE_15_CYCLES,
                ADC_SAMPLE_28_CYCLES,
                ADC_SAMPLE_56_CYCLES,
                ADC_SAMPLE_84_CYCLES,
                ADC_SAMPLE_112_CYCLES,
                ADC_SAMPLE_144_CYCLES,
                ADC_SAMPLE_480_CYCLES,
            };

            for (const auto resolution : RESOLUTIONS) {
                for (const auto time : SAMPLE_TIMES) {
                    const adc_config_t config{.alignment = ADC_LEFT_ALIGN, .resolution = resolution, .sampling_cycles = time};
                    TEST_ASSERT_EQUAL(HAL_OK, adc_configure(ADC1, &config));

                    TEST_ASSERT_TRUE(ADC1->CR2 & ADC_CR2_ALIGN);
                    TEST_ASSERT_EQUAL_UINT32(std::to_underlying(resolution), (ADC1->CR1 & ADC_CR1_RES) >> ADC_CR1_RES_Pos);

                    // Every external channel must carry the same sampling time
                    TEST_ASSERT_EQUAL_UINT32(std::to_underlying(time), (ADC1->SMPR1 >> ADC_SMPR1_SMP10_Pos) & 0b111UL);
                    TEST_ASSERT_EQUAL_UINT32(std::to_underlying(time), (ADC1->SMPR1 >> ADC_SMPR1_SMP15_Pos) & 0b111UL);
                    TEST_ASSERT_EQUAL_UINT32(std::to_underlying(time), (ADC1->SMPR2 >> ADC_SMPR2_SMP0_Pos) & 0b111UL);
                    TEST_ASSERT_EQUAL_UINT32(std::to_underlying(time), (ADC1->SMPR2 >> ADC_SMPR2_SMP9_Pos) & 0b111UL);
                }
            }

            constexpr adc_config_t right_align_config = {
                .alignment       = ADC_RIGHT_ALIGN,
                .resolution      = ADC_RES_12_BITS,
                .sampling_cycles = ADC_SAMPLE_28_CYCLES,
            };
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
            for (const auto prescaler : {ADC_CLK_PRESCALER_2, ADC_CLK_PRESCALER_4, ADC_CLK_PRESCALER_6, ADC_CLK_PRESCALER_8}) {
                adc_clk_configure(prescaler);
                TEST_ASSERT_EQUAL_UINT32(std::to_underlying(prescaler), (ADC->CCR & ADC_CCR_ADCPRE) >> ADC_CCR_ADCPRE_Pos);
            }
            adc_clk_configure(ADC_CLK_PRESCALER_4);
        }

        void nvic_irq_enable_toggles_the_adc_line() {
            auto irq_is_enabled = []() {
                return (NVIC->ISER[std::to_underlying(ADC_IRQn) >> 5] & (1UL << (std::to_underlying(ADC_IRQn) & 0x1FU)));
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

            adc_configure_channel(ADC_CHANNEL_0);
            TEST_ASSERT_EQUAL(HAL_OK, adc_regular_group_get_oneshot(ADC1, ADC_CHANNEL_0, &raw));
            TEST_ASSERT_TRUE(raw <= 0xFFFU);
            LOGI(TAG, "[oneshot_regular_group_completes_within_12_bit_range] Raw ADC channel 0 sample: %u", raw);
        }

        void internal_channel_readings_complete_and_manage_their_own_enables() {
            reset_to_baseline();

            uint16_t raw = 0;

            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, adc_get_v_bat(nullptr, &raw));
            TEST_ASSERT_EQUAL(HAL_OK, adc_get_v_bat(ADC1, &raw));
            TEST_ASSERT_TRUE(raw <= (0xFFFU * VBAT_DIVIDER_RATIO));
            LOGI(TAG, "[internal_channel_readings_complete_and_manage_their_own_enables] Raw V_bat sample: %u", raw);
            // VBATE must be switched back off once the reading is done
            TEST_ASSERT_FALSE(ADC->CCR & ADC_CCR_VBATE);

            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, adc_get_temperature(nullptr, &raw));
            TEST_ASSERT_EQUAL(HAL_OK, adc_get_temperature(ADC1, &raw));
            TEST_ASSERT_TRUE(raw <= 0xFFFU);
            LOGI(TAG, "[internal_channel_readings_complete_and_manage_their_own_enables] Raw temperature sensor sample: %u", raw);
            // The temperature sensor/VREFINT stay latched on afterwards as they share the TSVREFE bit
            TEST_ASSERT_TRUE(ADC->CCR & ADC_CCR_TSVREFE);

            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, adc_get_v_ref_internal(nullptr, &raw));
            TEST_ASSERT_EQUAL(HAL_OK, adc_get_v_ref_internal(ADC1, &raw));
            TEST_ASSERT_TRUE(raw <= 0xFFFU);
            LOGI(TAG, "[internal_channel_readings_complete_and_manage_their_own_enables] Raw V_ref_int sample: %u", raw);

            adc_power_on_temp_sensor(false);
        }

        void voltage_and_temperature_math_are_internally_consistent() {
            reset_to_baseline();

            float vdda = 0.0F;
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, adc_get_vdda(nullptr, &vdda));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, adc_get_vdda(ADC1, nullptr));
            TEST_ASSERT_EQUAL(HAL_OK, adc_get_vdda(ADC1, &vdda));
            // VDDA on this board is a regulated supply. Sanity bound it rather than assert an exact value
            TEST_ASSERT_TRUE(vdda > 3.0F && vdda < 3.5F);
            LOGI(TAG, "[voltage_and_temperature_math_are_internally_consistent] Vdda: %.3fV", (double)vdda);

            float temp_c = 0.0F;
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, adc_get_temp_celsius(nullptr, &temp_c));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, adc_get_temp_celsius(ADC1, nullptr));
            TEST_ASSERT_EQUAL(HAL_OK, adc_get_temp_celsius(ADC1, &temp_c));
            // Plausible temperature bound for a test runner. Not a calibrated accuracy check
            TEST_ASSERT_TRUE(temp_c > 25.0F && temp_c < 35.0F);
            LOGI(TAG, "[voltage_and_temperature_math_are_internally_consistent] Temperature: %.3fC", (double)temp_c);
        }

        void value_right_aligned_rejects_unknown_resolution() {
            reset_to_baseline();

            float          voltage          = 0.0F;
            constexpr auto bogus_resolution = static_cast<adc_resolution_t>(0xFF);
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, adc_get_value_right_aligned(ADC1, 100, bogus_resolution, &voltage));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, adc_get_value_right_aligned(ADC1, 100, ADC_RES_12_BITS, nullptr));
        }

        void regular_group_channel_sequence_registers_are_placed_correctly() {
            reset_to_baseline();

            // Exercises the SQR3/SQR2 boundary at 6/7 channels, the SQR2/SQR1 boundary at
            // 12/13 channels, and both extremes (1 and the maximum of 16)
            for (const auto count : {1U, 6U, 7U, 12U, 13U, MAX_REGULAR_CHANNELS}) {
                std::array<adc_channel_t, MAX_REGULAR_CHANNELS> sequence{};
                for (size_t i = 0; i < count; i++) {
                    sequence[i] = static_cast<adc_channel_t>(i);
                }

                std::array<uint16_t, MAX_REGULAR_CHANNELS> buffer{};
                const adc_continuous_config_t              config = {
                    .channels         = {.sequence = sequence.data(), .num_of_channels = count},
                    .trigger          = RG_TRIGGER_SOFTWARE,
                    .trigger_polarity = ADC_POLARITY_NONE,
                    .buffer_1         = buffer.data(),
                    .buffer_2         = nullptr,
                    .buffer_size      = static_cast<uint16_t>(count),
                    .priority         = DMA_PRIORITY_LOW,
                    // Circular mode is used here so the conversion doesn't terminate when the buffer is filled up
                    .circular_mode = DMA_MODE_CIRCULAR,
                    .callbacks     = {},
                };

                TEST_ASSERT_EQUAL(HAL_OK, adc_regular_group_cont_start_conv(ADC1, &config));

                constexpr uint32_t BIT_WIDTH = 5;
                constexpr uint32_t MASK      = (1U << BIT_WIDTH) - 1U;

                for (size_t i = 0; i < count; i++) {
                    const uint32_t expected = std::to_underlying(sequence[i]) & MASK;
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
                TEST_ASSERT_EQUAL(count > 1, static_cast<bool>(ADC1->CR1 & ADC_CR1_SCAN));

                TEST_ASSERT_EQUAL(HAL_OK, adc_regular_group_cont_end_conv(ADC1));
            }

            reset_to_baseline();
        }

        void continuous_conversion_arg_guards() {
            reset_to_baseline();

            constexpr adc_channel_t channel = ADC_CHANNEL_0;
            std::array<uint16_t, 4> buffer  = {};

            const adc_continuous_config_t base = {
                .channels         = {.sequence = &channel, .num_of_channels = 1},
                .trigger          = RG_TRIGGER_SOFTWARE,
                .trigger_polarity = ADC_POLARITY_NONE,
                .buffer_1         = buffer.data(),
                .buffer_2         = nullptr,
                .buffer_size      = buffer.size(),
                .priority         = DMA_PRIORITY_LOW,
                .circular_mode    = DMA_MODE_ONESHOT,
                .callbacks        = {},
            };

            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, adc_regular_group_cont_start_conv(nullptr, &base));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, adc_regular_group_cont_start_conv(ADC1, nullptr));

            auto no_channels     = base;
            no_channels.channels = {.sequence = nullptr, .num_of_channels = 1};
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, adc_regular_group_cont_start_conv(ADC1, &no_channels));

            auto zero_count     = base;
            zero_count.channels = {.sequence = &channel, .num_of_channels = 0};
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

            s_cont_done = false;

            const adc_continuous_config_t config = {
                .channels         = {.sequence = CHANNELS.data(), .num_of_channels = CHANNELS.size()},
                .trigger          = RG_TRIGGER_SOFTWARE,
                .trigger_polarity = ADC_POLARITY_NONE,
                .buffer_1         = buffer.data(),
                .buffer_2         = nullptr,
                .buffer_size      = buffer.size(),
                .priority         = DMA_PRIORITY_LOW,
                .circular_mode    = DMA_MODE_ONESHOT,
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
            for (const auto channel : CHANNELS) {
                adc_configure_channel(channel);
            }
            TEST_ASSERT_EQUAL(HAL_OK, adc_regular_group_cont_start_conv(ADC1, &config));

            TEST_ASSERT_TRUE_MESSAGE(wait_until([]() {
                                         return s_cont_done;
                                     }),
                                     "Continuous DMA conversion never completed");

            for (size_t i = 0; i < buffer.size(); i++) {
                TEST_ASSERT_TRUE(buffer[i] <= 0xFFFU);
                float voltage = 0;
                TEST_ASSERT_EQUAL(HAL_OK, adc_get_value_right_aligned(ADC1, buffer[i], ADC_RES_12_BITS, &voltage));
                // The samples are interleaved: [0][1][0][1]....
                if ((i & 1) == 0) {
                    LOGI(TAG, "ADC channel 0: %u (%.3fV)", buffer[i], (double)voltage);
                } else {
                    LOGI(TAG, "ADC channel 1: %u (%.3fV)", buffer[i], (double)voltage);
                }
            }

            adc_enable_nvic_irq(false);
            reset_to_baseline();
        }

        void injected_group_covers_every_channel_count_and_offset_slot() {
            reset_to_baseline();

            for (const auto count : {1U, 2U, 3U, 4U}) {
                std::array<adc_channel_t, MAX_INJECTED_CHANNELS> channels{};
                for (size_t i = 0; i < count; i++) {
                    channels[i] = static_cast<adc_channel_t>(i);
                    adc_configure_channel(static_cast<adc_channel_t>(i));
                }

                adc_injected_group_config_t config{};
                config.channels         = {.sequence = channels.data(), .num_of_channels = count};
                config.trigger          = JG_TRIGGER_SOFTWARE;
                config.trigger_polarity = ADC_POLARITY_NONE;
                for (size_t i = 0; i < count; i++) {
                    config.offsets[i] = static_cast<uint16_t>(10 * (i + 1));
                }
                config.on_conv_complete = nullptr;
                config.arg              = nullptr;

                TEST_ASSERT_EQUAL(HAL_OK, adc_injected_group_start_conv(ADC1, &config));

                TEST_ASSERT_TRUE_MESSAGE(wait_until([]() {
                                             return (ADC1->SR & ADC_SR_JEOC);
                                         }),
                                         "Injected conversion never finished");

                std::array<uint16_t, MAX_INJECTED_CHANNELS> results{};
                TEST_ASSERT_EQUAL(HAL_OK, adc_injected_group_get_result(ADC1, results.data(), count));

                for (size_t i = 0; i < count; i++) {
                    TEST_ASSERT_TRUE(results[i] <= 0xFFFU);
                    float voltage = 0;
                    TEST_ASSERT_EQUAL(HAL_OK, adc_get_value_right_aligned(ADC1, results[i], ADC_RES_12_BITS, &voltage));
                    LOGI(TAG, "Sample %zu: %u (%.3fV)", i, results[i], (double)voltage);
                }
            }

            reset_to_baseline();
        }

        void injected_group_result_before_completion_is_not_done() {
            reset_to_baseline();
            ADC1->SR &= ~ADC_SR_JEOC;

            uint16_t result = 0;
            TEST_ASSERT_EQUAL(HAL_ERR_NOT_DONE, adc_injected_group_get_result(ADC1, &result, 1));
        }

        void injected_group_arg_guards() {
            reset_to_baseline();

            constexpr adc_channel_t     channel = ADC_CHANNEL_0;
            adc_injected_group_config_t config{};
            config.channels = {.sequence = &channel, .num_of_channels = 1};
            config.trigger  = JG_TRIGGER_SOFTWARE;

            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, adc_injected_group_start_conv(nullptr, &config));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, adc_injected_group_start_conv(ADC1, nullptr));

            auto no_channels     = config;
            no_channels.channels = {.sequence = nullptr, .num_of_channels = 1};
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, adc_injected_group_start_conv(ADC1, &no_channels));

            auto too_many                     = config;
            too_many.channels.num_of_channels = MAX_INJECTED_CHANNELS + 1;
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, adc_injected_group_start_conv(ADC1, &too_many));

            uint16_t result = 0;
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, adc_injected_group_get_result(ADC1, nullptr, 1));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, adc_injected_group_get_result(ADC1, &result, 0));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, adc_injected_group_get_result(ADC1, &result, MAX_INJECTED_CHANNELS + 1));
        }

        void injected_group_invokes_the_completion_callback_via_the_isr() {
            reset_to_baseline();

            constexpr adc_channel_t     channels = ADC_CHANNEL_0;
            adc_injected_group_config_t config{};
            config.channels         = {.sequence = &channels, .num_of_channels = 1};
            config.trigger          = JG_TRIGGER_SOFTWARE;
            config.trigger_polarity = ADC_POLARITY_NONE;
            config.on_conv_complete = injected_done_cb;
            config.arg              = nullptr;

            s_injected_done = false;
            adc_enable_nvic_irq(true);

            TEST_ASSERT_EQUAL(HAL_OK, adc_injected_group_start_conv(ADC1, &config));
            TEST_ASSERT_TRUE_MESSAGE(wait_until([]() {
                                         return s_injected_done;
                                     }),
                                     "Injected group completion callback never fired");

            // Only the JEOCIE bit should have been cleared
            TEST_ASSERT_TRUE(ADC1->SR & ADC_SR_JEOC);
            TEST_ASSERT_FALSE(ADC1->CR1 & ADC_CR1_JEOCIE);

            std::array<uint16_t, MAX_INJECTED_CHANNELS> result{};
            TEST_ASSERT_EQUAL(HAL_OK, adc_injected_group_get_result(ADC1, result.data(), 1));

            // The JEOC flag should be cleared now by reading the result
            TEST_ASSERT_FALSE(ADC1->SR & ADC_SR_JEOC);

            adc_enable_nvic_irq(false);
            reset_to_baseline();
        }

        void analog_watchdog_validates_thresholds_and_monitor_flags() {
            reset_to_baseline();
            adc_enable_nvic_irq(true);

            adc_analog_wdg_config_t config   = {};
            config.min_adc_value             = 100;
            config.max_adc_value             = 50; // Inverted on purpose
            config.on_thresholds_violated    = wdg_cb;
            config.monitor_regular_channels  = true;
            config.monitor_injected_channels = false;

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
            constexpr std::array<combo_t, 3> COMBOS = {{
                {.regular = true, .injected = true, .expected_bits = ADC_CR1_AWDEN | ADC_CR1_JAWDEN},
                {.regular = false, .injected = true, .expected_bits = ADC_CR1_JAWDEN},
                {.regular = true, .injected = false, .expected_bits = ADC_CR1_AWDEN},
            }};

            for (const auto& combo : COMBOS) {
                auto cfg                      = config;
                cfg.monitor_regular_channels  = combo.regular;
                cfg.monitor_injected_channels = combo.injected;
                cfg.min_adc_value             = 0x1234; // Deliberately > 12 bits to verify masking. Should be masked to 0x234
                cfg.max_adc_value             = 0xFFFF; // Should be masked to 0xFFF

                TEST_ASSERT_EQUAL(HAL_OK, adc_analog_wdg_start(ADC1, &cfg));
                TEST_ASSERT_EQUAL_UINT32(combo.expected_bits, ADC1->CR1 & (ADC_CR1_AWDEN | ADC_CR1_JAWDEN));
                TEST_ASSERT_TRUE(ADC1->CR1 & ADC_CR1_AWDIE);
                TEST_ASSERT_EQUAL_UINT32(cfg.max_adc_value & 0xFFFU, ADC1->HTR);
                TEST_ASSERT_EQUAL_UINT32(cfg.min_adc_value & 0xFFFU, ADC1->LTR);

                TEST_ASSERT_EQUAL(HAL_OK, adc_analog_wdg_stop(ADC1));
                TEST_ASSERT_EQUAL_UINT32(0, ADC1->CR1 & (ADC_CR1_AWDEN | ADC_CR1_JAWDEN | ADC_CR1_AWDIE));
            }

            // Test to see if the watchdog fires the callback
            config.max_adc_value             = 1; // Deliberately low so the watchdog fires
            config.min_adc_value             = 0;
            config.on_thresholds_violated    = wdg_cb;
            config.monitor_regular_channels  = true;
            config.monitor_injected_channels = true;

            s_wdg_triggered = false;

            TEST_ASSERT_EQUAL(HAL_OK, adc_analog_wdg_start(ADC1, &config));

            // Start conversion on any channel
            constexpr adc_channel_t channel = ADC_CHANNEL_0;
            uint16_t                raw     = 0;
            TEST_ASSERT_EQUAL(HAL_OK, adc_regular_group_get_oneshot(ADC1, channel, &raw));

            TEST_ASSERT_TRUE_MESSAGE(wait_until([]() {
                                         return s_wdg_triggered;
                                     }),
                                     "Analog watchdog never fires");

            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, adc_analog_wdg_stop(nullptr));
            TEST_ASSERT_EQUAL(HAL_OK, adc_analog_wdg_stop(ADC1));

            adc_enable_nvic_irq(false);
            reset_to_baseline();
        }

    } // namespace

    void all() {
        LOGI(TAG, "Starting the tests on the ADC driver");
        UNITY_BEGIN();

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

        UNITY_END();
        LOGI(TAG, "Done with all tests on the ADC driver");
    }

} // namespace test::adc
