#include "stm32f411xe.h"
#include "Unity/unity.h"

#include "drivers/gpio_types.h"
#include "drivers/dma_types.h"
#include "drivers/adc_types.h"
#include "utils/common.h"
#include "drivers/gpio.h"
#include "drivers/adc.h"
#include "utils/board.h"
#include "utils/tick.h"
#include "test/adc.hpp"
#include "utils/err.h"
#include "utils/log.h"

#include <array>
#include <cstring>
#include <cstdint>
#include <utility>


namespace test::adc {

    namespace {

        constexpr const char* TAG = "ADC_Test";

        // Helpers
        void reset_to_baseline() {
            TEST_ASSERT_EQUAL(HAL_OK, adc_deconfigure(ADC1));
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

        volatile bool     s_overrun_fired      = false;
        volatile uint8_t  s_overrun_buffer_idx = 0xFFU;
        volatile uint16_t s_overrun_items_left = 0;
        void              overrun_cb(void*, uint8_t filled_buffer_idx, uint16_t num_of_items_left) {
            s_overrun_buffer_idx = filled_buffer_idx;
            s_overrun_items_left = num_of_items_left;
            s_overrun_fired      = true;
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

        void channel_get_gpio_matches_the_pin_map() {
            struct expectation_t {
                adc_channel_t channel;
                GPIO_TypeDef* port;
                uint16_t      pin;
            };
            const std::array<expectation_t, 4> EXPECTATIONS = {{
                {.channel = ADC_CHANNEL_0, .port = GPIOA, .pin = GPIO_PIN_0},
                {.channel = ADC_CHANNEL_3, .port = GPIOA, .pin = GPIO_PIN_3},
                {.channel = ADC_CHANNEL_8, .port = GPIOB, .pin = GPIO_PIN_0},
                {.channel = ADC_CHANNEL_10, .port = GPIOC, .pin = GPIO_PIN_0},
            }};

            for (const auto& expectation : EXPECTATIONS) {
                const gpio_pin_ctx_t ctx = adc_channel_get_gpio(expectation.channel);
                TEST_ASSERT_EQUAL_PTR(expectation.port, ctx.port);
                TEST_ASSERT_EQUAL_UINT16(expectation.pin, ctx.pin);
            }
        }

        void clk_enable_disables_the_peripheral_clock() {
            TEST_ASSERT_EQUAL(HAL_OK, adcx_clk_enable(ADC1, true));
            TEST_ASSERT_TRUE(RCC->APB2ENR & RCC_APB2ENR_ADC1EN);

            TEST_ASSERT_EQUAL(HAL_OK, adcx_clk_enable(ADC1, false));
            TEST_ASSERT_FALSE(RCC->APB2ENR & RCC_APB2ENR_ADC1EN);

            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, adcx_clk_enable(nullptr, false));

            TEST_ASSERT_EQUAL(HAL_OK, adcx_clk_enable(ADC1, true)); // leave enabled for the rest of the suite
        }

        void regular_group_cont_start_rejects_bad_trigger_polarity() {
            reset_to_baseline();
            constexpr adc_channel_t channel = ADC_CHANNEL_0;
            std::array<uint16_t, 4> buffer{};

            const adc_continuous_config_t config = {
                .channels         = {.sequence = &channel, .num_of_channels = 1},
                .trigger          = ADC_RG_TRIGGER_EXTI_LINE_11, // hardware trigger...
                .trigger_polarity = ADC_POLARITY_NONE,           // ...but no edge given
                .buffer_0         = buffer.data(),
                .buffer_1         = nullptr,
                .buffer_size      = buffer.size(),
                .priority         = DMA_PRIORITY_LOW,
                .circular_mode    = DMA_MODE_ONESHOT,
                .callbacks        = {},
            };
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, adc_regular_group_cont_start_conv(ADC1, &config));
        }

        void injected_group_start_rejects_bad_trigger_polarity() {
            reset_to_baseline();
            constexpr adc_channel_t channel = ADC_CHANNEL_0;

            adc_injected_group_config_t config{};
            config.channels         = {.sequence = &channel, .num_of_channels = 1};
            config.trigger          = ADC_JG_TRIGGER_EXTI_LINE_15;
            config.trigger_polarity = ADC_POLARITY_NONE;

            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, adc_injected_group_start_conv(ADC1, &config));
        }

        void regular_group_cont_start_rejects_reentry_while_active() {
            reset_to_baseline();
            adc_enable_nvic_irq(true);

            constexpr adc_channel_t channel = ADC_CHANNEL_0;
            adc_configure_channel(channel);
            std::array<uint16_t, 64> buffer{};

            const adc_continuous_config_t config = {
                .channels         = {.sequence = &channel, .num_of_channels = 1},
                .trigger          = ADC_RG_TRIGGER_SOFTWARE,
                .trigger_polarity = ADC_POLARITY_NONE,
                .buffer_0         = buffer.data(),
                .buffer_1         = nullptr,
                .buffer_size      = buffer.size(),
                .priority         = DMA_PRIORITY_LOW,
                .circular_mode    = DMA_MODE_CIRCULAR,
                .callbacks        = {},
            };
            TEST_ASSERT_EQUAL(HAL_OK, adc_regular_group_cont_start_conv(ADC1, &config));

            // Calling again while the stream is still enabled must not silently reconfigure over a live stream
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_STATE, adc_regular_group_cont_start_conv(ADC1, &config));

            TEST_ASSERT_EQUAL(HAL_OK, adc_regular_group_cont_end_conv(ADC1));
            adc_enable_nvic_irq(false);
            reset_to_baseline();
        }

        void injected_group_offsets_are_written_to_the_jofr_registers() {
            reset_to_baseline();

            constexpr auto CHANNELS = std::array{ADC_CHANNEL_0, ADC_CHANNEL_1, ADC_CHANNEL_2, ADC_CHANNEL_4};
            for (const auto channel : CHANNELS) {
                adc_configure_channel(channel);
            }

            adc_injected_group_config_t config{};
            config.channels         = {.sequence = CHANNELS.data(), .num_of_channels = CHANNELS.size()};
            config.trigger          = ADC_JG_TRIGGER_SOFTWARE;
            config.trigger_polarity = ADC_POLARITY_NONE;

            // last one deliberately > 12 bits, to verify masking
            constexpr std::array<uint16_t, CHANNELS.size()> offsets = {100U, 200U, 300U, 0x1234U};
            memcpy(config.offsets, offsets.data(), sizeof(config.offsets));

            TEST_ASSERT_EQUAL(HAL_OK, adc_injected_group_start_conv(ADC1, &config));

            TEST_ASSERT_EQUAL_UINT32(offsets[0], (ADC1->JOFR1 & ADC_JOFR1_JOFFSET1) >> ADC_JOFR1_JOFFSET1_Pos);
            TEST_ASSERT_EQUAL_UINT32(offsets[1], (ADC1->JOFR2 & ADC_JOFR2_JOFFSET2) >> ADC_JOFR2_JOFFSET2_Pos);
            TEST_ASSERT_EQUAL_UINT32(offsets[2], (ADC1->JOFR3 & ADC_JOFR3_JOFFSET3) >> ADC_JOFR3_JOFFSET3_Pos);
            TEST_ASSERT_EQUAL_UINT32(offsets[3] & 0xFFFU, (ADC1->JOFR4 & ADC_JOFR4_JOFFSET4) >> ADC_JOFR4_JOFFSET4_Pos);

            TEST_ASSERT_TRUE_MESSAGE(wait_until([]() {
                                         return (ADC1->SR & ADC_SR_JEOC);
                                     }),
                                     "Injected conversion never finished");

            std::array<uint16_t, CHANNELS.size()> result{};
            TEST_ASSERT_EQUAL(HAL_OK, adc_injected_group_get_result(ADC1, result.data(), result.size()));

            reset_to_baseline();
        }

        void continuous_conversion_overrun_ends_the_conversion_and_calls_back() {
            reset_to_baseline();
            adc_enable_nvic_irq(true);

            constexpr adc_channel_t channel = ADC_CHANNEL_0;
            adc_configure_channel(channel);

            std::array<uint16_t, 4096> buffer{};
            buffer.fill(UINT16_MAX);

            s_overrun_fired = false;

            const adc_continuous_config_t config = {
                .channels         = {.sequence = &channel, .num_of_channels = 1},
                .trigger          = ADC_RG_TRIGGER_SOFTWARE,
                .trigger_polarity = ADC_POLARITY_NONE,
                .buffer_0         = buffer.data(),
                .buffer_1         = nullptr,
                .buffer_size      = buffer.size(),
                .priority         = DMA_PRIORITY_LOW,
                .circular_mode    = DMA_MODE_CIRCULAR,
                .callbacks =
                    {
                        .on_buffer_full       = nullptr,
                        .on_transfer_error    = nullptr,
                        .on_direct_mode_error = nullptr,
                        .on_data_overrun      = overrun_cb,
                        .user                 = nullptr,
                    },
            };
            TEST_ASSERT_EQUAL(HAL_OK, adc_regular_group_cont_start_conv(ADC1, &config));

            // Let a few conversions run, then disable the DMA stream directly so the
            // next EOC has no consumer for DR -> forces a legitimate OVR
            delay_us(50);
            DMA2_Stream0->CR &= ~DMA_SxCR_EN;

            TEST_ASSERT_TRUE_MESSAGE(wait_until([]() {
                                         return s_overrun_fired;
                                     }),
                                     "Data overrun callback never fired");

            // On overrun, adc_regular_group_cont_end_conv(...) runs internally before the
            // callback fires, so CONT/DMA and the stream should already be torn down
            TEST_ASSERT_FALSE(ADC1->CR2 & ADC_CR2_CONT);
            TEST_ASSERT_FALSE(ADC1->CR2 & ADC_CR2_DMA);
            TEST_ASSERT_FALSE(DMA2_Stream0->CR & DMA_SxCR_EN);

            adc_enable_nvic_irq(false);
            reset_to_baseline();
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
            TEST_ASSERT_TRUE(temp_c > 20 && temp_c < 40);
            LOGI(TAG, "[voltage_and_temperature_math_are_internally_consistent] Temperature: %.3fC", (double)temp_c);
        }

        void value_right_aligned_rejects_unknown_resolution() {
            reset_to_baseline();

            float          voltage          = 0.0F;
            constexpr auto bogus_resolution = static_cast<adc_resolution_t>(0xFF);
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, adc_get_value_right_aligned(ADC1, 20, bogus_resolution, &voltage));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, adc_get_value_right_aligned(ADC1, 20, ADC_RES_12_BITS, nullptr));
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
                    .trigger          = ADC_RG_TRIGGER_SOFTWARE,
                    .trigger_polarity = ADC_POLARITY_NONE,
                    .buffer_0         = buffer.data(),
                    .buffer_1         = nullptr,
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
                .trigger          = ADC_RG_TRIGGER_SOFTWARE,
                .trigger_polarity = ADC_POLARITY_NONE,
                .buffer_0         = buffer.data(),
                .buffer_1         = nullptr,
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
            no_buffer.buffer_0 = nullptr;
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, adc_regular_group_cont_start_conv(ADC1, &no_buffer));

            auto missing_second_buffer          = base;
            missing_second_buffer.circular_mode = DMA_MODE_DOUBLE_BUFFERS;
            missing_second_buffer.buffer_1      = nullptr;
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, adc_regular_group_cont_start_conv(ADC1, &missing_second_buffer));
        }

        void continuous_conversion_fills_the_buffer_via_dma_and_calls_back() {
            reset_to_baseline();

            constexpr auto          CHANNELS = std::array{ADC_CHANNEL_0, ADC_CHANNEL_1};
            std::array<uint16_t, 8> buffer{};

            s_cont_done = false;

            const adc_continuous_config_t config = {
                .channels         = {.sequence = CHANNELS.data(), .num_of_channels = CHANNELS.size()},
                .trigger          = ADC_RG_TRIGGER_SOFTWARE,
                .trigger_polarity = ADC_POLARITY_NONE,
                .buffer_0         = buffer.data(),
                .buffer_1         = nullptr,
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

                const adc_injected_group_config_t config = {
                    .channels         = {.sequence = channels.data(), .num_of_channels = count},
                    .trigger          = ADC_JG_TRIGGER_SOFTWARE,
                    .trigger_polarity = ADC_POLARITY_NONE,
                    .offsets          = {},
                    .on_conv_complete = nullptr,
                    .arg              = nullptr,
                };
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
                LOGI(TAG, "-----------------------------");
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
            config.trigger  = ADC_JG_TRIGGER_SOFTWARE;

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
            config.trigger          = ADC_JG_TRIGGER_SOFTWARE;
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

            uint16_t result = 0;
            TEST_ASSERT_EQUAL(HAL_OK, adc_injected_group_get_result(ADC1, &result, 1));
            TEST_ASSERT_TRUE(result <= 0xFFFU);

            // The JEOC flag should be cleared now by reading the result
            TEST_ASSERT_FALSE(ADC1->SR & ADC_SR_JEOC);

            adc_enable_nvic_irq(false);
            reset_to_baseline();
        }

        void injected_group_interrupts_regular_group_without_corruption() {
            reset_to_baseline();
            adc_enable_nvic_irq(true);

            // Free running regular group on channel 0
            constexpr adc_channel_t regular_channel = ADC_CHANNEL_0;
            adc_configure_channel(regular_channel);

            std::array<uint16_t, 64> reg_buffer{};
            reg_buffer.fill(UINT16_MAX);

            s_cont_done                              = false;
            const adc_continuous_config_t reg_config = {
                .channels         = {.sequence = &regular_channel, .num_of_channels = 1},
                .trigger          = ADC_RG_TRIGGER_SOFTWARE,
                .trigger_polarity = ADC_POLARITY_NONE,
                .buffer_0         = reg_buffer.data(),
                .buffer_1         = nullptr,
                .buffer_size      = reg_buffer.size(),
                .priority         = DMA_PRIORITY_LOW,
                .circular_mode    = DMA_MODE_CIRCULAR,
                .callbacks =
                    {
                        .on_buffer_full       = cont_done_cb,
                        .on_transfer_error    = nullptr,
                        .on_direct_mode_error = nullptr,
                        .on_data_overrun      = nullptr,
                        .user                 = nullptr,
                    },
            };
            TEST_ASSERT_EQUAL(HAL_OK, adc_regular_group_cont_start_conv(ADC1, &reg_config));

            // Let it complete at least one full cycle before interrupting it
            TEST_ASSERT_TRUE_MESSAGE(wait_until([]() {
                                         return s_cont_done;
                                     }),
                                     "Regular group never completed a cycle before injection");

            // Fire an injected conversion on a different channel while the regular group is live
            constexpr adc_channel_t injected_channel = ADC_CHANNEL_1;
            adc_configure_channel(injected_channel);

            s_injected_done                             = false;
            const adc_injected_group_config_t jg_config = {
                .channels         = {.sequence = &injected_channel, .num_of_channels = 1},
                .trigger          = ADC_JG_TRIGGER_SOFTWARE,
                .trigger_polarity = ADC_POLARITY_NONE,
                .offsets          = {},
                .on_conv_complete = injected_done_cb,
                .arg              = nullptr,
            };
            TEST_ASSERT_EQUAL(HAL_OK, adc_injected_group_start_conv(ADC1, &jg_config));
            TEST_ASSERT_TRUE_MESSAGE(wait_until([]() {
                                         return s_injected_done;
                                     }),
                                     "Injected group never completed while the regular group was running");

            uint16_t injected_result = 0;
            TEST_ASSERT_EQUAL(HAL_OK, adc_injected_group_get_result(ADC1, &injected_result, 1));
            TEST_ASSERT_TRUE(injected_result <= 0xFFFU);

            // The injected path must not have torn down the regular group's continuous DMA state
            TEST_ASSERT_TRUE(ADC1->CR2 & ADC_CR2_CONT);
            TEST_ASSERT_TRUE(ADC1->CR2 & ADC_CR2_DMA);

            // And the regular group must actually keep converting afterward, not just leave stale bits set
            s_cont_done = false;
            TEST_ASSERT_TRUE_MESSAGE(wait_until([]() {
                                         return s_cont_done;
                                     }),
                                     "Regular group did not resume after the injected conversion");

            for (const auto sample : reg_buffer) {
                TEST_ASSERT_TRUE(sample <= 0xFFFU);
            }

            TEST_ASSERT_EQUAL(HAL_OK, adc_regular_group_cont_end_conv(ADC1));
            adc_enable_nvic_irq(false);
            reset_to_baseline();
        }

        void regular_group_runs_in_circular_and_double_buffering_mode() {
            reset_to_baseline();
            adc_enable_nvic_irq(true);

            constexpr auto CHANNELS = std::array{ADC_CHANNEL_0, ADC_CHANNEL_1, ADC_CHANNEL_2};
            for (const auto channel : CHANNELS) {
                adc_configure_channel(channel);
            }

            // Double buffering mode
            // Big enough buffers so it takes a bit of time to get filled up
            std::array<uint16_t, CHANNELS.size() * 4096> buffer_0{};
            std::array<uint16_t, buffer_0.size()>        buffer_1{};

            buffer_0.fill(UINT16_MAX);
            buffer_1.fill(UINT16_MAX);

            const adc_continuous_config_t config = {
                .channels         = {.sequence = CHANNELS.data(), .num_of_channels = CHANNELS.size()},
                .trigger          = ADC_RG_TRIGGER_SOFTWARE,
                .trigger_polarity = ADC_POLARITY_NONE,
                .buffer_0         = buffer_0.data(),
                .buffer_1         = buffer_1.data(),
                .buffer_size      = buffer_0.size(),
                .priority         = DMA_PRIORITY_LOW,
                .circular_mode    = DMA_MODE_DOUBLE_BUFFERS,
                .callbacks =
                    {
                        .on_buffer_full       = cont_done_cb,
                        .on_transfer_error    = nullptr,
                        .on_direct_mode_error = nullptr,
                        .on_data_overrun      = nullptr,
                        .user                 = nullptr,
                    },
            };

            TEST_ASSERT_EQUAL(HAL_OK, adc_regular_group_cont_start_conv(ADC1, &config));

            // Let the buffers get filled a good number of times
            for (size_t i = 0; i < 20; i++) {
                // Wait for the first buffer (buffer_0) to be filled
                s_cont_done = false;
                TEST_ASSERT_TRUE_MESSAGE(wait_until([]() {
                                             return s_cont_done;
                                         }),
                                         "Continuous DMA conversion: buffer 0 never filled");
                TEST_ASSERT_EQUAL(0, s_filled_buffer_idx);

                // Wait for the second buffer (buffer_1) to be filled
                s_cont_done = false;
                TEST_ASSERT_TRUE_MESSAGE(wait_until([]() {
                                             return s_cont_done;
                                         }),
                                         "Continuous DMA conversion: buffer 1 never filled");
                TEST_ASSERT_EQUAL(1, s_filled_buffer_idx);
            }

            TEST_ASSERT_EQUAL(HAL_OK, adc_regular_group_cont_end_conv(ADC1));

            for (const auto sample : buffer_0) {
                TEST_ASSERT_TRUE(sample <= 0xFFFU);
            }
            for (const auto sample : buffer_1) {
                TEST_ASSERT_TRUE(sample <= 0xFFFU);
            }

            // Pure circular mode (buffer_0 acts as a ring buffer)
            auto circ_config          = config;
            circ_config.circular_mode = DMA_MODE_CIRCULAR;
            circ_config.buffer_1      = nullptr;

            buffer_0.fill(UINT16_MAX);
            TEST_ASSERT_EQUAL(HAL_OK, adc_regular_group_cont_start_conv(ADC1, &circ_config));

            for (size_t i = 0; i < 20; i++) {
                s_cont_done = false;
                TEST_ASSERT_TRUE_MESSAGE(wait_until([]() {
                                             return s_cont_done;
                                         }),
                                         "Continuous DMA conversion: buffer 0 never filled");

                // s_filled_buffer_idx contains garbage when not in double buffer mode, but that value should be constant
                static const uint8_t filled_buffer_idx = s_filled_buffer_idx;
                TEST_ASSERT_EQUAL_UINT8(filled_buffer_idx, s_filled_buffer_idx);
            }

            TEST_ASSERT_EQUAL(HAL_OK, adc_regular_group_cont_end_conv(ADC1));

            for (const auto sample : buffer_0) {
                TEST_ASSERT_TRUE(sample <= 0xFFFU);
            }

            adc_enable_nvic_irq(false);
            reset_to_baseline();
        }

        void external_trigger_source_works_for_regular_and_injected_groups() {
            reset_to_baseline();
            adc_enable_nvic_irq(true);

            constexpr auto CHANNELS = std::array{ADC_CHANNEL_0, ADC_CHANNEL_1, ADC_CHANNEL_2, ADC_CHANNEL_4};
            for (const auto channel : CHANNELS) {
                adc_configure_channel(channel);
            }

            std::array<uint16_t, CHANNELS.size() * 1024> buffer_0{};
            buffer_0.fill(UINT16_MAX);

            // The regular group: falling edge
            const adc_continuous_config_t config = {
                .channels         = {.sequence = CHANNELS.data(), .num_of_channels = CHANNELS.size()},
                .trigger          = ADC_RG_TRIGGER_EXTI_LINE_11,
                .trigger_polarity = ADC_POLARITY_FALLING_EDGE,
                .buffer_0         = buffer_0.data(),
                .buffer_1         = nullptr,
                .buffer_size      = buffer_0.size(),
                .priority         = DMA_PRIORITY_LOW,
                .circular_mode    = DMA_MODE_CIRCULAR,
                .callbacks =
                    {
                        .on_buffer_full       = cont_done_cb,
                        .on_transfer_error    = nullptr,
                        .on_direct_mode_error = nullptr,
                        .on_data_overrun      = nullptr,
                        .user                 = nullptr,
                    },
            };
            TEST_ASSERT_EQUAL(HAL_OK, adc_regular_group_cont_start_conv(ADC1, &config));
            TEST_ASSERT_EQUAL_UINT32(ADC_RG_TRIGGER_EXTI_LINE_11, ((ADC1->CR2 & ADC_CR2_EXTSEL) >> ADC_CR2_EXTSEL_Pos));
            TEST_ASSERT_EQUAL_UINT32(ADC_POLARITY_FALLING_EDGE, ((ADC1->CR2 & ADC_CR2_EXTEN) >> ADC_CR2_EXTEN_Pos));

            // Conversion should not be started yet. It waits for a falling edge on GPIO_PIN_11 on any port
            TEST_ASSERT_FALSE(ADC1->SR & ADC_SR_STRT);

            // Generate an EXTI request on EXTI line 11
            gpiox_clk_enable(GPIOA, true);
            gpio_set_output(GPIOA, GPIO_PIN_11);
            gpio_enable_pullups(GPIOA, GPIO_PIN_11, true);
            gpio_set_interrupt(GPIOA, GPIO_PIN_11, GPIO_FALLING_EDGE, nullptr, nullptr);

            // Get a falling edge on PA11 which is configured as EXTI line 11
            gpio_set_level(GPIOA, GPIO_PIN_11, true);
            delay_us(1);
            gpio_set_level(GPIOA, GPIO_PIN_11, false);
            delay_us(1);

            // The conversion should be started now, and everything should work as normal
            TEST_ASSERT_TRUE(ADC1->SR & ADC_SR_STRT);

            for (size_t i = 0; i < 20; i++) {
                s_cont_done = false;
                TEST_ASSERT_TRUE(wait_until([]() {
                    return s_cont_done;
                }));
            }

            TEST_ASSERT_EQUAL(HAL_OK, adc_regular_group_cont_end_conv(ADC1));
            TEST_ASSERT_FALSE(ADC1->SR & ADC_SR_STRT);
            TEST_ASSERT_EQUAL_UINT32(0, ((ADC1->CR2 & ADC_CR2_EXTSEL) >> ADC_CR2_EXTSEL_Pos));
            TEST_ASSERT_EQUAL_UINT32(0, ((ADC1->CR2 & ADC_CR2_EXTEN) >> ADC_CR2_EXTEN_Pos));

            for (const auto sample : buffer_0) {
                TEST_ASSERT_TRUE(sample <= 0xFFFU);
            }

            // The injected group: rising edge
            const adc_injected_group_config_t jg_config = {
                .channels         = {.sequence = CHANNELS.data(), .num_of_channels = CHANNELS.size()},
                .trigger          = ADC_JG_TRIGGER_EXTI_LINE_15,
                .trigger_polarity = ADC_POLARITY_RISING_EDGE,
                .offsets          = {},
                .on_conv_complete = injected_done_cb,
                .arg              = nullptr,
            };
            TEST_ASSERT_EQUAL(HAL_OK, adc_injected_group_start_conv(ADC1, &jg_config));
            TEST_ASSERT_EQUAL_UINT32(ADC_JG_TRIGGER_EXTI_LINE_15, ((ADC1->CR2 & ADC_CR2_JEXTSEL) >> ADC_CR2_JEXTSEL_Pos));
            TEST_ASSERT_EQUAL_UINT32(ADC_POLARITY_RISING_EDGE, ((ADC1->CR2 & ADC_CR2_JEXTEN) >> ADC_CR2_JEXTEN_Pos));

            // Conversion should not be started yet. It waits for a rising edge on GPIO_PIN_15 on any port
            TEST_ASSERT_FALSE(ADC1->SR & ADC_SR_JSTRT);
            s_injected_done = false;

            // Generate an EXTI request on EXTI line 15
            gpio_set_output(GPIOA, GPIO_PIN_15);
            gpio_enable_pullups(GPIOA, GPIO_PIN_15, true);
            gpio_set_interrupt(GPIOA, GPIO_PIN_15, GPIO_RISING_EDGE, nullptr, nullptr);

            // Get a rising edge on PA15 which is configured as EXTI line 15
            gpio_set_level(GPIOA, GPIO_PIN_15, false);
            delay_us(1);
            gpio_set_level(GPIOA, GPIO_PIN_15, true);
            delay_us(1);

            // The conversion should be started now, and everything should work as normal
            TEST_ASSERT_TRUE(ADC1->SR & ADC_SR_JSTRT);

            TEST_ASSERT_TRUE(wait_until([]() {
                return s_injected_done;
            }));
            TEST_ASSERT_TRUE(ADC1->SR & ADC_SR_JEOC);

            std::array<uint16_t, CHANNELS.size()> jg_buffer{};
            jg_buffer.fill(UINT16_MAX);
            static_assert(jg_buffer.size() <= MAX_INJECTED_CHANNELS);

            TEST_ASSERT_EQUAL(HAL_OK, adc_injected_group_get_result(ADC1, jg_buffer.data(), jg_buffer.size()));
            TEST_ASSERT_FALSE(ADC1->SR & ADC_SR_JEOC);
            TEST_ASSERT_FALSE(ADC1->SR & ADC_SR_JSTRT);
            TEST_ASSERT_EQUAL_UINT32(0, ((ADC1->CR2 & ADC_CR2_JEXTSEL) >> ADC_CR2_JEXTSEL_Pos));
            TEST_ASSERT_EQUAL_UINT32(0, ((ADC1->CR2 & ADC_CR2_JEXTEN) >> ADC_CR2_JEXTEN_Pos));

            for (const auto sample : jg_buffer) {
                TEST_ASSERT_TRUE(sample <= 0xFFFU);
                LOGI(TAG, "Injected sample: %u", sample);
            }

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
            config.max_adc_value             = 4095;
            config.min_adc_value             = 4094; // Deliberately high so the watchdog fires
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
        RUN_TEST(channel_get_gpio_matches_the_pin_map);
        RUN_TEST(clk_enable_disables_the_peripheral_clock);
        RUN_TEST(regular_group_cont_start_rejects_bad_trigger_polarity);
        RUN_TEST(injected_group_start_rejects_bad_trigger_polarity);
        RUN_TEST(regular_group_cont_start_rejects_reentry_while_active);
        RUN_TEST(injected_group_offsets_are_written_to_the_jofr_registers);
        RUN_TEST(continuous_conversion_overrun_ends_the_conversion_and_calls_back);
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
        RUN_TEST(injected_group_interrupts_regular_group_without_corruption);
        RUN_TEST(regular_group_runs_in_circular_and_double_buffering_mode);
        RUN_TEST(external_trigger_source_works_for_regular_and_injected_groups);
        RUN_TEST(analog_watchdog_validates_thresholds_and_monitor_flags);

        UNITY_END();
        LOGI(TAG, "Done with all tests on the ADC driver");
    }

} // namespace test::adc
