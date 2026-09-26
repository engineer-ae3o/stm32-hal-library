#include "stm32f411xe.h"
#include "Unity/unity.h"

#include "drivers/dma_types.h"
#include "utils/common.h"
#include "utils/memcpy.h"
#include "drivers/dma.h"
#include "test/dma.hpp"
#include "utils/err.h"
#include "utils/log.h"

#include <array>
#include <cstdint>
#include <utility>
#include <cstring>
#include <string_view>


namespace test::dma {

    namespace {

        constexpr const char* TAG = "DMA_Test";

        // An otherwise unused stream reserved for tests that need to drive a real transfer
        DMA_Stream_TypeDef* const SCRATCH_STREAM = DMA2_Stream1;

        struct stream_expectation_t {
            DMA_Stream_TypeDef* stream;
            DMA_TypeDef*        controller;
            uint32_t            stream_number;
            IRQn_Type           irq_type;
        };

        const std::array<stream_expectation_t, 16> ALL_STREAMS = {{
            {.stream = DMA1_Stream0, .controller = DMA1, .stream_number = 0, .irq_type = DMA1_Stream0_IRQn},
            {.stream = DMA1_Stream1, .controller = DMA1, .stream_number = 1, .irq_type = DMA1_Stream1_IRQn},
            {.stream = DMA1_Stream2, .controller = DMA1, .stream_number = 2, .irq_type = DMA1_Stream2_IRQn},
            {.stream = DMA1_Stream3, .controller = DMA1, .stream_number = 3, .irq_type = DMA1_Stream3_IRQn},
            {.stream = DMA1_Stream4, .controller = DMA1, .stream_number = 4, .irq_type = DMA1_Stream4_IRQn},
            {.stream = DMA1_Stream5, .controller = DMA1, .stream_number = 5, .irq_type = DMA1_Stream5_IRQn},
            {.stream = DMA1_Stream6, .controller = DMA1, .stream_number = 6, .irq_type = DMA1_Stream6_IRQn},
            {.stream = DMA1_Stream7, .controller = DMA1, .stream_number = 7, .irq_type = DMA1_Stream7_IRQn},
            {.stream = DMA2_Stream0, .controller = DMA2, .stream_number = 0, .irq_type = DMA2_Stream0_IRQn},
            {.stream = DMA2_Stream1, .controller = DMA2, .stream_number = 1, .irq_type = DMA2_Stream1_IRQn},
            {.stream = DMA2_Stream2, .controller = DMA2, .stream_number = 2, .irq_type = DMA2_Stream2_IRQn},
            {.stream = DMA2_Stream3, .controller = DMA2, .stream_number = 3, .irq_type = DMA2_Stream3_IRQn},
            {.stream = DMA2_Stream4, .controller = DMA2, .stream_number = 4, .irq_type = DMA2_Stream4_IRQn},
            {.stream = DMA2_Stream5, .controller = DMA2, .stream_number = 5, .irq_type = DMA2_Stream5_IRQn},
            {.stream = DMA2_Stream6, .controller = DMA2, .stream_number = 6, .irq_type = DMA2_Stream6_IRQn},
            {.stream = DMA2_Stream7, .controller = DMA2, .stream_number = 7, .irq_type = DMA2_Stream7_IRQn},
        }};

        // Helpers
        hal_err_t configure_scratch_m2m(const uint32_t* src, uint32_t* dst, uint16_t count, bool enable) {
            const dma_stream_config_t config = {
                .deconfigure   = false,
                .enable_stream = enable,

                .per_addr_increment = true,
                .mem_addr_increment = true,

                .tc_irq_enable  = false,
                .ht_irq_enable  = false,
                .te_irq_enable  = false,
                .dme_irq_enable = false,
                .fe_irq_enable  = false,

                .mode            = DMA_MODE_FIFO,
                .priority        = DMA_PRIORITY_LOW,
                .direction       = DMA_DIR_M2M,
                .per_data_size   = DMA_SIZE_WORD,
                .mem_data_size   = DMA_SIZE_WORD,
                .circular_mode   = DMA_MODE_ONESHOT,
                .flow_controller = DMA_FLOW_CONTROLLER_DMA,

                .buffer_size       = count,
                .channel           = 0,
                .nvic_irq_priority = M2M_DMA_NVIC_IRQ_PRIORITY,

                .per_addr  = src,
                .mem_buf_0 = dst,
                .mem_buf_1 = nullptr,
            };
            return dma_configure_stream(SCRATCH_STREAM, &config);
        }

        bool wait_for_tc_flag() {
            uint32_t timeout = TIMEOUT;
            // This assumes DMA2, stream 1. Should match SCRATCH_STREAM
            while (!(DMA2->LISR & DMA_LISR_TCIF1) && --timeout);
            return (DMA2->LISR & DMA_LISR_TCIF1) != 0;
        }

        // TESTS
        void clk_enable_toggles_only_the_targeted_controller() {
            TEST_ASSERT_EQUAL(HAL_OK, dmax_clk_enable(DMA1, true));
            TEST_ASSERT_TRUE(RCC->AHB1ENR & RCC_AHB1ENR_DMA1EN);
            TEST_ASSERT_EQUAL(HAL_OK, dmax_clk_enable(DMA1, false));
            TEST_ASSERT_FALSE(RCC->AHB1ENR & RCC_AHB1ENR_DMA1EN);

            TEST_ASSERT_EQUAL(HAL_OK, dmax_clk_enable(DMA2, true));
            TEST_ASSERT_TRUE(RCC->AHB1ENR & RCC_AHB1ENR_DMA2EN);
            // Leave DMA2 clock enabled as its used for the other M2M tests

            const uint32_t before = RCC->AHB1ENR;
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, dmax_clk_enable(nullptr, true));
            TEST_ASSERT_EQUAL_UINT32(before, RCC->AHB1ENR);
        }

        void stream_info_lookup_covers_every_stream() {
            for (const auto& expected : ALL_STREAMS) {
                dma_stream_info_t info;
                TEST_ASSERT_EQUAL(HAL_OK, dma_get_stream_info(expected.stream, &info));
                TEST_ASSERT_EQUAL_PTR(expected.stream, info.stream);
                TEST_ASSERT_EQUAL_PTR(expected.controller, info.controller);
                TEST_ASSERT_EQUAL_UINT32(expected.stream_number, info.stream_number);
                TEST_ASSERT_EQUAL(expected.irq_type, info.nvic_irq_type);
            }

            dma_stream_info_t info;
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, dma_get_stream_info(nullptr, &info));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, dma_get_stream_info(DMA1_Stream0, nullptr));
        }

        void stream_flags_pick_the_right_registers_and_masks() {
            struct flag_expectation_t {
                uint32_t stream_number;
                uint32_t tc_mask;
                uint32_t te_mask;
                uint32_t ht_mask;
                uint32_t dme_mask;
                uint32_t fe_mask;
                bool     is_low;
            };

            constexpr std::array<flag_expectation_t, 8> EXPECTATIONS = {{
                {
                    .stream_number = 0,
                    .tc_mask       = DMA_LISR_TCIF0,
                    .te_mask       = DMA_LISR_TEIF0,
                    .ht_mask       = DMA_LISR_HTIF0,
                    .dme_mask      = DMA_LISR_DMEIF0,
                    .fe_mask       = DMA_LISR_FEIF0,
                    .is_low        = true,
                },
                {
                    .stream_number = 1,
                    .tc_mask       = DMA_LISR_TCIF1,
                    .te_mask       = DMA_LISR_TEIF1,
                    .ht_mask       = DMA_LISR_HTIF1,
                    .dme_mask      = DMA_LISR_DMEIF1,
                    .fe_mask       = DMA_LISR_FEIF1,
                    .is_low        = true,
                },
                {
                    .stream_number = 2,
                    .tc_mask       = DMA_LISR_TCIF2,
                    .te_mask       = DMA_LISR_TEIF2,
                    .ht_mask       = DMA_LISR_HTIF2,
                    .dme_mask      = DMA_LISR_DMEIF2,
                    .fe_mask       = DMA_LISR_FEIF2,
                    .is_low        = true,
                },
                {
                    .stream_number = 3,
                    .tc_mask       = DMA_LISR_TCIF3,
                    .te_mask       = DMA_LISR_TEIF3,
                    .ht_mask       = DMA_LISR_HTIF3,
                    .dme_mask      = DMA_LISR_DMEIF3,
                    .fe_mask       = DMA_LISR_FEIF3,
                    .is_low        = true,
                },
                {
                    .stream_number = 4,
                    .tc_mask       = DMA_HISR_TCIF4,
                    .te_mask       = DMA_HISR_TEIF4,
                    .ht_mask       = DMA_HISR_HTIF4,
                    .dme_mask      = DMA_HISR_DMEIF4,
                    .fe_mask       = DMA_HISR_FEIF4,
                    .is_low        = false,
                },
                {
                    .stream_number = 5,
                    .tc_mask       = DMA_HISR_TCIF5,
                    .te_mask       = DMA_HISR_TEIF5,
                    .ht_mask       = DMA_HISR_HTIF5,
                    .dme_mask      = DMA_HISR_DMEIF5,
                    .fe_mask       = DMA_HISR_FEIF5,
                    .is_low        = false,
                },
                {
                    .stream_number = 6,
                    .tc_mask       = DMA_HISR_TCIF6,
                    .te_mask       = DMA_HISR_TEIF6,
                    .ht_mask       = DMA_HISR_HTIF6,
                    .dme_mask      = DMA_HISR_DMEIF6,
                    .fe_mask       = DMA_HISR_FEIF6,
                    .is_low        = false,
                },
                {
                    .stream_number = 7,
                    .tc_mask       = DMA_HISR_TCIF7,
                    .te_mask       = DMA_HISR_TEIF7,
                    .ht_mask       = DMA_HISR_HTIF7,
                    .dme_mask      = DMA_HISR_DMEIF7,
                    .fe_mask       = DMA_HISR_FEIF7,
                    .is_low        = false,
                },
            }};

            for (const auto& expected : EXPECTATIONS) {
                dma_stream_flags_t flags;
                TEST_ASSERT_EQUAL(HAL_OK, dma_get_stream_flags(DMA1, &flags, expected.stream_number));

                TEST_ASSERT_EQUAL_UINT32(expected.tc_mask, flags.tc_mask);
                TEST_ASSERT_EQUAL_UINT32(expected.te_mask, flags.te_mask);
                TEST_ASSERT_EQUAL_UINT32(expected.ht_mask, flags.ht_mask);
                TEST_ASSERT_EQUAL_UINT32(expected.fe_mask, flags.fe_mask);
                TEST_ASSERT_EQUAL_UINT32(expected.dme_mask, flags.dme_mask);

                if (expected.is_low) {
                    TEST_ASSERT_EQUAL_PTR(&DMA1->LISR, flags.irq_status_register);
                    TEST_ASSERT_EQUAL_PTR(&DMA1->LIFCR, flags.irq_clear_register);
                } else {
                    TEST_ASSERT_EQUAL_PTR(&DMA1->HISR, flags.irq_status_register);
                    TEST_ASSERT_EQUAL_PTR(&DMA1->HIFCR, flags.irq_clear_register);
                }
            }

            // Test the invalid arguments catching
            dma_stream_flags_t flags;
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, dma_get_stream_flags(nullptr, &flags, 0));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, dma_get_stream_flags(DMA1, &flags, 8));
        }

        void low_level_setters_are_null_safe_and_cover_every_enum_value() {
            // Passing nullptr to any of these should be a no-op
            dma_set_channel(nullptr, 3);
            dma_set_direct_mode(nullptr, true);
            dma_set_trans_length(nullptr, 10);
            dma_set_direction(nullptr, DMA_DIR_M2P);
            dma_set_increment(nullptr, true, true);
            dma_set_flow_controller(nullptr, true);
            dma_set_stream_priority(nullptr, DMA_PRIORITY_HIGH);
            dma_set_circular_mode(nullptr, DMA_MODE_DOUBLE_BUFFERS);
            dma_set_per_mem_size(nullptr, DMA_SIZE_BYTE, DMA_SIZE_WORD);
            dma_enable_irqs(nullptr, true, true, true, true);
            dma_set_addresses(nullptr, nullptr, nullptr, nullptr);

            DMA_Stream_TypeDef* const stream = SCRATCH_STREAM;

            for (uint8_t channel = 0; channel < 8; channel++) {
                dma_set_channel(stream, channel);
                TEST_ASSERT_EQUAL_UINT32(channel, (stream->CR & DMA_SxCR_CHSEL) >> DMA_SxCR_CHSEL_Pos);
            }

            dma_set_direct_mode(stream, true);
            TEST_ASSERT_FALSE(stream->FCR & DMA_SxFCR_DMDIS);
            dma_set_direct_mode(stream, false);
            TEST_ASSERT_TRUE(stream->FCR & DMA_SxFCR_DMDIS);

            dma_set_trans_length(stream, 1234);
            TEST_ASSERT_EQUAL_UINT32(1234, stream->NDTR);

            for (const auto direction : {DMA_DIR_M2M, DMA_DIR_M2P, DMA_DIR_P2M}) {
                dma_set_direction(stream, direction);
                TEST_ASSERT_EQUAL_UINT32(std::to_underlying(direction), (stream->CR & DMA_SxCR_DIR) >> DMA_SxCR_DIR_Pos);
            }

            dma_set_increment(stream, true, false);
            TEST_ASSERT_TRUE(stream->CR & DMA_SxCR_PINC);
            TEST_ASSERT_FALSE(stream->CR & DMA_SxCR_MINC);
            dma_set_increment(stream, false, true);
            TEST_ASSERT_FALSE(stream->CR & DMA_SxCR_PINC);
            TEST_ASSERT_TRUE(stream->CR & DMA_SxCR_MINC);

            // NOTE: The PFCTRL bit is forced by the hardware to 0 if the direction is DMA_DIR_M2M
            dma_set_flow_controller(stream, true);
            TEST_ASSERT_FALSE(stream->CR & DMA_SxCR_PFCTRL);
            dma_set_flow_controller(stream, false);
            TEST_ASSERT_TRUE(stream->CR & DMA_SxCR_PFCTRL);

            for (const auto priority : {DMA_PRIORITY_LOW, DMA_PRIORITY_MEDIUM, DMA_PRIORITY_HIGH, DMA_PRIORITY_VERY_HIGH}) {
                dma_set_stream_priority(stream, priority);
                TEST_ASSERT_EQUAL_UINT32(std::to_underlying(priority), (stream->CR & DMA_SxCR_PL) >> DMA_SxCR_PL_Pos);
            }

            dma_set_circular_mode(stream, DMA_MODE_DOUBLE_BUFFERS);
            TEST_ASSERT_TRUE(stream->CR & DMA_SxCR_CIRC);
            TEST_ASSERT_TRUE(stream->CR & DMA_SxCR_DBM);
            dma_set_circular_mode(stream, DMA_MODE_CIRCULAR);
            TEST_ASSERT_TRUE(stream->CR & DMA_SxCR_CIRC);
            TEST_ASSERT_FALSE(stream->CR & DMA_SxCR_DBM);
            dma_set_circular_mode(stream, DMA_MODE_ONESHOT);
            TEST_ASSERT_FALSE(stream->CR & DMA_SxCR_CIRC);
            TEST_ASSERT_FALSE(stream->CR & DMA_SxCR_DBM);

            constexpr auto SIZES = std::array{DMA_SIZE_BYTE, DMA_SIZE_HWORD, DMA_SIZE_WORD};
            for (const auto per : SIZES) {
                for (const auto mem : SIZES) {
                    dma_set_per_mem_size(stream, per, mem);
                    TEST_ASSERT_EQUAL_UINT32(std::to_underlying(per), (stream->CR & DMA_SxCR_PSIZE) >> DMA_SxCR_PSIZE_Pos);
                    TEST_ASSERT_EQUAL_UINT32(std::to_underlying(mem), (stream->CR & DMA_SxCR_MSIZE) >> DMA_SxCR_MSIZE_Pos);
                }
            }

            dma_enable_irqs(stream, true, false, true, false);
            TEST_ASSERT_TRUE(stream->CR & DMA_SxCR_TCIE);
            TEST_ASSERT_FALSE(stream->CR & DMA_SxCR_TEIE);
            TEST_ASSERT_TRUE(stream->CR & DMA_SxCR_HTIE);
            TEST_ASSERT_FALSE(stream->CR & DMA_SxCR_DMEIE);

            uint32_t per_addr = 0, mem0 = 0, mem1 = 0;
            dma_set_addresses(stream, &per_addr, &mem0, &mem1);
            TEST_ASSERT_EQUAL_UINT32(reinterpret_cast<uint32_t>(&per_addr), stream->PAR);
            TEST_ASSERT_EQUAL_UINT32(reinterpret_cast<uint32_t>(&mem0), stream->M0AR);
            TEST_ASSERT_EQUAL_UINT32(reinterpret_cast<uint32_t>(&mem1), stream->M1AR);

            // A NULL address argument must leave the corresponding register untouched, not zero it
            dma_set_addresses(stream, nullptr, nullptr, nullptr);
            TEST_ASSERT_EQUAL_UINT32(reinterpret_cast<uint32_t>(&per_addr), stream->PAR);
            TEST_ASSERT_EQUAL_UINT32(reinterpret_cast<uint32_t>(&mem0), stream->M0AR);
            TEST_ASSERT_EQUAL_UINT32(reinterpret_cast<uint32_t>(&mem1), stream->M1AR);

            stream->CR  = 0;
            stream->FCR = 0;
        }

        void configure_stream_invalid_arg_guards() {
            dma_stream_config_t config{};
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, dma_configure_stream(nullptr, &config));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, dma_configure_stream(SCRATCH_STREAM, nullptr));
        }

        void deconfigure_clears_addresses_length_and_irq() {
            uint32_t src = 0, dst = 0;

            TEST_ASSERT_EQUAL(HAL_OK, configure_scratch_m2m(&src, &dst, sizeof(src), false));
            TEST_ASSERT_EQUAL_UINT32(&src, SCRATCH_STREAM->PAR);
            TEST_ASSERT_EQUAL_UINT32(&dst, SCRATCH_STREAM->M0AR);
            TEST_ASSERT_EQUAL_UINT32(sizeof(src), SCRATCH_STREAM->NDTR);

            dma_stream_config_t deconf{};
            deconf.deconfigure = true;

            TEST_ASSERT_EQUAL(HAL_OK, dma_configure_stream(SCRATCH_STREAM, &deconf));
            TEST_ASSERT_EQUAL_UINT32(0, SCRATCH_STREAM->PAR);
            TEST_ASSERT_EQUAL_UINT32(0, SCRATCH_STREAM->M0AR);
            TEST_ASSERT_EQUAL_UINT32(0, SCRATCH_STREAM->M1AR);
            TEST_ASSERT_EQUAL_UINT32(0, SCRATCH_STREAM->NDTR);
            TEST_ASSERT_FALSE(SCRATCH_STREAM->CR & DMA_SxCR_EN);
        }

        void enable_and_disable_stream_are_null_safe() {
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, dma_enable_stream(nullptr));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, dma_disable_stream(nullptr));
        }

        void end_to_end_m2m_transfer_completes_and_matches_source() {
            constexpr size_t            COUNT = 16;
            std::array<uint32_t, COUNT> src{};
            std::array<uint32_t, COUNT> dst{};

            for (size_t i = 0; i < COUNT; i++) {
                src[i] = 0xC0FFEE00U + i;
            }

            TEST_ASSERT_EQUAL(HAL_OK, configure_scratch_m2m(src.data(), dst.data(), COUNT, true));
            TEST_ASSERT_TRUE_MESSAGE(wait_for_tc_flag(), "DMA M2M transfer never completed");

            TEST_ASSERT_EQUAL(HAL_OK, dma_isr_helper(SCRATCH_STREAM));

            // A non circular transfer must be auto disabled by the isr helper once it's done
            TEST_ASSERT_FALSE(SCRATCH_STREAM->CR & DMA_SxCR_EN);
            TEST_ASSERT_FALSE(DMA2->LISR & DMA_LISR_TCIF1);

            for (size_t i = 0; i < COUNT; i++) {
                TEST_ASSERT_EQUAL_UINT32(src[i], dst[i]);
            }

            dma_stream_config_t deconf;
            deconf.deconfigure = true;
            dma_configure_stream(SCRATCH_STREAM, &deconf);
        }

        void dma_memcpy_with_flag_works() {
            constexpr std::string_view      source = "This is the DMA memcpy function source. Nothing's happening here";
            std::array<char, source.size()> destination{};

            volatile memcpy_state_t flag = DMA_MEMCPY_NOT_DONE;
            TEST_ASSERT_EQUAL(HAL_OK, dma_memcpy(destination.data(), source.data(), source.size(), &flag));
            TEST_ASSERT_EQUAL(DMA_MEMCPY_DONE, dma_memcpy_wait_for_flag(&flag, TIMEOUT));

            TEST_ASSERT_EQUAL_CHAR_ARRAY(source.data(), destination.data(), source.size());
        }

        void dma_memcpy_with_callback_works() {
            constexpr std::string_view      source = "This is the DMA memcpy callback function source. Nothing's happening here";
            std::array<char, source.size()> destination{};

            // Its only static so the lambda passed to dma_memcpy_cb can see it
            memcpy_state_t flag = DMA_MEMCPY_NOT_DONE;
            TEST_ASSERT_EQUAL(HAL_OK,
                              dma_memcpy_cb(
                                  destination.data(),
                                  source.data(),
                                  source.size(),
                                  [](void* arg, hal_err_t ret) {
                                      TEST_ASSERT_EQUAL_UINT8(HAL_OK, ret);
                                      *static_cast<memcpy_state_t*>(arg) = DMA_MEMCPY_DONE;
                                  },
                                  &flag));

            TEST_ASSERT_EQUAL(DMA_MEMCPY_DONE, dma_memcpy_wait_for_flag(&flag, TIMEOUT));
            TEST_ASSERT_EQUAL_CHAR_ARRAY(source.data(), destination.data(), source.size());
        }

    } // namespace

    void all() {
        LOGI(TAG, "Starting the tests on the DMA driver");
        UNITY_BEGIN();

        RUN_TEST(clk_enable_toggles_only_the_targeted_controller);
        RUN_TEST(stream_info_lookup_covers_every_stream);
        RUN_TEST(stream_flags_pick_the_right_registers_and_masks);
        RUN_TEST(low_level_setters_are_null_safe_and_cover_every_enum_value);
        RUN_TEST(configure_stream_invalid_arg_guards);
        RUN_TEST(deconfigure_clears_addresses_length_and_irq);
        RUN_TEST(enable_and_disable_stream_are_null_safe);
        RUN_TEST(end_to_end_m2m_transfer_completes_and_matches_source);
        RUN_TEST(dma_memcpy_with_flag_works);
        RUN_TEST(dma_memcpy_with_callback_works);

        UNITY_END();
        LOGI(TAG, "Done with all tests on the DMA driver");
    }

} // namespace test::dma
