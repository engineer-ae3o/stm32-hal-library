#include "stm32f411xe.h"
#include "Unity/unity.h"

#include "drivers/dma.h"
#include "drivers/dma_types.h"
#include "utils/common.h"
#include "utils/err.h"
#include "utils/log.h"

#include <array>
#include <cstdint>
#include <cstring>


namespace test::dma {

    namespace {

        constexpr const char* TAG = "DmaTest";

        // An otherwise-unused stream reserved for tests that need to drive a real transfer
        DMA_Stream_TypeDef* const SCRATCH_STREAM = DMA2_Stream1;

        struct stream_expectation_t {
            DMA_Stream_TypeDef* stream;
            DMA_TypeDef*        controller;
            uint32_t            stream_number;
            IRQn_Type           irq;
        };

        const std::array<stream_expectation_t, 16> ALL_STREAMS{{
            {DMA1_Stream0, DMA1, 0, DMA1_Stream0_IRQn},
            {DMA1_Stream1, DMA1, 1, DMA1_Stream1_IRQn},
            {DMA1_Stream2, DMA1, 2, DMA1_Stream2_IRQn},
            {DMA1_Stream3, DMA1, 3, DMA1_Stream3_IRQn},
            {DMA1_Stream4, DMA1, 4, DMA1_Stream4_IRQn},
            {DMA1_Stream5, DMA1, 5, DMA1_Stream5_IRQn},
            {DMA1_Stream6, DMA1, 6, DMA1_Stream6_IRQn},
            {DMA1_Stream7, DMA1, 7, DMA1_Stream7_IRQn},
            {DMA2_Stream0, DMA2, 0, DMA2_Stream0_IRQn},
            {DMA2_Stream1, DMA2, 1, DMA2_Stream1_IRQn},
            {DMA2_Stream2, DMA2, 2, DMA2_Stream2_IRQn},
            {DMA2_Stream3, DMA2, 3, DMA2_Stream3_IRQn},
            {DMA2_Stream4, DMA2, 4, DMA2_Stream4_IRQn},
            {DMA2_Stream5, DMA2, 5, DMA2_Stream5_IRQn},
            {DMA2_Stream6, DMA2, 6, DMA2_Stream6_IRQn},
            {DMA2_Stream7, DMA2, 7, DMA2_Stream7_IRQn},
        }};

        // Deinitialize + fully reconfigure the scratch stream as a safe, self-contained
        // memory-to-memory word copy so tests can drive it without risking a bus fault.
        hal_err_t configure_scratch_m2m(const uint32_t* src, uint32_t* dst, uint16_t count, dma_circ_mode_t circ, bool enable) {
            dma_stream_config_t deconf{};
            deconf.deconfigure = true;
            TRY(dma_configure_stream(SCRATCH_STREAM, &deconf));

            const dma_stream_config_t config{
                .deconfigure   = false,
                .enable_stream = enable,

                .per_addr_incement = true,
                .mem_addr_incement = true,

                .tc_irq_enable  = true,
                .ht_irq_enable  = false,
                .te_irq_enable  = true,
                .dme_irq_enable = false,
                .fe_irq_enable  = true,

                .mode            = DMA_MODE_FIFO,
                .priority        = DMA_PRIORITY_LOW,
                .direction       = DMA_DIR_M_M,
                .per_data_size   = DMA_SIZE_WORD,
                .mem_data_size   = DMA_SIZE_WORD,
                .circular_mode   = circ,
                .flow_controller = DMA_FLOW_CONTROLLER_DMA,

                .buffer_size       = count,
                .channel           = 0,
                .nvic_irq_priority = CRC_DMA_NVIC_IRQ_PRIORITY,

                .per_addr  = src,
                .mem_buf_0 = dst,
                .mem_buf_1 = nullptr,
            };
            return dma_configure_stream(SCRATCH_STREAM, &config);
        }

        bool wait_for_tc_flag() {
            uint32_t timeout = 10U * TIMEOUT_CYCLES;
            while (!(DMA2->LISR & DMA_LISR_TCIF1) && --timeout) {
            }
            return (DMA2->LISR & DMA_LISR_TCIF1) != 0;
        }

    } // namespace

    void clk_enable_toggles_only_the_targeted_controller() {
        TEST_ASSERT_EQUAL(HAL_OK, dmax_clk_enable(DMA1, true));
        TEST_ASSERT_TRUE(RCC->AHB1ENR & RCC_AHB1ENR_DMA1EN);
        TEST_ASSERT_EQUAL(HAL_OK, dmax_clk_enable(DMA1, false));
        TEST_ASSERT_FALSE(RCC->AHB1ENR & RCC_AHB1ENR_DMA1EN);

        TEST_ASSERT_EQUAL(HAL_OK, dmax_clk_enable(DMA2, true));
        TEST_ASSERT_TRUE(RCC->AHB1ENR & RCC_AHB1ENR_DMA2EN);

        const uint32_t before = RCC->AHB1ENR;
        TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, dmax_clk_enable(nullptr, true));
        TEST_ASSERT_EQUAL_UINT32(before, RCC->AHB1ENR);
    }

    void stream_info_lookup_covers_every_stream() {
        for (const auto& exp : ALL_STREAMS) {
            dma_stream_info_t info{};
            TEST_ASSERT_EQUAL(HAL_OK, dma_get_stream_info(exp.stream, &info));
            TEST_ASSERT_EQUAL_PTR(exp.stream, info.stream);
            TEST_ASSERT_EQUAL_PTR(exp.controller, info.controller);
            TEST_ASSERT_EQUAL_UINT32(exp.stream_number, info.stream_number);
            TEST_ASSERT_EQUAL(exp.irq, info.nvic_irq_type);
        }

        dma_stream_info_t info{};
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

        const std::array<flag_expectation_t, 8> EXPECTATIONS{{
            {0, DMA_LISR_TCIF0, DMA_LISR_TEIF0, DMA_LISR_HTIF0, DMA_LISR_DMEIF0, DMA_LISR_FEIF0, true},
            {1, DMA_LISR_TCIF1, DMA_LISR_TEIF1, DMA_LISR_HTIF1, DMA_LISR_DMEIF1, DMA_LISR_FEIF1, true},
            {2, DMA_LISR_TCIF2, DMA_LISR_TEIF2, DMA_LISR_HTIF2, DMA_LISR_DMEIF2, DMA_LISR_FEIF2, true},
            {3, DMA_LISR_TCIF3, DMA_LISR_TEIF3, DMA_LISR_HTIF3, DMA_LISR_DMEIF3, DMA_LISR_FEIF3, true},
            {4, DMA_HISR_TCIF4, DMA_HISR_TEIF4, DMA_HISR_HTIF4, DMA_HISR_DMEIF4, DMA_HISR_FEIF4, false},
            {5, DMA_HISR_TCIF5, DMA_HISR_TEIF5, DMA_HISR_HTIF5, DMA_HISR_DMEIF5, DMA_HISR_FEIF5, false},
            {6, DMA_HISR_TCIF6, DMA_HISR_TEIF6, DMA_HISR_HTIF6, DMA_HISR_DMEIF6, DMA_HISR_FEIF6, false},
            {7, DMA_HISR_TCIF7, DMA_HISR_TEIF7, DMA_HISR_HTIF7, DMA_HISR_DMEIF7, DMA_HISR_FEIF7, false},
        }};

        for (const auto& exp : EXPECTATIONS) {
            dma_stream_flags_t flags{};
            TEST_ASSERT_EQUAL(HAL_OK, dma_get_stream_flags(DMA1_Stream0, DMA1, &flags, exp.stream_number));

            TEST_ASSERT_EQUAL_UINT32(exp.tc_mask, flags.tc_mask);
            TEST_ASSERT_EQUAL_UINT32(exp.te_mask, flags.te_mask);
            TEST_ASSERT_EQUAL_UINT32(exp.ht_mask, flags.ht_mask);
            TEST_ASSERT_EQUAL_UINT32(exp.dme_mask, flags.dme_mask);
            TEST_ASSERT_EQUAL_UINT32(exp.fe_mask, flags.fe_mask);

            if (exp.is_low) {
                TEST_ASSERT_EQUAL_PTR(&DMA1->LISR, flags.irq_status_register);
                TEST_ASSERT_EQUAL_PTR(&DMA1->LIFCR, flags.irq_clear_register);
            } else {
                TEST_ASSERT_EQUAL_PTR(&DMA1->HISR, flags.irq_status_register);
                TEST_ASSERT_EQUAL_PTR(&DMA1->HIFCR, flags.irq_clear_register);
            }
        }

        dma_stream_flags_t flags{};
        TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, dma_get_stream_flags(nullptr, DMA1, &flags, 0));
        TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, dma_get_stream_flags(DMA1_Stream0, DMA1, &flags, 8));
    }

    void low_level_setters_are_null_safe_and_cover_every_enum_value() {
        // NULL must be a documented safe no-op for every one of these - the real assertion
        // here is simply that none of these crash
        dma_set_channel(nullptr, 3);
        dma_set_direct_mode(nullptr, true);
        dma_set_trans_length(nullptr, 10);
        dma_set_direction(nullptr, DMA_DIR_M_P);
        dma_set_increment(nullptr, true, true);
        dma_set_flow_controller(nullptr, true);
        dma_set_stream_priority(nullptr, DMA_PRIORITY_HIGH);
        dma_enable_circm_dbm(nullptr, true, true);
        dma_set_per_mem_size(nullptr, DMA_SIZE_BYTE, DMA_SIZE_WORD);
        dma_enable_irqs(nullptr, true, true, true, true);
        dma_set_addresses(nullptr, nullptr, nullptr, nullptr);

        DMA_Stream_TypeDef* const s = SCRATCH_STREAM;
        s->CR                       = 0;
        s->FCR                      = 0;

        for (uint32_t channel = 0; channel <= 7; channel++) {
            dma_set_channel(s, channel);
            TEST_ASSERT_EQUAL_UINT32(channel, (s->CR & DMA_SxCR_CHSEL) >> DMA_SxCR_CHSEL_Pos);
        }

        dma_set_direct_mode(s, true);
        TEST_ASSERT_FALSE(s->FCR & DMA_SxFCR_DMDIS);
        dma_set_direct_mode(s, false);
        TEST_ASSERT_TRUE(s->FCR & DMA_SxFCR_DMDIS);

        dma_set_trans_length(s, 1234);
        TEST_ASSERT_EQUAL_UINT32(1234, s->NDTR);

        constexpr std::array<dma_direction_t, 3> DIRECTIONS{DMA_DIR_P_M, DMA_DIR_M_P, DMA_DIR_M_M};
        for (const auto dir : DIRECTIONS) {
            dma_set_direction(s, dir);
            TEST_ASSERT_EQUAL_UINT32(static_cast<uint32_t>(dir), (s->CR & DMA_SxCR_DIR) >> DMA_SxCR_DIR_Pos);
        }

        dma_set_increment(s, true, false);
        TEST_ASSERT_TRUE(s->CR & DMA_SxCR_PINC);
        TEST_ASSERT_FALSE(s->CR & DMA_SxCR_MINC);
        dma_set_increment(s, false, true);
        TEST_ASSERT_FALSE(s->CR & DMA_SxCR_PINC);
        TEST_ASSERT_TRUE(s->CR & DMA_SxCR_MINC);

        dma_set_flow_controller(s, true);
        TEST_ASSERT_FALSE(s->CR & DMA_SxCR_PFCTRL);
        dma_set_flow_controller(s, false);
        TEST_ASSERT_TRUE(s->CR & DMA_SxCR_PFCTRL);

        constexpr std::array<dma_priority_t, 4> PRIORITIES{DMA_PRIORITY_LOW, DMA_PRIORITY_MEDIUM, DMA_PRIORITY_HIGH, DMA_PRIORITY_VERY_HIGH};
        for (const auto prio : PRIORITIES) {
            dma_set_stream_priority(s, prio);
            TEST_ASSERT_EQUAL_UINT32(static_cast<uint32_t>(prio), (s->CR & DMA_SxCR_PL) >> DMA_SxCR_PL_Pos);
        }

        dma_enable_circm_dbm(s, true, true);
        TEST_ASSERT_TRUE(s->CR & DMA_SxCR_CIRC);
        TEST_ASSERT_TRUE(s->CR & DMA_SxCR_DBM);
        dma_enable_circm_dbm(s, false, false);
        TEST_ASSERT_FALSE(s->CR & DMA_SxCR_CIRC);
        TEST_ASSERT_FALSE(s->CR & DMA_SxCR_DBM);

        constexpr std::array<dma_data_size_t, 3> SIZES{DMA_SIZE_BYTE, DMA_SIZE_HWORD, DMA_SIZE_WORD};
        for (const auto per : SIZES) {
            for (const auto mem : SIZES) {
                dma_set_per_mem_size(s, per, mem);
                TEST_ASSERT_EQUAL_UINT32(static_cast<uint32_t>(per), (s->CR & DMA_SxCR_PSIZE) >> DMA_SxCR_PSIZE_Pos);
                TEST_ASSERT_EQUAL_UINT32(static_cast<uint32_t>(mem), (s->CR & DMA_SxCR_MSIZE) >> DMA_SxCR_MSIZE_Pos);
            }
        }

        dma_enable_irqs(s, true, false, true, false);
        TEST_ASSERT_TRUE(s->CR & DMA_SxCR_TCIE);
        TEST_ASSERT_FALSE(s->CR & DMA_SxCR_TEIE);
        TEST_ASSERT_TRUE(s->CR & DMA_SxCR_HTIE);
        TEST_ASSERT_FALSE(s->CR & DMA_SxCR_DMEIE);

        uint32_t per_addr = 0, mem0 = 0, mem1 = 0;
        dma_set_addresses(s, &per_addr, &mem0, &mem1);
        TEST_ASSERT_EQUAL_UINT32(reinterpret_cast<uint32_t>(&per_addr), s->PAR);
        TEST_ASSERT_EQUAL_UINT32(reinterpret_cast<uint32_t>(&mem0), s->M0AR);
        TEST_ASSERT_EQUAL_UINT32(reinterpret_cast<uint32_t>(&mem1), s->M1AR);

        // A NULL address must leave the corresponding register untouched, not zero it
        dma_set_addresses(s, nullptr, nullptr, nullptr);
        TEST_ASSERT_EQUAL_UINT32(reinterpret_cast<uint32_t>(&per_addr), s->PAR);
        TEST_ASSERT_EQUAL_UINT32(reinterpret_cast<uint32_t>(&mem0), s->M0AR);
        TEST_ASSERT_EQUAL_UINT32(reinterpret_cast<uint32_t>(&mem1), s->M1AR);

        s->CR  = 0;
        s->FCR = 0;
    }

    void configure_stream_invalid_arg_guards() {
        dma_stream_config_t config{};
        TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, dma_configure_stream(nullptr, &config));
        TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, dma_configure_stream(SCRATCH_STREAM, nullptr));
    }

    void deconfigure_clears_addresses_length_and_irq() {
        uint32_t src = 0, dst = 0;
        TEST_ASSERT_EQUAL(HAL_OK, configure_scratch_m2m(&src, &dst, 4, DMA_MODE_NO_CIRCULAR, false));

        dma_stream_config_t deconf{};
        deconf.deconfigure = true;
        TEST_ASSERT_EQUAL(HAL_OK, dma_configure_stream(SCRATCH_STREAM, &deconf));

        TEST_ASSERT_EQUAL_UINT32(0, SCRATCH_STREAM->PAR);
        TEST_ASSERT_EQUAL_UINT32(0, SCRATCH_STREAM->M0AR);
        TEST_ASSERT_EQUAL_UINT32(0, SCRATCH_STREAM->M1AR);
        TEST_ASSERT_EQUAL_UINT32(0, SCRATCH_STREAM->NDTR);
        TEST_ASSERT_FALSE(SCRATCH_STREAM->CR & DMA_SxCR_EN);
    }

    void configure_stream_double_buffer_sets_circ_and_dbm() {
        uint32_t src = 0, dst0 = 0, dst1 = 0;

        dma_stream_config_t deconf{};
        deconf.deconfigure = true;
        TEST_ASSERT_EQUAL(HAL_OK, dma_configure_stream(SCRATCH_STREAM, &deconf));

        const dma_stream_config_t config{
            .deconfigure       = false,
            .enable_stream     = false,
            .per_addr_incement = true,
            .mem_addr_incement = true,
            .tc_irq_enable     = false,
            .ht_irq_enable     = false,
            .te_irq_enable     = false,
            .dme_irq_enable    = false,
            .fe_irq_enable     = false,
            .mode              = DMA_MODE_FIFO,
            .priority          = DMA_PRIORITY_LOW,
            .direction         = DMA_DIR_M_M,
            .per_data_size     = DMA_SIZE_WORD,
            .mem_data_size     = DMA_SIZE_WORD,
            .circular_mode     = DMA_MODE_DOUBLE_BUFFER,
            .flow_controller   = DMA_FLOW_CONTROLLER_DMA,
            .buffer_size       = 4,
            .channel           = 0,
            .nvic_irq_priority = CRC_DMA_NVIC_IRQ_PRIORITY,
            .per_addr          = &src,
            .mem_buf_0         = &dst0,
            .mem_buf_1         = &dst1,
        };
        TEST_ASSERT_EQUAL(HAL_OK, dma_configure_stream(SCRATCH_STREAM, &config));

        TEST_ASSERT_TRUE(SCRATCH_STREAM->CR & DMA_SxCR_CIRC);
        TEST_ASSERT_TRUE(SCRATCH_STREAM->CR & DMA_SxCR_DBM);
        TEST_ASSERT_EQUAL_UINT32(reinterpret_cast<uint32_t>(&dst1), SCRATCH_STREAM->M1AR);

        deconf.deconfigure = true;
        dma_configure_stream(SCRATCH_STREAM, &deconf);
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
            src[i] = static_cast<uint32_t>(0xC0FFEE00U + i);
        }
        dst.fill(0);

        TEST_ASSERT_EQUAL(HAL_OK, configure_scratch_m2m(src.data(), dst.data(), COUNT, DMA_MODE_NO_CIRCULAR, true));
        TEST_ASSERT_TRUE_MESSAGE(wait_for_tc_flag(), "DMA M2M transfer never completed");

        TEST_ASSERT_EQUAL(HAL_OK, dma_isr_helper(SCRATCH_STREAM));

        // A non-circular transfer must be auto-disabled by the isr helper once it's done
        TEST_ASSERT_FALSE(SCRATCH_STREAM->CR & DMA_SxCR_EN);
        TEST_ASSERT_FALSE(DMA2->LISR & DMA_LISR_TCIF1);

        for (size_t i = 0; i < COUNT; i++) {
            TEST_ASSERT_EQUAL_UINT32(src[i], dst[i]);
        }

        dma_stream_config_t deconf{};
        deconf.deconfigure = true;
        dma_configure_stream(SCRATCH_STREAM, &deconf);
    }

    void circular_transfer_is_left_enabled_by_the_isr_helper() {
        constexpr size_t            COUNT = 8;
        std::array<uint32_t, COUNT> src{};
        std::array<uint32_t, COUNT> dst{};
        src.fill(0x1234'5678U);
        dst.fill(0);

        TEST_ASSERT_EQUAL(HAL_OK, configure_scratch_m2m(src.data(), dst.data(), COUNT, DMA_MODE_CIRCULAR, true));
        TEST_ASSERT_TRUE_MESSAGE(wait_for_tc_flag(), "DMA M2M transfer never completed");

        TEST_ASSERT_EQUAL(HAL_OK, dma_isr_helper(SCRATCH_STREAM));

        // Circular mode must be left running by the isr helper, not disabled
        TEST_ASSERT_TRUE(SCRATCH_STREAM->CR & DMA_SxCR_EN);

        TEST_ASSERT_EQUAL(HAL_OK, dma_disable_stream(SCRATCH_STREAM));

        dma_stream_config_t deconf{};
        deconf.deconfigure = true;
        dma_configure_stream(SCRATCH_STREAM, &deconf);
    }

    void all() {
        LOGI(TAG, "Starting the tests on the DMA driver");

        RUN_TEST(clk_enable_toggles_only_the_targeted_controller);
        RUN_TEST(stream_info_lookup_covers_every_stream);
        RUN_TEST(stream_flags_pick_the_right_registers_and_masks);
        RUN_TEST(low_level_setters_are_null_safe_and_cover_every_enum_value);
        RUN_TEST(configure_stream_invalid_arg_guards);
        RUN_TEST(deconfigure_clears_addresses_length_and_irq);
        RUN_TEST(configure_stream_double_buffer_sets_circ_and_dbm);
        RUN_TEST(enable_and_disable_stream_are_null_safe);
        RUN_TEST(end_to_end_m2m_transfer_completes_and_matches_source);
        RUN_TEST(circular_transfer_is_left_enabled_by_the_isr_helper);

        LOGI(TAG, "Done with all tests on the DMA driver");
    }

} // namespace test::dma
