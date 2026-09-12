#include "stm32f411xe.h"
#include "Unity/unity.h"

#include "utils/common.h"
#include "drivers/crc.h"
#include "drivers/dma.h"
#include "utils/err.h"
#include "utils/log.h"

#include <array>
#include <cstdint>


namespace test::crc {

    namespace {

        constexpr const char* TAG = "CRC_Test";

        // A wide buffer so the DMA transfer takes long enough to reliably observe the
        // "already in progress" state from software, not just the two endpoints.
        constexpr size_t                        WIDE_BUFFER_WORDS = 2048;
        std::array<uint32_t, WIDE_BUFFER_WORDS> s_wide_buffer{};

        // Software reference model of the STM32 hardware CRC unit: polynomial 0x04C11DB7,
        // initial value 0xFFFFFFFF, no input/output reflection, no final XOR, word-at-a-time.
        uint32_t soft_crc32(const uint32_t* data, size_t count) {
            uint32_t crc = 0xFFFFFFFFU;
            for (size_t i = 0; i < count; i++) {
                crc ^= data[i];
                for (int bit = 0; bit < 32; bit++) {
                    crc = (crc & 0x8000'0000U) ? (crc << 1) ^ 0x04C1'1DB7U : (crc << 1);
                }
            }
            return crc;
        }

        volatile bool      s_dma_done  = false;
        volatile hal_err_t s_dma_err   = HAL_OK;
        volatile uint32_t  s_dma_crc32 = 0;

        void dma_done_callback(void* arg, hal_err_t err, uint32_t crc32) {
            UNUSED(arg);
            s_dma_err   = err;
            s_dma_crc32 = crc32;
            s_dma_done  = true;
        }

        // Bounded spin-wait, mirroring the TIMEOUT_CYCLES pattern used throughout the drivers
        bool wait_for_dma_done() {
            uint32_t timeout = 10U * TIMEOUT_CYCLES;
            while (!s_dma_done && --timeout) {
            }
            return s_dma_done;
        }

        void invalid_arg_guards() {
            uint32_t result = 0;

            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, crc_get(nullptr, 4, &result));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, crc_get(s_wide_buffer.data(), 0, &result));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, crc_get(s_wide_buffer.data(), 4, nullptr));

            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, crc_get_dma(nullptr, 4, DMA_PRIORITY_LOW, dma_done_callback, nullptr));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, crc_get_dma(s_wide_buffer.data(), 0, DMA_PRIORITY_LOW, dma_done_callback, nullptr));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, crc_get_dma(s_wide_buffer.data(), 4, DMA_PRIORITY_LOW, nullptr, nullptr));
        }

        void cpu_path_matches_the_software_reference_model() {
            constexpr std::array<uint32_t, 1> SINGLE_WORD{0x1234'5678U};
            constexpr std::array<uint32_t, 8> MULTI_WORD{
                0x0000'0000U,
                0xFFFF'FFFFU,
                0xDEAD'BEEFU,
                0x0123'4567U,
                0x89AB'CDEFU,
                0x7FFF'FFFFU,
                0x8000'0000U,
                0x5A5A'5A5AU,
            };

            uint32_t hw_result = 0;
            TEST_ASSERT_EQUAL(HAL_OK, crc_get(SINGLE_WORD.data(), SINGLE_WORD.size(), &hw_result));
            TEST_ASSERT_EQUAL_UINT32(soft_crc32(SINGLE_WORD.data(), SINGLE_WORD.size()), hw_result);

            TEST_ASSERT_EQUAL(HAL_OK, crc_get(MULTI_WORD.data(), MULTI_WORD.size(), &hw_result));
            TEST_ASSERT_EQUAL_UINT32(soft_crc32(MULTI_WORD.data(), MULTI_WORD.size()), hw_result);
        }

        void cpu_path_is_stateless_between_calls() {
            // Back-to-back calls with different data must not leak state from the previous
            // computation - the driver explicitly issues CRC_CR_RESET before every call.
            constexpr std::array<uint32_t, 3> FIRST{0x1111'1111U, 0x2222'2222U, 0x3333'3333U};
            constexpr std::array<uint32_t, 3> SECOND{0x4444'4444U, 0x5555'5555U, 0x6666'6666U};

            uint32_t first_result  = 0;
            uint32_t second_result = 0;

            TEST_ASSERT_EQUAL(HAL_OK, crc_get(FIRST.data(), FIRST.size(), &first_result));
            TEST_ASSERT_EQUAL(HAL_OK, crc_get(SECOND.data(), SECOND.size(), &second_result));

            TEST_ASSERT_EQUAL_UINT32(soft_crc32(FIRST.data(), FIRST.size()), first_result);
            TEST_ASSERT_EQUAL_UINT32(soft_crc32(SECOND.data(), SECOND.size()), second_result);

            // Re-running FIRST again must reproduce the same result as the first time
            uint32_t repeat_result = 0;
            TEST_ASSERT_EQUAL(HAL_OK, crc_get(FIRST.data(), FIRST.size(), &repeat_result));
            TEST_ASSERT_EQUAL_UINT32(first_result, repeat_result);
        }

        void dma_path_matches_the_cpu_path_and_the_software_model() {
            constexpr std::array<uint32_t, 16> DATA{
                0x0000'0001U,
                0x0000'0002U,
                0x0000'0003U,
                0x0000'0004U,
                0x0000'0005U,
                0x0000'0006U,
                0x0000'0007U,
                0x0000'0008U,
                0x0000'0009U,
                0x0000'000AU,
                0x0000'000BU,
                0x0000'000CU,
                0x0000'000DU,
                0x0000'000EU,
                0x0000'000FU,
                0x0000'0010U,
            };

            uint32_t cpu_result = 0;
            TEST_ASSERT_EQUAL(HAL_OK, crc_get(DATA.data(), DATA.size(), &cpu_result));

            s_dma_done  = false;
            s_dma_err   = HAL_FAIL;
            s_dma_crc32 = 0;

            TEST_ASSERT_EQUAL(HAL_OK, crc_get_dma(DATA.data(), static_cast<uint16_t>(DATA.size()), DMA_PRIORITY_HIGH, dma_done_callback, nullptr));
            TEST_ASSERT_TRUE_MESSAGE(wait_for_dma_done(), "CRC DMA transfer never completed");

            TEST_ASSERT_EQUAL(HAL_OK, s_dma_err);
            TEST_ASSERT_EQUAL_UINT32(soft_crc32(DATA.data(), DATA.size()), s_dma_crc32);
            TEST_ASSERT_EQUAL_UINT32(cpu_result, s_dma_crc32);
        }

        void dma_path_rejects_a_second_start_while_busy() {
            for (auto& word : s_wide_buffer) {
                // cppcheck-suppress useStlAlgorithm
                word = 0xA5A5'A5A5U;
            }

            s_dma_done  = false;
            s_dma_err   = HAL_FAIL;
            s_dma_crc32 = 0;

            TEST_ASSERT_EQUAL(
                HAL_OK, crc_get_dma(s_wide_buffer.data(), static_cast<uint16_t>(WIDE_BUFFER_WORDS), DMA_PRIORITY_LOW, dma_done_callback, nullptr));

            // Immediately re-issuing another DMA request must be rejected while the first is still in flight
            TEST_ASSERT_EQUAL(
                HAL_ERR_INVALID_STATE,
                crc_get_dma(s_wide_buffer.data(), static_cast<uint16_t>(WIDE_BUFFER_WORDS), DMA_PRIORITY_LOW, dma_done_callback, nullptr));

            TEST_ASSERT_TRUE_MESSAGE(wait_for_dma_done(), "CRC DMA transfer never completed");
            TEST_ASSERT_EQUAL(HAL_OK, s_dma_err);
            TEST_ASSERT_EQUAL_UINT32(soft_crc32(s_wide_buffer.data(), WIDE_BUFFER_WORDS), s_dma_crc32);

            // Once complete, a fresh request must be accepted again
            s_dma_done = false;
            TEST_ASSERT_EQUAL(HAL_OK, crc_get_dma(s_wide_buffer.data(), 4, DMA_PRIORITY_LOW, dma_done_callback, nullptr));
            TEST_ASSERT_TRUE_MESSAGE(wait_for_dma_done(), "CRC DMA transfer never completed");
        }

        void stream_info_reports_the_fixed_dma_mapping() {
            const dma_map_t map = crc_get_dma_stream_info();
            TEST_ASSERT_EQUAL_PTR(DMA2_Stream5, map.stream);
            TEST_ASSERT_EQUAL_UINT8(0, map.channel);
        }

        void clk_enable_toggles_the_ahb1_bit() {
            crc_clk_enable(true);
            TEST_ASSERT_TRUE(RCC->AHB1ENR & RCC_AHB1ENR_CRCEN);

            crc_clk_enable(false);
            TEST_ASSERT_FALSE(RCC->AHB1ENR & RCC_AHB1ENR_CRCEN);

            crc_clk_enable(true);
        }

    } // namespace

    void all() {
        LOGI(TAG, "Starting the tests on the CRC driver");

        RUN_TEST(clk_enable_toggles_the_ahb1_bit);
        RUN_TEST(invalid_arg_guards);
        RUN_TEST(cpu_path_matches_the_software_reference_model);
        RUN_TEST(cpu_path_is_stateless_between_calls);
        RUN_TEST(dma_path_matches_the_cpu_path_and_the_software_model);
        RUN_TEST(dma_path_rejects_a_second_start_while_busy);
        RUN_TEST(stream_info_reports_the_fixed_dma_mapping);

        LOGI(TAG, "Done with all tests on the CRC driver");
    }

} // namespace test::crc
