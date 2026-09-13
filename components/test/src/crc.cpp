#include "stm32f411xe.h"
#include "Unity/unity.h"

#include "utils/common.h"
#include "drivers/crc.h"
#include "drivers/dma.h"
#include "utils/err.h"
#include "utils/log.h"

#include <span>
#include <array>
#include <cstdint>


namespace test::crc {

    namespace {

        constexpr const char* TAG = "CRC_Test";

        // Software reference model of the STM32F411's hardware CRC unit: polynomial 0x04C11DB7,
        // initial value 0xFFFFFFFF, no input/output reflection, no final XOR, word at a time.
        constexpr uint32_t software_crc32(std::span<const uint32_t> data) {
            uint32_t crc = 0xFFFFFFFFU;
            for (const auto& num : data) {
                crc ^= num;
                for (size_t bit = 0; bit < 32; bit++) {
                    crc = (crc & 0x8000'0000U) ? (crc << 1) ^ 0x04C1'1DB7U : (crc << 1);
                }
            }
            return crc;
        }

        volatile bool      s_dma_done  = false;
        volatile hal_err_t s_dma_err   = HAL_OK;
        volatile uint32_t  s_dma_crc32 = 0;

        // Helpers
        void dma_done_callback(void*, hal_err_t err, uint32_t crc32) {
            s_dma_err   = err;
            s_dma_crc32 = crc32;
            s_dma_done  = true;
        }

        bool wait_for_dma_done() {
            uint32_t timeout = 10U * TIMEOUT_CYCLES;
            while (!s_dma_done && --timeout);
            return s_dma_done;
        }

        // TESTS
        void invalid_arg_guards() {
            uint32_t result = 0;

            constexpr std::array<uint32_t, 4> data{};

            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, crc_get(nullptr, data.size(), &result));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, crc_get(data.data(), 0, &result));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, crc_get(data.data(), data.size(), nullptr));

            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, crc_get_dma(nullptr, data.size(), DMA_PRIORITY_LOW, dma_done_callback, nullptr));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, crc_get_dma(data.data(), 0, DMA_PRIORITY_LOW, dma_done_callback, nullptr));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, crc_get_dma(data.data(), data.size(), DMA_PRIORITY_LOW, nullptr, nullptr));
        }

        void cpu_path_matches_the_software_reference_model() {
            constexpr auto SINGLE_WORD = std::array{
                0x1234'5678UL,
            };
            constexpr auto MULTI_WORD = std::array{
                0x0000'0000UL,
                0xFFFF'FFFFUL,
                0xDEAD'BEEFUL,
                0x0123'4567UL,
                0x89AB'CDEFUL,
                0x7FFF'FFFFUL,
                0x8000'0000UL,
                0x5A5A'5A5AUL,
            };

            constexpr uint32_t crc32_single_word = software_crc32(SINGLE_WORD);
            constexpr uint32_t crc32_multi_word  = software_crc32(MULTI_WORD);

            uint32_t hw_result = 0;
            TEST_ASSERT_EQUAL(HAL_OK, crc_get(SINGLE_WORD.data(), SINGLE_WORD.size(), &hw_result));
            TEST_ASSERT_EQUAL_UINT32(crc32_single_word, hw_result);

            TEST_ASSERT_EQUAL(HAL_OK, crc_get(MULTI_WORD.data(), MULTI_WORD.size(), &hw_result));
            TEST_ASSERT_EQUAL_UINT32(crc32_multi_word, hw_result);
        }

        void cpu_path_is_stateless_between_calls() {
            // Back to back calls with different data must not leak state from the previous
            // computation. The driver explicitly issues a CRC_CR_RESET before every call.
            constexpr auto FIRST = std::array{
                0x1111'1111UL,
                0x2222'2222UL,
                0x3333'3333UL,
            };
            constexpr auto SECOND = std::array{
                0x4444'4444UL,
                0x5555'5555UL,
                0x6666'6666UL,
            };

            constexpr uint32_t crc32_first  = software_crc32(FIRST);
            constexpr uint32_t crc32_second = software_crc32(SECOND);

            uint32_t first_result  = 0;
            uint32_t second_result = 0;

            TEST_ASSERT_EQUAL(HAL_OK, crc_get(FIRST.data(), FIRST.size(), &first_result));
            TEST_ASSERT_EQUAL(HAL_OK, crc_get(SECOND.data(), SECOND.size(), &second_result));

            TEST_ASSERT_EQUAL_UINT32(crc32_first, first_result);
            TEST_ASSERT_EQUAL_UINT32(crc32_second, second_result);

            // Re running FIRST again must reproduce the same result as the first time
            uint32_t repeat_result = 0;
            TEST_ASSERT_EQUAL(HAL_OK, crc_get(FIRST.data(), FIRST.size(), &repeat_result));
            TEST_ASSERT_EQUAL_UINT32(first_result, repeat_result);
        }

        void dma_path_matches_the_cpu_path_and_the_software_model() {
            constexpr auto DATA = std::array{
                0x0000'0001UL,
                0x0000'0002UL,
                0x0000'0003UL,
                0x0000'0004UL,
                0x0000'0005UL,
                0x0000'0006UL,
                0x0000'0007UL,
                0x0000'0008UL,
                0x0000'0009UL,
                0x0000'000AUL,
                0x0000'000BUL,
                0x0000'000CUL,
                0x0000'000DUL,
                0x0000'000EUL,
                0x0000'000FUL,
                0x0000'0010UL,
            };

            constexpr uint32_t crc32 = software_crc32(DATA);

            uint32_t cpu_result = 0;
            TEST_ASSERT_EQUAL(HAL_OK, crc_get(DATA.data(), DATA.size(), &cpu_result));
            TEST_ASSERT_EQUAL_UINT32(crc32, cpu_result);

            s_dma_done  = false;
            s_dma_err   = HAL_FAIL;
            s_dma_crc32 = 0;

            TEST_ASSERT_EQUAL(HAL_OK, crc_get_dma(DATA.data(), DATA.size(), DMA_PRIORITY_HIGH, dma_done_callback, nullptr));
            TEST_ASSERT_TRUE_MESSAGE(wait_for_dma_done(), "CRC DMA transfer never completed");

            TEST_ASSERT_EQUAL(HAL_OK, s_dma_err);
            TEST_ASSERT_EQUAL_UINT32(crc32, s_dma_crc32);
            TEST_ASSERT_EQUAL_UINT32(cpu_result, s_dma_crc32);
        }

        void dma_path_rejects_a_second_start_while_busy() {
            constexpr auto make_filled_buffer = [](uint32_t val) {
                // A big buffer so the DMA controller doesn't finish the transfers almost immediately before we can check
                std::array<uint32_t, 512> data{};
                data.fill(val);
                return data;
            };

            // A wide buffer so the DMA transfer takes long enough to reliably observe the
            // "already in progress" state from software, not just the two endpoints.
            constexpr auto     wide_buffer = make_filled_buffer(0xA5A5'A5A5U);
            constexpr uint32_t crc32_wide  = software_crc32(wide_buffer);

            s_dma_done  = false;
            s_dma_err   = HAL_FAIL;
            s_dma_crc32 = 0;

            TEST_ASSERT_EQUAL(HAL_OK, crc_get_dma(wide_buffer.data(), wide_buffer.size(), DMA_PRIORITY_LOW, dma_done_callback, nullptr));

            // Immediately re issuing another DMA request must be rejected while the first is still in flight
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_STATE,
                              crc_get_dma(wide_buffer.data(), wide_buffer.size(), DMA_PRIORITY_LOW, dma_done_callback, nullptr));

            TEST_ASSERT_TRUE_MESSAGE(wait_for_dma_done(), "CRC DMA transfer never completed");
            TEST_ASSERT_EQUAL(HAL_OK, s_dma_err);
            TEST_ASSERT_EQUAL_UINT32(crc32_wide, s_dma_crc32);

            // Once complete, a fresh request must be accepted again
            s_dma_done  = false;
            s_dma_err   = HAL_FAIL;
            s_dma_crc32 = 0;

            TEST_ASSERT_EQUAL(HAL_OK, crc_get_dma(wide_buffer.data(), wide_buffer.size(), DMA_PRIORITY_LOW, dma_done_callback, nullptr));
            TEST_ASSERT_TRUE_MESSAGE(wait_for_dma_done(), "CRC DMA transfer never completed");
        }

        void stream_info_reports_the_fixed_dma_mapping() {
            const dma_map_t map = crc_get_dma_stream_info();
            TEST_ASSERT_EQUAL_PTR(DMA2_Stream5, map.stream);
            TEST_ASSERT_EQUAL_UINT8(0, map.channel);
        }

        void clk_enable_toggles_the_ahb1_bit() {
            crc_clk_enable(false);
            TEST_ASSERT_FALSE(RCC->AHB1ENR & RCC_AHB1ENR_CRCEN);

            crc_clk_enable(true);
            TEST_ASSERT_TRUE(RCC->AHB1ENR & RCC_AHB1ENR_CRCEN);
        }

    } // namespace

    void all() {
        LOGI(TAG, "Starting the tests on the CRC driver");
        crc_clk_enable(true);

        RUN_TEST(clk_enable_toggles_the_ahb1_bit);
        RUN_TEST(invalid_arg_guards);
        RUN_TEST(cpu_path_matches_the_software_reference_model);
        RUN_TEST(cpu_path_is_stateless_between_calls);
        RUN_TEST(dma_path_matches_the_cpu_path_and_the_software_model);
        RUN_TEST(dma_path_rejects_a_second_start_while_busy);
        RUN_TEST(stream_info_reports_the_fixed_dma_mapping);

        crc_clk_enable(false);
        LOGI(TAG, "Done with all tests on the CRC driver");
    }

} // namespace test::crc
