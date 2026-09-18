#include "stm32f411xe.h"
#include "Unity/unity.h"

#include "utils/common.h"
#include "drivers/uart.h"
#include "drivers/gpio.h"
#include "test/uart.hpp"
#include "utils/tick.h"
#include "utils/err.h"
#include "utils/log.h"

#include <array>
#include <cstdint>
#include <algorithm>


namespace test::uart {

    namespace {

        constexpr const char* TAG = "UART_Test";

        // Instance under test. TX and RX must be physically jumpered together on the
        // board for the loopback tests below to pass.
        USART_TypeDef* const TEST_INSTANCE = USART1;
        GPIO_TypeDef* const  TEST_PORT     = GPIOA;
        constexpr gpio_pin_t TEST_TX_PIN   = GPIO_PIN_9;
        constexpr gpio_pin_t TEST_RX_PIN   = GPIO_PIN_10;

        const uart_config_t DEFAULT_CONFIG = {
            .over_sampling = UART_OVER_SAMPLING_16,
            .baud_rate     = 115200UL,
            .tx_pin        = BOARD_UART1_TX_PA9,
            .rx_pin        = BOARD_UART1_RX_PA10,
        };

        // Helpers
        volatile bool      s_tx_done = false;
        volatile hal_err_t s_tx_err  = HAL_OK;
        inline void        tx_done_callback(void*, hal_err_t err) {
            s_tx_err  = err;
            s_tx_done = true;
        }

        volatile bool      s_rx_done = false;
        volatile hal_err_t s_rx_err  = HAL_OK;
        inline void        rx_done_callback(void*, hal_err_t err) {
            s_rx_err  = err;
            s_rx_done = true;
        }

        inline bool wait_for(volatile bool& flag) {
            uint32_t timeout = 100 * TIMEOUT_CYCLES;
            while (!flag && --timeout);
            return flag;
        }

        // TESTS
        void invalid_arg_guards() {
            constexpr std::array<uint8_t, 4> data{};
            std::array<uint8_t, 4>           rx_buf{};
            uint8_t                          byte = 0;

            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, uart_init(nullptr, &DEFAULT_CONFIG));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, uart_init(TEST_INSTANCE, nullptr));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, uart_deinit(nullptr));

            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, uart_transmit_byte(nullptr, byte));

            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, uart_transmit_poll(nullptr, data.data(), data.size()));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, uart_transmit_poll(TEST_INSTANCE, nullptr, data.size()));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, uart_transmit_poll(TEST_INSTANCE, data.data(), 0));

            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, uart_transmit_dma(nullptr, data.data(), data.size(), nullptr, nullptr));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, uart_transmit_dma(TEST_INSTANCE, nullptr, data.size(), nullptr, nullptr));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, uart_transmit_dma(TEST_INSTANCE, data.data(), 0, nullptr, nullptr));

            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, uart_receive_dma(nullptr, rx_buf.data(), rx_buf.size(), nullptr, nullptr));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, uart_receive_dma(TEST_INSTANCE, nullptr, rx_buf.size(), nullptr, nullptr));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, uart_receive_dma(TEST_INSTANCE, rx_buf.data(), 0, nullptr, nullptr));

            // A handle with no DMA mapping (or an unrecognized handle entirely) must be rejected too
            auto* const bogus_handle = reinterpret_cast<USART_TypeDef*>(0x1);
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, uart_dma_init(bogus_handle, DMA_PRIORITY_LOW));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, uart_dma_deinit(bogus_handle));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, uart_transmit_dma(bogus_handle, data.data(), data.size(), nullptr, nullptr));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, uart_receive_dma(bogus_handle, rx_buf.data(), rx_buf.size(), nullptr, nullptr));
        }

        void clk_enable_toggles_the_correct_bus_bit() {
            uartx_clk_enable(USART1, false);
            TEST_ASSERT_FALSE(RCC->APB2ENR & RCC_APB2ENR_USART1EN);
            uartx_clk_enable(USART1, true);
            TEST_ASSERT_TRUE(RCC->APB2ENR & RCC_APB2ENR_USART1EN);
            // Leave USART1 enabled: the loopback tests below need it

            uartx_clk_enable(USART2, false);
            TEST_ASSERT_FALSE(RCC->APB1ENR & RCC_APB1ENR_USART2EN);
            uartx_clk_enable(USART2, true);
            TEST_ASSERT_TRUE(RCC->APB1ENR & RCC_APB1ENR_USART2EN);
            uartx_clk_enable(USART2, false);

            uartx_clk_enable(USART6, false);
            TEST_ASSERT_FALSE(RCC->APB2ENR & RCC_APB2ENR_USART6EN);
            uartx_clk_enable(USART6, true);
            TEST_ASSERT_TRUE(RCC->APB2ENR & RCC_APB2ENR_USART6EN);
            uartx_clk_enable(USART6, false);

            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, uartx_clk_enable(reinterpret_cast<USART_TypeDef*>(0x1), true));
        }

        void dma_init_rejects_unsupported_instance() {
            // USART6 has no DMA streams mapped to it
            uartx_clk_enable(USART6, true);
            TEST_ASSERT_EQUAL(HAL_ERR_NOT_SUPPORTED, uart_dma_init(USART6, DMA_PRIORITY_LOW));
            TEST_ASSERT_EQUAL(HAL_ERR_NOT_SUPPORTED, uart_dma_deinit(USART6));
            uartx_clk_enable(USART6, false);
        }

        void poll_tx_and_dma_rx_loopback_matches_sent_bytes() {
            TEST_ASSERT_EQUAL(HAL_OK, uart_init(TEST_INSTANCE, &DEFAULT_CONFIG));
            TEST_ASSERT_EQUAL(HAL_OK, uart_dma_init(TEST_INSTANCE, DMA_PRIORITY_HIGH));

            constexpr std::array<uint8_t, 8>    TX_DATA = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
            std::array<uint8_t, TX_DATA.size()> rx_buf{};

            s_rx_done = false;
            s_rx_err  = HAL_FAIL;

            // Arm the receiver before transmitting: it's the same wire, so RX must
            // already be listening when the bytes hit it
            TEST_ASSERT_EQUAL(HAL_OK, uart_receive_dma(TEST_INSTANCE, rx_buf.data(), rx_buf.size(), rx_done_callback, nullptr));
            TEST_ASSERT_EQUAL(HAL_OK, uart_transmit_poll(TEST_INSTANCE, TX_DATA.data(), TX_DATA.size()));

            TEST_ASSERT_TRUE_MESSAGE(wait_for(s_rx_done), "UART loopback RX never completed");
            TEST_ASSERT_EQUAL(HAL_OK, s_rx_err);
            TEST_ASSERT_TRUE(std::equal(TX_DATA.begin(), TX_DATA.end(), rx_buf.begin()));

            TEST_ASSERT_EQUAL(HAL_OK, uart_dma_deinit(TEST_INSTANCE));
            TEST_ASSERT_EQUAL(HAL_OK, uart_deinit(TEST_INSTANCE));
        }

        void single_byte_transmit_loopback() {
            TEST_ASSERT_EQUAL(HAL_OK, uart_init(TEST_INSTANCE, &DEFAULT_CONFIG));
            TEST_ASSERT_EQUAL(HAL_OK, uart_dma_init(TEST_INSTANCE, DMA_PRIORITY_LOW));

            uint8_t rx_byte = 0;
            s_rx_done       = false;
            s_rx_err        = HAL_FAIL;

            TEST_ASSERT_EQUAL(HAL_OK, uart_receive_dma(TEST_INSTANCE, &rx_byte, 1, rx_done_callback, nullptr));
            TEST_ASSERT_EQUAL(HAL_OK, uart_transmit_byte(TEST_INSTANCE, 0xA5));

            TEST_ASSERT_TRUE_MESSAGE(wait_for(s_rx_done), "UART single byte loopback never completed");
            TEST_ASSERT_EQUAL(HAL_OK, s_rx_err);
            TEST_ASSERT_EQUAL_UINT8(0xA5, rx_byte);

            TEST_ASSERT_EQUAL(HAL_OK, uart_dma_deinit(TEST_INSTANCE));
            TEST_ASSERT_EQUAL(HAL_OK, uart_deinit(TEST_INSTANCE));
        }

        void dma_roundtrip_tx_and_rx_with_callbacks() {
            TEST_ASSERT_EQUAL(HAL_OK, uart_init(TEST_INSTANCE, &DEFAULT_CONFIG));
            TEST_ASSERT_EQUAL(HAL_OK, uart_dma_init(TEST_INSTANCE, DMA_PRIORITY_VERY_HIGH));

            constexpr std::array<uint8_t, 16>   TX_DATA = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
            std::array<uint8_t, TX_DATA.size()> rx_buf{};

            s_tx_done = false;
            s_rx_done = false;
            s_tx_err  = HAL_FAIL;
            s_rx_err  = HAL_FAIL;

            TEST_ASSERT_EQUAL(HAL_OK, uart_receive_dma(TEST_INSTANCE, rx_buf.data(), rx_buf.size(), rx_done_callback, nullptr));
            TEST_ASSERT_EQUAL(HAL_OK, uart_transmit_dma(TEST_INSTANCE, TX_DATA.data(), TX_DATA.size(), tx_done_callback, nullptr));

            TEST_ASSERT_TRUE_MESSAGE(wait_for(s_tx_done), "UART DMA TX never completed");
            TEST_ASSERT_TRUE_MESSAGE(wait_for(s_rx_done), "UART DMA RX never completed");
            TEST_ASSERT_EQUAL(HAL_OK, s_tx_err);
            TEST_ASSERT_EQUAL(HAL_OK, s_rx_err);
            TEST_ASSERT_TRUE(std::equal(TX_DATA.begin(), TX_DATA.end(), rx_buf.begin()));

            TEST_ASSERT_EQUAL(HAL_OK, uart_dma_deinit(TEST_INSTANCE));
            TEST_ASSERT_EQUAL(HAL_OK, uart_deinit(TEST_INSTANCE));
        }

        void dma_transfer_without_callback_still_completes() {
            TEST_ASSERT_EQUAL(HAL_OK, uart_init(TEST_INSTANCE, &DEFAULT_CONFIG));
            TEST_ASSERT_EQUAL(HAL_OK, uart_dma_init(TEST_INSTANCE, DMA_PRIORITY_LOW));

            constexpr std::array<uint8_t, 4>    TX_DATA = {0xDE, 0xAD, 0xBE, 0xEF};
            std::array<uint8_t, TX_DATA.size()> rx_buf{};

            TEST_ASSERT_EQUAL(HAL_OK, uart_receive_dma(TEST_INSTANCE, rx_buf.data(), rx_buf.size(), nullptr, nullptr));
            TEST_ASSERT_EQUAL(HAL_OK, uart_transmit_poll(TEST_INSTANCE, TX_DATA.data(), TX_DATA.size()));

            delay_ms(20);
            TEST_ASSERT_TRUE(std::equal(TX_DATA.begin(), TX_DATA.end(), rx_buf.begin()));

            TEST_ASSERT_EQUAL(HAL_OK, uart_dma_deinit(TEST_INSTANCE));
            TEST_ASSERT_EQUAL(HAL_OK, uart_deinit(TEST_INSTANCE));
        }

        void deinit_clears_control_and_baud_registers() {
            TEST_ASSERT_EQUAL(HAL_OK, uart_init(TEST_INSTANCE, &DEFAULT_CONFIG));
            TEST_ASSERT_EQUAL(HAL_OK, uart_deinit(TEST_INSTANCE));

            TEST_ASSERT_FALSE(TEST_INSTANCE->CR1 & USART_CR1_UE);
            TEST_ASSERT_FALSE(TEST_INSTANCE->CR1 & (USART_CR1_TE | USART_CR1_RE | USART_CR1_OVER8));
            TEST_ASSERT_EQUAL_UINT32(0, TEST_INSTANCE->BRR & (USART_BRR_DIV_Mantissa | USART_BRR_DIV_Fraction));
        }

    } // namespace

    void all() {
        LOGI(TAG, "Starting the tests on the UART driver");
        UNITY_BEGIN();

        RUN_TEST(invalid_arg_guards);
        RUN_TEST(clk_enable_toggles_the_correct_bus_bit);
        RUN_TEST(dma_init_rejects_unsupported_instance);
        RUN_TEST(poll_tx_and_dma_rx_loopback_matches_sent_bytes);
        RUN_TEST(single_byte_transmit_loopback);
        RUN_TEST(dma_roundtrip_tx_and_rx_with_callbacks);
        RUN_TEST(dma_transfer_without_callback_still_completes);
        RUN_TEST(deinit_clears_control_and_baud_registers);

        UNITY_END();
        LOGI(TAG, "Done with all tests on the UART driver");
    }

} // namespace test::uart
