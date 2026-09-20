#include "stm32f411xe.h"
#include "Unity/unity.h"

#include "utils/common.h"
#include "drivers/gpio.h"
#include "drivers/spi.h"
#include "test/spi.hpp"
#include "utils/tick.h"
#include "utils/err.h"
#include "utils/log.h"

#include <array>
#include <cstdint>
#include <utility>
#include <algorithm>


namespace test::spi {

    namespace {

        constexpr const char* TAG = "SPI_Test";

        // Instance under test. MOSI and MISO must be physically jumpered together on
        // the board for the loopback tests below to pass.
        SPI_TypeDef* const   TEST_INSTANCE = SPI2;
        GPIO_TypeDef* const  TEST_PORT     = GPIOB;
        constexpr gpio_pin_t TEST_MISO_PIN = GPIO_PIN_14;
        constexpr gpio_pin_t TEST_MOSI_PIN = GPIO_PIN_15;
        constexpr gpio_pin_t TEST_SCLK_PIN = GPIO_PIN_10;

        const spi_master_config_t DEFAULT_CONFIG = {
            .cpol      = false,
            .cpha      = false,
            .use_miso  = true,
            .use_mosi  = true,
            .data_size = SPI_DATA_8_BITS,
            .prescaler = SPI_PRESCALER_DIV4,
            .miso_pin  = BOARD_SPI2_MISO_PB14,
            .mosi_pin  = BOARD_SPI2_MOSI_PB15,
            .sclk_pin  = BOARD_SPI2_SCLK_PB10,
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
            uint32_t timeout = TIMEOUT;
            while (!flag && --timeout);
            return flag;
        }

        inline uint32_t get_gpio_moder(gpio_pin_t pin) {
            return (TEST_PORT->MODER >> (std::to_underlying(pin) * 2U)) & 0b11U;
        }

        // TESTS
        void invalid_arg_guards() {
            constexpr std::array<uint8_t, 4> data{};
            std::array<uint8_t, 4>           rx_buf{};

            spi_master_config_t no_pins_config = DEFAULT_CONFIG;
            no_pins_config.use_miso            = false;
            no_pins_config.use_mosi            = false;

            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, spi_master_init(nullptr, &DEFAULT_CONFIG));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, spi_master_init(TEST_INSTANCE, nullptr));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, spi_master_init(TEST_INSTANCE, &no_pins_config));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, spi_master_deinit(nullptr));

            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, spi_master_transmit_poll(nullptr, data.data(), data.size()));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, spi_master_transmit_poll(TEST_INSTANCE, nullptr, data.size()));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, spi_master_transmit_poll(TEST_INSTANCE, data.data(), 0));

            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, spi_master_receive_poll(nullptr, rx_buf.data(), rx_buf.size()));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, spi_master_receive_poll(TEST_INSTANCE, nullptr, rx_buf.size()));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, spi_master_receive_poll(TEST_INSTANCE, rx_buf.data(), 0));

            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, spi_master_transceive_poll(nullptr, data.data(), rx_buf.data(), data.size()));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, spi_master_transceive_poll(TEST_INSTANCE, nullptr, rx_buf.data(), data.size()));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, spi_master_transceive_poll(TEST_INSTANCE, data.data(), nullptr, data.size()));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, spi_master_transceive_poll(TEST_INSTANCE, data.data(), rx_buf.data(), 0));

            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, spi_master_transmit_dma(nullptr, data.data(), data.size(), nullptr, nullptr));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, spi_master_transmit_dma(TEST_INSTANCE, nullptr, data.size(), nullptr, nullptr));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, spi_master_transmit_dma(TEST_INSTANCE, data.data(), 0, nullptr, nullptr));

            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, spi_master_receive_dma(nullptr, rx_buf.data(), rx_buf.size(), nullptr, nullptr));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, spi_master_receive_dma(TEST_INSTANCE, nullptr, rx_buf.size(), nullptr, nullptr));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, spi_master_receive_dma(TEST_INSTANCE, rx_buf.data(), 0, nullptr, nullptr));

            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, spi_master_transceive_dma(nullptr, data.data(), rx_buf.data(), data.size(), nullptr, nullptr));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, spi_master_transceive_dma(TEST_INSTANCE, nullptr, rx_buf.data(), data.size(), nullptr, nullptr));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, spi_master_transceive_dma(TEST_INSTANCE, data.data(), nullptr, data.size(), nullptr, nullptr));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, spi_master_transceive_dma(TEST_INSTANCE, data.data(), rx_buf.data(), 0, nullptr, nullptr));

            // A handle with no DMA mapping (or an unrecognized handle entirely) must be rejected too
            auto* const bogus_handle = reinterpret_cast<SPI_TypeDef*>(0x1);
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, spi_master_dma_init(bogus_handle, DMA_PRIORITY_LOW));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, spi_master_dma_deinit(bogus_handle));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, spi_master_transmit_dma(bogus_handle, data.data(), data.size(), nullptr, nullptr));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, spi_master_receive_dma(bogus_handle, rx_buf.data(), rx_buf.size(), nullptr, nullptr));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG,
                              spi_master_transceive_dma(bogus_handle, data.data(), rx_buf.data(), data.size(), nullptr, nullptr));
        }

        void clk_enable_toggles_the_correct_bus_bit() {
            spix_clk_enable(SPI1, false);
            TEST_ASSERT_FALSE(RCC->APB2ENR & RCC_APB2ENR_SPI1EN);
            spix_clk_enable(SPI1, true);
            TEST_ASSERT_TRUE(RCC->APB2ENR & RCC_APB2ENR_SPI1EN);
            spix_clk_enable(SPI1, false);

            spix_clk_enable(SPI2, false);
            TEST_ASSERT_FALSE(RCC->APB1ENR & RCC_APB1ENR_SPI2EN);
            spix_clk_enable(SPI2, true);
            TEST_ASSERT_TRUE(RCC->APB1ENR & RCC_APB1ENR_SPI2EN);
            // Leave SPI2 enabled: the loopback tests below need it

            spix_clk_enable(SPI3, false);
            TEST_ASSERT_FALSE(RCC->APB1ENR & RCC_APB1ENR_SPI3EN);
            spix_clk_enable(SPI3, true);
            TEST_ASSERT_TRUE(RCC->APB1ENR & RCC_APB1ENR_SPI3EN);
            spix_clk_enable(SPI3, false);

            spix_clk_enable(SPI4, false);
            TEST_ASSERT_FALSE(RCC->APB2ENR & RCC_APB2ENR_SPI4EN);
            spix_clk_enable(SPI4, true);
            TEST_ASSERT_TRUE(RCC->APB2ENR & RCC_APB2ENR_SPI4EN);
            spix_clk_enable(SPI4, false);

            spix_clk_enable(SPI5, false);
            TEST_ASSERT_FALSE(RCC->APB2ENR & RCC_APB2ENR_SPI5EN);
            spix_clk_enable(SPI5, true);
            TEST_ASSERT_TRUE(RCC->APB2ENR & RCC_APB2ENR_SPI5EN);
            spix_clk_enable(SPI5, false);

            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, spix_clk_enable(reinterpret_cast<SPI_TypeDef*>(0x1), true));
        }

        void dma_init_rejects_unsupported_instances() {
            // SPI1 and SPI5 have no DMA streams mapped to them
            spix_clk_enable(SPI1, true);
            TEST_ASSERT_EQUAL(HAL_ERR_NOT_SUPPORTED, spi_master_dma_init(SPI1, DMA_PRIORITY_LOW));
            TEST_ASSERT_EQUAL(HAL_ERR_NOT_SUPPORTED, spi_master_dma_deinit(SPI1));
            spix_clk_enable(SPI1, false);

            spix_clk_enable(SPI5, true);
            TEST_ASSERT_EQUAL(HAL_ERR_NOT_SUPPORTED, spi_master_dma_init(SPI5, DMA_PRIORITY_LOW));
            TEST_ASSERT_EQUAL(HAL_ERR_NOT_SUPPORTED, spi_master_dma_deinit(SPI5));
            spix_clk_enable(SPI5, false);
        }

        void only_mosi_leaves_miso_pin_untouched() {
            const uint32_t miso_mode_before = get_gpio_moder(TEST_MISO_PIN);

            spi_master_config_t config = DEFAULT_CONFIG;
            config.use_miso            = false;
            config.use_mosi            = true;

            TEST_ASSERT_EQUAL(HAL_OK, spi_master_init(TEST_INSTANCE, &config));

            // The GPIO alternate function mode encoding is 0b10
            TEST_ASSERT_EQUAL_UINT32(miso_mode_before, get_gpio_moder(TEST_MISO_PIN));
            TEST_ASSERT_EQUAL_UINT32(0b10, get_gpio_moder(TEST_MOSI_PIN));

            TEST_ASSERT_EQUAL(HAL_OK, spi_master_deinit(TEST_INSTANCE));
        }

        void only_miso_leaves_mosi_pin_untouched() {
            const uint32_t mosi_mode_before = get_gpio_moder(TEST_MOSI_PIN);

            spi_master_config_t config = DEFAULT_CONFIG;
            config.use_miso            = true;
            config.use_mosi            = false;

            TEST_ASSERT_EQUAL(HAL_OK, spi_master_init(TEST_INSTANCE, &config));

            TEST_ASSERT_EQUAL_UINT32(mosi_mode_before, get_gpio_moder(TEST_MOSI_PIN));
            TEST_ASSERT_EQUAL_UINT32(0b10, get_gpio_moder(TEST_MISO_PIN));

            TEST_ASSERT_EQUAL(HAL_OK, spi_master_deinit(TEST_INSTANCE));
        }

        void transceive_poll_loopback_8bit() {
            TEST_ASSERT_EQUAL(HAL_OK, spi_master_init(TEST_INSTANCE, &DEFAULT_CONFIG));

            constexpr std::array<uint8_t, 16>   TX_DATA = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
            std::array<uint8_t, TX_DATA.size()> rx_buf{};

            TEST_ASSERT_EQUAL(HAL_OK, spi_master_transceive_poll(TEST_INSTANCE, TX_DATA.data(), rx_buf.data(), TX_DATA.size()));
            TEST_ASSERT_TRUE(std::equal(TX_DATA.begin(), TX_DATA.end(), rx_buf.begin()));

            TEST_ASSERT_EQUAL(HAL_OK, spi_master_deinit(TEST_INSTANCE));
        }

        void transceive_poll_loopback_16bit() {
            spi_master_config_t config = DEFAULT_CONFIG;
            config.data_size           = SPI_DATA_16_BITS;
            TEST_ASSERT_EQUAL(HAL_OK, spi_master_init(TEST_INSTANCE, &config));

            constexpr std::array<uint16_t, 8>    TX_DATA = {0x0102, 0x0304, 0x0506, 0x0708, 0xDEAD, 0xBEEF, 0xCAFE, 0xF00D};
            std::array<uint16_t, TX_DATA.size()> rx_buf{};

            TEST_ASSERT_EQUAL(HAL_OK, spi_master_transceive_poll(TEST_INSTANCE, TX_DATA.data(), rx_buf.data(), TX_DATA.size()));
            TEST_ASSERT_TRUE(std::equal(TX_DATA.begin(), TX_DATA.end(), rx_buf.begin()));

            TEST_ASSERT_EQUAL(HAL_OK, spi_master_deinit(TEST_INSTANCE));
        }

        void all_cpol_cpha_combinations_loopback() {
            constexpr std::array<uint8_t, 4>    TX_DATA = {0xA5, 0x5A, 0x3C, 0xC3};
            std::array<uint8_t, TX_DATA.size()> rx_buf{};

            for (bool cpol : {false, true}) {
                for (bool cpha : {false, true}) {
                    spi_master_config_t config = DEFAULT_CONFIG;
                    config.cpol                = cpol;
                    config.cpha                = cpha;

                    TEST_ASSERT_EQUAL(HAL_OK, spi_master_init(TEST_INSTANCE, &config));
                    TEST_ASSERT_EQUAL(HAL_OK, spi_master_transceive_poll(TEST_INSTANCE, TX_DATA.data(), rx_buf.data(), TX_DATA.size()));
                    TEST_ASSERT_TRUE(std::equal(TX_DATA.begin(), TX_DATA.end(), rx_buf.begin()));
                    TEST_ASSERT_EQUAL(HAL_OK, spi_master_deinit(TEST_INSTANCE));
                }
            }
        }

        void dma_roundtrip_transceive_with_callback() {
            TEST_ASSERT_EQUAL(HAL_OK, spi_master_init(TEST_INSTANCE, &DEFAULT_CONFIG));
            TEST_ASSERT_EQUAL(HAL_OK, spi_master_dma_init(TEST_INSTANCE, DMA_PRIORITY_HIGH));

            constexpr std::array<uint8_t, 16>   TX_DATA = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
            std::array<uint8_t, TX_DATA.size()> rx_buf{};

            s_tx_done = false;
            s_tx_err  = HAL_FAIL;

            TEST_ASSERT_EQUAL(HAL_OK,
                              spi_master_transceive_dma(TEST_INSTANCE, TX_DATA.data(), rx_buf.data(), TX_DATA.size(), tx_done_callback, nullptr));

            TEST_ASSERT_TRUE_MESSAGE(wait_for(s_tx_done), "SPI DMA transceive never completed");
            TEST_ASSERT_EQUAL(HAL_OK, s_tx_err);
            TEST_ASSERT_TRUE(std::equal(TX_DATA.begin(), TX_DATA.end(), rx_buf.begin()));

            TEST_ASSERT_EQUAL(HAL_OK, spi_master_dma_deinit(TEST_INSTANCE));
            TEST_ASSERT_EQUAL(HAL_OK, spi_master_deinit(TEST_INSTANCE));
        }

        void dma_transfer_without_callback_still_completes() {
            TEST_ASSERT_EQUAL(HAL_OK, spi_master_init(TEST_INSTANCE, &DEFAULT_CONFIG));
            TEST_ASSERT_EQUAL(HAL_OK, spi_master_dma_init(TEST_INSTANCE, DMA_PRIORITY_LOW));

            constexpr std::array<uint8_t, 4>    TX_DATA = {0xDE, 0xAD, 0xBE, 0xEF};
            std::array<uint8_t, TX_DATA.size()> rx_buf{};

            TEST_ASSERT_EQUAL(HAL_OK, spi_master_transceive_dma(TEST_INSTANCE, TX_DATA.data(), rx_buf.data(), TX_DATA.size(), nullptr, nullptr));

            // Poll until we are sure the transfer should have been
            // completed. 20ms should be more than enough in this case
            delay_ms(20);

            TEST_ASSERT_FALSE(TEST_INSTANCE->CR1 & SPI_CR1_SPE);
            TEST_ASSERT_TRUE(std::equal(TX_DATA.begin(), TX_DATA.end(), rx_buf.begin()));

            TEST_ASSERT_EQUAL(HAL_OK, spi_master_dma_deinit(TEST_INSTANCE));
            TEST_ASSERT_EQUAL(HAL_OK, spi_master_deinit(TEST_INSTANCE));
        }

        void deinit_clears_control_registers() {
            TEST_ASSERT_EQUAL(HAL_OK, spi_master_init(TEST_INSTANCE, &DEFAULT_CONFIG));
            TEST_ASSERT_EQUAL(HAL_OK, spi_master_deinit(TEST_INSTANCE));

            TEST_ASSERT_FALSE(TEST_INSTANCE->CR1 & (SPI_CR1_SPE | SPI_CR1_MSTR | SPI_CR1_BR | SPI_CR1_DFF));
            TEST_ASSERT_FALSE(TEST_INSTANCE->CR2 & (SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN));
        }

    } // namespace

    void all() {
        LOGI(TAG, "Starting the tests on the SPI driver");
        UNITY_BEGIN();

        spix_clk_enable(TEST_INSTANCE, true);

        RUN_TEST(invalid_arg_guards);
        RUN_TEST(clk_enable_toggles_the_correct_bus_bit);
        RUN_TEST(dma_init_rejects_unsupported_instances);
        RUN_TEST(only_mosi_leaves_miso_pin_untouched);
        RUN_TEST(only_miso_leaves_mosi_pin_untouched);
        RUN_TEST(transceive_poll_loopback_8bit);
        RUN_TEST(transceive_poll_loopback_16bit);
        RUN_TEST(all_cpol_cpha_combinations_loopback);
        RUN_TEST(dma_roundtrip_transceive_with_callback);
        RUN_TEST(dma_transfer_without_callback_still_completes);
        RUN_TEST(deinit_clears_control_registers);

        spix_clk_enable(TEST_INSTANCE, false);

        UNITY_END();
        LOGI(TAG, "Done with all tests on the SPI driver");
    }

} // namespace test::spi
