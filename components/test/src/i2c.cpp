#include "stm32f411xe.h"
#include "Unity/unity.h"

#include "drivers/gpio.h"
#include "drivers/i2c.h"
#include "test/i2c.hpp"
#include "utils/err.h"
#include "utils/log.h"

#include <array>
#include <cstdint>


namespace test::i2c {

    namespace {

        constexpr const char* TAG = "I2C_Test";

        // Only used to give i2c_master_init(...) a structurally valid config to
        // validate against - no bus activity happens in these tests, so the pins
        // and port below don't need to correspond to real wiring yet.
        const i2c_master_config_t VALID_CONFIG = {
            .use_pullups    = true,
            .frequency      = I2C_FREQ_100KHz,
            .digital_filter = I2C_DIGITAL_FILTER_0,
            .sda_pin        = GPIO_PIN_9,
            .scl_pin        = GPIO_PIN_8,
            .gpio_port      = GPIOB,
        };

        // TESTS
        // Deeper functional coverage (device present/absent, byte-count boundaries in
        // rx_trans(), repeated-start transceive against the AHT20) is deferred until
        // that slave is wired up. For now this only locks down the argument-guard
        // contract, which doesn't require any bus activity.
        void invalid_arg_guards() {
            constexpr std::array<uint8_t, 4> data{};
            std::array<uint8_t, 4>           rx_buf{};

            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, i2c_master_init(nullptr, &VALID_CONFIG));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, i2c_master_init(I2C1, nullptr));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, i2c_master_deinit(nullptr));

            // Frequency must be one of the two supported presets
            i2c_master_config_t bad_freq_config = VALID_CONFIG;
            bad_freq_config.frequency           = static_cast<i2c_frequency_t>(0);
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, i2c_master_init(I2C1, &bad_freq_config));

            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, i2c_master_transmit(nullptr, 0x50, data.data(), data.size()));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, i2c_master_transmit(I2C1, 0, data.data(), data.size()));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, i2c_master_transmit(I2C1, 0x80, data.data(), data.size()));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, i2c_master_transmit(I2C1, 0x50, nullptr, data.size()));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, i2c_master_transmit(I2C1, 0x50, data.data(), 0));

            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, i2c_master_receive(nullptr, 0x50, rx_buf.data(), rx_buf.size()));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, i2c_master_receive(I2C1, 0, rx_buf.data(), rx_buf.size()));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, i2c_master_receive(I2C1, 0x80, rx_buf.data(), rx_buf.size()));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, i2c_master_receive(I2C1, 0x50, nullptr, rx_buf.size()));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, i2c_master_receive(I2C1, 0x50, rx_buf.data(), 0));

            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, i2c_master_transceive(nullptr, 0x50, data.data(), data.size(), rx_buf.data(), rx_buf.size()));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, i2c_master_transceive(I2C1, 0, data.data(), data.size(), rx_buf.data(), rx_buf.size()));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, i2c_master_transceive(I2C1, 0x80, data.data(), data.size(), rx_buf.data(), rx_buf.size()));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, i2c_master_transceive(I2C1, 0x50, nullptr, data.size(), rx_buf.data(), rx_buf.size()));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, i2c_master_transceive(I2C1, 0x50, data.data(), 0, rx_buf.data(), rx_buf.size()));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, i2c_master_transceive(I2C1, 0x50, data.data(), data.size(), nullptr, rx_buf.size()));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, i2c_master_transceive(I2C1, 0x50, data.data(), data.size(), rx_buf.data(), 0));

            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, i2cx_clk_enable(reinterpret_cast<I2C_TypeDef*>(0x1), true));
        }

    } // namespace

    void all() {
        LOGI(TAG, "Starting the tests on the I2C driver (argument validation only, for now)");
        UNITY_BEGIN();

        RUN_TEST(invalid_arg_guards);

        UNITY_END();
        LOGI(TAG, "Done with the I2C driver argument validation tests");
    }

} // namespace test::i2c
