#include "stm32f411xe.h"
#include "Unity/unity.h"

#include "drivers/gpio.h"
#include "drivers/i2c.h"
#include "test/i2c.hpp"
#include "utils/tick.h"
#include "utils/err.h"
#include "utils/log.h"

#include <array>
#include <cstdint>


namespace test::i2c {

    namespace {

        constexpr const char* TAG = "I2C_Test";

        const i2c_master_config_t PORT_CONFIG = {
            .use_pullups    = true,
            .frequency      = I2C_FREQ_100KHz,
            .digital_filter = I2C_DIGITAL_FILTER_0,
            .sda_pin        = GPIO_PIN_9,
            .scl_pin        = GPIO_PIN_8,
            .gpio_port      = GPIOB,
        };

        auto* const       TEST_PORT         = I2C1;
        constexpr uint8_t AHT20_I2C_ADDRESS = 0x38;

        // TESTS
        void invalid_arg_guards() {
            // This test doesn't require any other physical component, so it doesn't test
            // bus activity. It only checks and tests invalid arguments to the driver.

            constexpr std::array<uint8_t, 4> data{};
            std::array<uint8_t, 4>           rx_buf{};

            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, i2cx_clk_enable(reinterpret_cast<I2C_TypeDef*>(1), true));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, i2c_master_init(nullptr, &PORT_CONFIG));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, i2c_master_init(TEST_PORT, nullptr));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, i2c_master_deinit(nullptr));

            TEST_ASSERT_EQUAL(HAL_OK, i2cx_clk_enable(TEST_PORT, true));
            TEST_ASSERT_EQUAL(HAL_OK, i2c_master_init(TEST_PORT, &PORT_CONFIG));
            TEST_ASSERT_EQUAL(HAL_OK, i2c_master_deinit(TEST_PORT));
            TEST_ASSERT_EQUAL(HAL_OK, i2cx_clk_enable(TEST_PORT, false));

            // Frequency must be one of the two supported presets
            i2c_master_config_t bad_freq_config = PORT_CONFIG;
            bad_freq_config.frequency           = static_cast<i2c_frequency_t>(0);
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, i2c_master_init(TEST_PORT, &bad_freq_config));

            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, i2c_master_transmit(nullptr, 0x50, data.data(), data.size()));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, i2c_master_transmit(TEST_PORT, 0, data.data(), data.size()));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, i2c_master_transmit(TEST_PORT, 0x80, data.data(), data.size()));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, i2c_master_transmit(TEST_PORT, 0x50, nullptr, data.size()));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, i2c_master_transmit(TEST_PORT, 0x50, data.data(), 0));

            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, i2c_master_receive(nullptr, 0x50, rx_buf.data(), rx_buf.size()));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, i2c_master_receive(TEST_PORT, 0, rx_buf.data(), rx_buf.size()));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, i2c_master_receive(TEST_PORT, 0x80, rx_buf.data(), rx_buf.size()));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, i2c_master_receive(TEST_PORT, 0x50, nullptr, rx_buf.size()));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, i2c_master_receive(TEST_PORT, 0x50, rx_buf.data(), 0));

            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, i2c_master_transceive(nullptr, 0x50, data.data(), data.size(), rx_buf.data(), rx_buf.size()));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, i2c_master_transceive(TEST_PORT, 0, data.data(), data.size(), rx_buf.data(), rx_buf.size()));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, i2c_master_transceive(TEST_PORT, 0x80, data.data(), data.size(), rx_buf.data(), rx_buf.size()));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, i2c_master_transceive(TEST_PORT, 0x50, nullptr, data.size(), rx_buf.data(), rx_buf.size()));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, i2c_master_transceive(TEST_PORT, 0x50, data.data(), 0, rx_buf.data(), rx_buf.size()));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, i2c_master_transceive(TEST_PORT, 0x50, data.data(), data.size(), nullptr, rx_buf.size()));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, i2c_master_transceive(TEST_PORT, 0x50, data.data(), data.size(), rx_buf.data(), 0));
        }

        // These tests requires another physical component to be present: the AHT20 sensor.
        // They test the communication with another I2C device as a way to model real world usage.

        void aht20_inits_fine() {
            // This test must run first before any other AHT20 test since it sets up the bus and initializes the AHT20

            // Initialize the I2C bus to be used
            TEST_ASSERT_EQUAL(HAL_OK, i2cx_clk_enable(TEST_PORT, true));
            TEST_ASSERT_EQUAL(HAL_OK, i2c_master_init(TEST_PORT, &PORT_CONFIG));

            // The AHT20 requires a startup delay of about 40ms
            delay_ms(40);

            // Transmit the initialization sequence for the AHT20
            constexpr std::array<uint8_t, 3> tx_buffer = {0xBE, 0x08, 0x00};
            TEST_ASSERT_EQUAL(HAL_OK, i2c_master_transmit(TEST_PORT, AHT20_I2C_ADDRESS, tx_buffer.data(), tx_buffer.size()));

            // Get the initialization status byte from the AHt20
            constexpr uint8_t tx_init_rq = 0x71;
            uint8_t           rx_init_rq = 0;
            TEST_ASSERT_EQUAL(HAL_OK,
                              i2c_master_transceive(TEST_PORT, AHT20_I2C_ADDRESS, &tx_init_rq, sizeof(tx_init_rq), &rx_init_rq, sizeof(rx_init_rq)));

            // If the third bit of the received byte is 1, the AHT20 is initialized
            TEST_ASSERT_EQUAL_UINT32(1U, ((rx_init_rq >> 3U) & 1U));

            // The AHT20 requires another delay of about 15ms after it is initialized before any reading can take place
            delay_ms(15);
        }

        void aht20_read_works() {
            // Buffer which contains the values to trigger a read from the AHT20
            constexpr std::array<uint8_t, 3> tx_trigger = {0xAC, 0x33, 0x00};
            TEST_ASSERT_EQUAL(HAL_OK, i2c_master_transmit(TEST_PORT, AHT20_I2C_ADDRESS, tx_trigger.data(), tx_trigger.size()));

            // Delay to allow the AHT20 take measurements. The datasheet recommends 80ms. This is not a production
            // grade driver nor does it claims to be one, so we just poll for 100ms and then read the measured data.
            delay_ms(100);

            // Get the raw measured data after the 100ms
            std::array<uint8_t, 7> rx_data{};
            TEST_ASSERT_EQUAL(HAL_OK, i2c_master_receive(TEST_PORT, AHT20_I2C_ADDRESS, rx_data.data(), rx_data.size()));

            // Run the CRC8 checksum on the data
            uint8_t crc8 = 0xFF;
            for (uint8_t i = 0; i < 6; i++) {
                crc8 ^= rx_data[i];
                for (uint8_t j = 0; j < 8; j++) {
                    crc8 = (crc8 & 0x80) ? ((uint8_t)(crc8 << 1) ^ 0x31) : (uint8_t)(crc8 << 1);
                }
            }
            TEST_ASSERT_EQUAL_UINT8(crc8, rx_data[6]);

            const uint32_t raw_hmdt = (uint32_t)((rx_data[1] << 12) | (rx_data[2] << 4) | ((rx_data[3] >> 4) & 0xF));
            const uint32_t raw_temp = (uint32_t)(((rx_data[3] & 0xF) << 16) | (rx_data[4] << 8) | (rx_data[5]));

            // Convert the raw readings to temperature in celsius and humidity in percent
            constexpr uint32_t conversion_factor = (1 << 20);
            const float        humidity          = ((float)raw_hmdt / conversion_factor) * 100;
            const float        temperature       = (((float)raw_temp / conversion_factor) * 200) - 50;

            // Validate the ranges of the received data with the specified ranges from the datasheet
            TEST_ASSERT_TRUE(humidity >= 0 && humidity <= 100);
            TEST_ASSERT_TRUE(temperature >= -40 && temperature <= 80);

            LOGI(TAG, "Humidity: %f%%. Temperature: %fC", (double)humidity, (double)temperature);
        }

        void aht20_deinits_fine() {
            // This test must run last after all other AHT20 tests since it tears up the bus and deinitializes the AHT20
            constexpr uint8_t aht20_reset_code = 0xBA;
            TEST_ASSERT_EQUAL(HAL_OK, i2c_master_transmit(TEST_PORT, AHT20_I2C_ADDRESS, &aht20_reset_code, sizeof(aht20_reset_code)));
            TEST_ASSERT_EQUAL(HAL_OK, i2c_master_deinit(TEST_PORT));
            TEST_ASSERT_EQUAL(HAL_OK, i2cx_clk_enable(TEST_PORT, false));
        }

    } // namespace

    void all() {
        LOGI(TAG, "Starting the tests on the I2C driver");
        UNITY_BEGIN();

        RUN_TEST(invalid_arg_guards);
        RUN_TEST(aht20_inits_fine);
        RUN_TEST(aht20_read_works);
        RUN_TEST(aht20_deinits_fine);
        RUN_TEST(aht20_inits_fine);
        RUN_TEST(aht20_read_works);
        RUN_TEST(aht20_read_works);
        RUN_TEST(aht20_read_works);
        RUN_TEST(aht20_deinits_fine);
        RUN_TEST(aht20_inits_fine);
        RUN_TEST(aht20_read_works);
        RUN_TEST(aht20_read_works);
        RUN_TEST(aht20_deinits_fine);

        UNITY_END();
        LOGI(TAG, "Done with the I2C driver tests");
    }

} // namespace test::i2c
