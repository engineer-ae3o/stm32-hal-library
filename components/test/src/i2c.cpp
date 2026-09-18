#include "stm32f411xe.h"
#include "Unity/unity.h"

#include "utils/common.h"
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

        I2C_TypeDef* const TEST_PORT         = I2C1;
        constexpr uint8_t  AHT20_I2C_ADDRESS = 0x38;

        const i2c_master_config_t PORT_CONFIG = {
            .use_pullups    = true,
            .frequency      = I2C_FREQ_100kHz,
            .digital_filter = I2C_DIGITAL_FILTER_5,
            .sda_pin        = BOARD_I2C1_SDA_PB7,
            .scl_pin        = BOARD_I2C1_SCL_PB6,
        };

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

        void clk_enable_toggles_the_correct_bus_bit() {
            i2cx_clk_enable(I2C1, false);
            TEST_ASSERT_FALSE(RCC->APB1ENR & RCC_APB1ENR_I2C1EN);
            i2cx_clk_enable(I2C1, true);
            TEST_ASSERT_TRUE(RCC->APB1ENR & RCC_APB1ENR_I2C1EN);
            i2cx_clk_enable(I2C1, false);

            i2cx_clk_enable(I2C2, false);
            TEST_ASSERT_FALSE(RCC->APB1ENR & RCC_APB1ENR_I2C2EN);
            i2cx_clk_enable(I2C2, true);
            TEST_ASSERT_TRUE(RCC->APB1ENR & RCC_APB1ENR_I2C2EN);
            i2cx_clk_enable(I2C2, false);

            i2cx_clk_enable(I2C3, false);
            TEST_ASSERT_FALSE(RCC->APB1ENR & RCC_APB1ENR_I2C3EN);
            i2cx_clk_enable(I2C3, true);
            TEST_ASSERT_TRUE(RCC->APB1ENR & RCC_APB1ENR_I2C3EN);
            i2cx_clk_enable(I2C3, false);

            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, i2cx_clk_enable(reinterpret_cast<I2C_TypeDef*>(0x1), true));
        }

        void deinit_clears_control_registers() {
            TEST_ASSERT_EQUAL(HAL_OK, i2cx_clk_enable(TEST_PORT, true));
            TEST_ASSERT_EQUAL(HAL_OK, i2c_master_init(TEST_PORT, &PORT_CONFIG));
            TEST_ASSERT_EQUAL(HAL_OK, i2c_master_deinit(TEST_PORT));

            TEST_ASSERT_FALSE(TEST_PORT->CR1 & I2C_CR1_PE);
            TEST_ASSERT_FALSE(TEST_PORT->CR1 & I2C_CR1_ACK);
            TEST_ASSERT_EQUAL_UINT32(0, TEST_PORT->CR2 & I2C_CR2_FREQ);
            TEST_ASSERT_EQUAL_UINT32(0, TEST_PORT->CCR & (I2C_CCR_FS | I2C_CCR_DUTY | I2C_CCR_CCR));
            TEST_ASSERT_EQUAL_UINT32(0, TEST_PORT->FLTR & (I2C_FLTR_DNF | I2C_FLTR_ANOFF));
            TEST_ASSERT_EQUAL_UINT32(0, TEST_PORT->TRISE & I2C_TRISE_TRISE);

            TEST_ASSERT_EQUAL(HAL_OK, i2cx_clk_enable(TEST_PORT, false));
        }

        void init_selects_fast_mode_only_at_400khz() {
            i2c_master_config_t config = PORT_CONFIG;

            TEST_ASSERT_EQUAL(HAL_OK, i2cx_clk_enable(TEST_PORT, true));

            // At 100kHz, FS/DUTY must stay clear (standard mode) and CCR must still be programmed
            config.frequency = I2C_FREQ_100kHz;
            TEST_ASSERT_EQUAL(HAL_OK, i2c_master_init(TEST_PORT, &config));
            TEST_ASSERT_FALSE(TEST_PORT->CCR & (I2C_CCR_FS | I2C_CCR_DUTY));
            TEST_ASSERT_TRUE((TEST_PORT->CCR & I2C_CCR_CCR) != 0);
            const uint32_t trise_100khz = TEST_PORT->TRISE & I2C_TRISE_TRISE;
            TEST_ASSERT_TRUE(trise_100khz != 0);
            TEST_ASSERT_EQUAL(HAL_OK, i2c_master_deinit(TEST_PORT));

            // At 400kHz, both FS and DUTY (16:9) must be set, and the rise time budget shrinks
            config.frequency = I2C_FREQ_400kHz;
            TEST_ASSERT_EQUAL(HAL_OK, i2c_master_init(TEST_PORT, &config));
            TEST_ASSERT_TRUE(TEST_PORT->CCR & I2C_CCR_FS);
            TEST_ASSERT_TRUE(TEST_PORT->CCR & I2C_CCR_DUTY);
            TEST_ASSERT_TRUE((TEST_PORT->CCR & I2C_CCR_CCR) != 0);
            const uint32_t trise_400khz = TEST_PORT->TRISE & I2C_TRISE_TRISE;
            TEST_ASSERT_TRUE(trise_400khz != 0);
            TEST_ASSERT_TRUE(trise_400khz < trise_100khz);
            TEST_ASSERT_EQUAL(HAL_OK, i2c_master_deinit(TEST_PORT));

            TEST_ASSERT_EQUAL(HAL_OK, i2cx_clk_enable(TEST_PORT, false));
        }

        void init_programs_the_configured_digital_filter() {
            i2c_master_config_t config = PORT_CONFIG;
            config.digital_filter      = I2C_DIGITAL_FILTER_9;

            TEST_ASSERT_EQUAL(HAL_OK, i2cx_clk_enable(TEST_PORT, true));
            TEST_ASSERT_EQUAL(HAL_OK, i2c_master_init(TEST_PORT, &config));

            TEST_ASSERT_EQUAL_UINT32(I2C_DIGITAL_FILTER_9, (TEST_PORT->FLTR & I2C_FLTR_DNF) >> I2C_FLTR_DNF_Pos);

            TEST_ASSERT_EQUAL(HAL_OK, i2c_master_deinit(TEST_PORT));
            TEST_ASSERT_EQUAL(HAL_OK, i2cx_clk_enable(TEST_PORT, false));
        }

        void transmit_to_unaddressed_device_returns_device_not_found() {
            // No physical device on the bus should ACK this address.
            constexpr uint8_t UNUSED_ADDRESS = 0x1A;
            constexpr uint8_t data           = 0x69;

            TEST_ASSERT_EQUAL(HAL_OK, i2cx_clk_enable(TEST_PORT, true));
            TEST_ASSERT_EQUAL(HAL_OK, i2c_master_init(TEST_PORT, &PORT_CONFIG));

            TEST_ASSERT_EQUAL(HAL_ERR_I2C_DEVICE_NOT_FOUND, i2c_master_transmit(TEST_PORT, UNUSED_ADDRESS, &data, sizeof(data)));

            TEST_ASSERT_EQUAL(HAL_OK, i2c_master_deinit(TEST_PORT));
            TEST_ASSERT_EQUAL(HAL_OK, i2cx_clk_enable(TEST_PORT, false));
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

        void read_attempt_from_aht20() {
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

        void transmit_and_receive_reject_a_busy_bus() {
            // Manually drive a start condition and deliberately withhold the stop condition,
            // so the peripheral's own BUSY flag (line-state based, not just internal state)
            // gets set the same way it would during genuine bus contention.
            constexpr uint8_t tx_data = 0xFF;
            uint8_t           rx_data = 0;

            TEST_ASSERT_EQUAL(HAL_OK, i2cx_clk_enable(TEST_PORT, true));
            TEST_ASSERT_EQUAL(HAL_OK, i2c_master_init(TEST_PORT, &PORT_CONFIG));

            TEST_PORT->CR1 |= I2C_CR1_PE;
            TEST_PORT->CR1 |= I2C_CR1_START;

            uint32_t timeout = TIMEOUT;
            while (!(TEST_PORT->SR1 & I2C_SR1_SB) && --timeout);
            TEST_ASSERT_TRUE_MESSAGE(timeout != 0, "Never observed SB after manually issuing a start condition");
            (void)TEST_PORT->SR1; // Clear SB the same way the driver does

            TEST_ASSERT_TRUE_MESSAGE(TEST_PORT->SR2 & I2C_SR2_BUSY, "Bus did not report BUSY after an unterminated start condition");

            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_STATE, i2c_master_transmit(TEST_PORT, AHT20_I2C_ADDRESS, &tx_data, sizeof(tx_data)));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_STATE, i2c_master_receive(TEST_PORT, AHT20_I2C_ADDRESS, &rx_data, sizeof(rx_data)));
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_STATE,
                              i2c_master_transceive(TEST_PORT, AHT20_I2C_ADDRESS, &tx_data, sizeof(tx_data), &rx_data, sizeof(rx_data)));

            // Release the bus so later tests don't inherit a stuck BUSY flag
            TEST_PORT->CR1 |= I2C_CR1_STOP;
            delay_ms(1);
            TEST_PORT->CR1 &= ~I2C_CR1_PE;

            TEST_ASSERT_EQUAL(HAL_OK, i2c_master_deinit(TEST_PORT));
            TEST_ASSERT_EQUAL(HAL_OK, i2cx_clk_enable(TEST_PORT, false));
        }

        void receive_handles_one_and_two_byte_transfers() {
            // rx_trans() branches on size (1, 2, or >2 remaining bytes) and each branch has its
            // own ACK/POS/stop sequencing. Every other test here requests all 7 bytes, so this
            // is the only place the 1- and 2-byte branches get exercised at all.

            // Trigger a measurement so the AHT20 has fresh bytes queued up to read back
            constexpr std::array<uint8_t, 3> tx_trigger = {0xAC, 0x33, 0x00};
            TEST_ASSERT_EQUAL(HAL_OK, i2c_master_transmit(TEST_PORT, AHT20_I2C_ADDRESS, tx_trigger.data(), tx_trigger.size()));
            delay_ms(100);

            // N == 1: status byte only
            uint8_t status_byte = 0;
            TEST_ASSERT_EQUAL(HAL_OK, i2c_master_receive(TEST_PORT, AHT20_I2C_ADDRESS, &status_byte, 1));

            // Trigger again for a clean 2-byte read
            TEST_ASSERT_EQUAL(HAL_OK, i2c_master_transmit(TEST_PORT, AHT20_I2C_ADDRESS, tx_trigger.data(), tx_trigger.size()));
            delay_ms(100);

            // N == 2
            std::array<uint8_t, 2> two_bytes{};
            TEST_ASSERT_EQUAL(HAL_OK, i2c_master_receive(TEST_PORT, AHT20_I2C_ADDRESS, two_bytes.data(), two_bytes.size()));

            // Leave the sensor in a known state for whatever runs next
            delay_ms(15);
        }

        void aht20_deinits_fine() {
            // This test must run last after all other AHT20 tests since it tears up the bus and deinitializes the AHT20
            constexpr uint8_t aht20_reset_code = 0xBA;
            TEST_ASSERT_EQUAL(HAL_OK, i2c_master_transmit(TEST_PORT, AHT20_I2C_ADDRESS, &aht20_reset_code, sizeof(aht20_reset_code)));
            TEST_ASSERT_EQUAL(HAL_OK, i2c_master_deinit(TEST_PORT));
            TEST_ASSERT_EQUAL(HAL_OK, i2cx_clk_enable(TEST_PORT, false));
        }

        void multiple_aht20_reads_work() {
            aht20_inits_fine();
            for (uint8_t i = 0; i < 10; i++) {
                read_attempt_from_aht20();
            }
            aht20_deinits_fine();
        }

        void aht20_init_deinit_stress_test() {
            aht20_inits_fine();
            read_attempt_from_aht20();
            aht20_deinits_fine();
            aht20_inits_fine();
            aht20_deinits_fine();
            aht20_inits_fine();
            read_attempt_from_aht20();
            aht20_deinits_fine();
        }

    } // namespace

    void all() {
        LOGI(TAG, "Starting the tests on the I2C driver");
        UNITY_BEGIN();

        RUN_TEST(invalid_arg_guards);
        RUN_TEST(clk_enable_toggles_the_correct_bus_bit);
        RUN_TEST(deinit_clears_control_registers);
        RUN_TEST(init_selects_fast_mode_only_at_400khz);
        RUN_TEST(init_programs_the_configured_digital_filter);
        RUN_TEST(transmit_and_receive_reject_a_busy_bus);
        RUN_TEST(transmit_to_unaddressed_device_returns_device_not_found);
        RUN_TEST(aht20_inits_fine);
        RUN_TEST(read_attempt_from_aht20);
        RUN_TEST(receive_handles_one_and_two_byte_transfers);
        RUN_TEST(multiple_aht20_reads_work);
        RUN_TEST(aht20_init_deinit_stress_test);
        RUN_TEST(aht20_deinits_fine);

        UNITY_END();
        LOGI(TAG, "Done with the I2C driver tests");
    }

} // namespace test::i2c
