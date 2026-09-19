#include "stm32f411xe.h"
#include "drivers/gpio.h"
#include "utils/common.h"
#include "utils/board.h"
#include "utils/clock.h"
#include "drivers/i2c.h"
#include "utils/tick.h"
#include "utils/err.h"


// Forward declarations
[[__gnu__::__always_inline__]] static inline hal_err_t send_start(I2C_TypeDef* handle);
[[__gnu__::__always_inline__]] static inline void      send_stop(I2C_TypeDef* handle);
[[__gnu__::__always_inline__]] static inline hal_err_t check_error_flags(I2C_TypeDef* handle, bool send_stop_on_error);

static hal_err_t tx_trans(I2C_TypeDef* handle, uint8_t address, const uint8_t* data, size_t size);
static hal_err_t rx_trans(I2C_TypeDef* handle, uint8_t address, uint8_t* data, size_t size);


// General API
hal_err_t i2cx_clk_enable(I2C_TypeDef* handle, bool enable) {
    if (enable) {
        if (handle == I2C1) {
            RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
        } else if (handle == I2C2) {
            RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
        } else if (handle == I2C3) {
            RCC->APB1ENR |= RCC_APB1ENR_I2C3EN;
        } else {
            return HAL_ERR_INVALID_ARG;
        }

    } else {
        if (handle == I2C1) {
            RCC->APB1ENR &= ~RCC_APB1ENR_I2C1EN;
        } else if (handle == I2C2) {
            RCC->APB1ENR &= ~RCC_APB1ENR_I2C2EN;
        } else if (handle == I2C3) {
            RCC->APB1ENR &= ~RCC_APB1ENR_I2C3EN;
        } else {
            return HAL_ERR_INVALID_ARG;
        }
    }

    __DSB();
    return HAL_OK;
}

hal_err_t i2c_master_init(I2C_TypeDef* handle, const i2c_master_config_t* config) {
    if (handle == NULL || config == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    // Get the APB1 bus frequency and cache it
    const uint32_t apb1_clk_freq_mhz = get_apb1_core_clock() / 1'000'000U;

    if (config->frequency == I2C_FREQ_100kHz) {
        if (apb1_clk_freq_mhz < MINIMUM_I2C_100kHz_APB1_CLK_MHz) {
            return HAL_ERR_NOT_SUPPORTED;
        }
    } else if (config->frequency == I2C_FREQ_400kHz) {
        if ((apb1_clk_freq_mhz < MINIMUM_I2C_400kHz_APB1_CLK_MHz) || (apb1_clk_freq_mhz % 10 != 0)) {
            return HAL_ERR_NOT_SUPPORTED;
        }
    } else {
        return HAL_ERR_INVALID_ARG;
    }

    // Disable the I2C peripheral before writing to any of its registers and issue a software and hardware bus reset
    TRY(i2c_master_hardware_reset(config->scl_pin.port, config->scl_pin.pin, config->sda_pin.port, config->sda_pin.pin));
    TRY(i2c_master_software_reset(handle));
    handle->CR1 &= ~I2C_CR1_PE;

    handle->CR2 &= ~I2C_CR2_FREQ;
    handle->CR2 |= (uint32_t)(apb1_clk_freq_mhz << I2C_CR2_FREQ_Pos) & I2C_CR2_FREQ;

    // Clock configuration
    uint32_t ccr = handle->CCR & ~(I2C_CCR_CCR | I2C_CCR_FS | I2C_CCR_DUTY);

    // Enable Full mode and duty cycle mode of 16:9 if using 400kHz. Leave at standard mode if 100kHz
    ccr |= (config->frequency == I2C_FREQ_400kHz) ? (I2C_CCR_FS | I2C_CCR_DUTY) : 0;
    const uint32_t clock_val = (config->frequency == I2C_FREQ_400kHz) ? (apb1_clk_freq_mhz * 1'000'000U) / (25 * I2C_FREQ_400kHz)
                                                                      : (apb1_clk_freq_mhz * 1'000'000U) / (2 * I2C_FREQ_100kHz);
    ccr |= (clock_val << I2C_CCR_CCR_Pos) & I2C_CCR_CCR;
    handle->CCR = ccr;

    // Analog and digital noise filters
    handle->FLTR &= ~(I2C_FLTR_ANOFF | I2C_FLTR_DNF);
    handle->FLTR |= (uint32_t)(config->digital_filter << I2C_FLTR_DNF_Pos);

    // Rise time
    const uint32_t trise_ns = (config->frequency == I2C_FREQ_400kHz) ? I2C_TRISE_TIME_400kHz_ns : I2C_TRISE_TIME_100kHz_ns;
    handle->TRISE &= ~I2C_TRISE_TRISE;
    handle->TRISE |= (((trise_ns * apb1_clk_freq_mhz) / 1000U) + config->digital_filter + 1) & I2C_TRISE_TRISE;

    // Configure pins for I2C
    // The pins already have their ports enabled and have been set as open drain already. No need to repeat here
    TRY(gpio_set_alternate_function(config->sda_pin.port, config->sda_pin.pin, config->sda_pin.af));
    gpio_set_speed_mode(config->sda_pin.port, config->sda_pin.pin, GPIO_MEDIUM_SPEED);
    gpio_enable_pullups(config->sda_pin.port, config->sda_pin.pin, config->use_pullups);

    TRY(gpio_set_alternate_function(config->scl_pin.port, config->scl_pin.pin, config->scl_pin.af));
    gpio_set_speed_mode(config->scl_pin.port, config->scl_pin.pin, GPIO_MEDIUM_SPEED);
    gpio_enable_pullups(config->scl_pin.port, config->scl_pin.pin, config->use_pullups);

    // Enable the I2C peripheral after all setup
    handle->CR1 |= I2C_CR1_PE;

    return HAL_OK;
}

hal_err_t i2c_master_deinit(I2C_TypeDef* handle) {
    if (handle == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    // Disable before modifiying any other bits
    handle->CR1 &= ~I2C_CR1_PE;

    handle->CCR &= ~(I2C_CCR_FS | I2C_CCR_DUTY | I2C_CCR_CCR);
    handle->CR1 &= ~(I2C_CR1_SMBUS | I2C_CR1_SMBTYPE | I2C_CR1_ENARP | I2C_CR1_ENPEC | I2C_CR1_ENGC | I2C_CR1_NOSTRETCH | I2C_CR1_START |
                     I2C_CR1_STOP | I2C_CR1_ACK | I2C_CR1_POS | I2C_CR1_PEC | I2C_CR1_ALERT | I2C_CR1_SWRST);
    handle->CR2 &= ~(I2C_CR2_FREQ | I2C_CR2_ITERREN | I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN | I2C_CR2_DMAEN | I2C_CR2_LAST);
    handle->FLTR &= ~(I2C_FLTR_DNF | I2C_FLTR_ANOFF);
    handle->TRISE &= ~I2C_TRISE_TRISE;

    return HAL_OK;
}


// Bus recovery mechanisms
hal_err_t i2c_master_software_reset(I2C_TypeDef* handle) {
    if (handle == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    // Assert the software reset
    handle->CR1 |= I2C_CR1_SWRST;
    __DSB();

    // Hold the reset for a brief moment
    delay_us(5);

    // Deassert the software reset
    handle->CR1 &= ~I2C_CR1_SWRST;
    __DSB();

    return HAL_OK;
}

hal_err_t i2c_master_hardware_reset(GPIO_TypeDef* scl_port, gpio_pin_t scl_pin, GPIO_TypeDef* sda_port, gpio_pin_t sda_pin) {
    if (scl_port == NULL || sda_port == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    // Set the SCL and SDA as GPIO open drain output with pullups
    TRY(gpiox_clk_enable(scl_port, true));
    gpio_set_output(scl_port, scl_pin);
    gpio_set_output_type(scl_port, scl_pin, GPIO_OPEN_DRAIN);
    gpio_enable_pullups(scl_port, scl_pin, true);

    TRY(gpiox_clk_enable(sda_port, true));
    gpio_set_output(sda_port, sda_pin);
    gpio_set_output_type(sda_port, sda_pin, GPIO_OPEN_DRAIN);
    gpio_enable_pullups(sda_port, sda_pin, true);

    // Release the SDA line ourselves so it doesn't interfer with the recovery loop
    gpio_set_level(sda_port, sda_pin, true);
    delay_us(5);

    // Pulse the SCL up to 9 times to clock out a stuck target data
    for (uint8_t i = 0; i < 9; i++) {
        gpio_set_level(scl_port, scl_pin, false);
        delay_us(5);

        gpio_set_level(scl_port, scl_pin, true);
        delay_us(5);

        // Check if the SDA has been released
        if (gpio_get_level(sda_port, sda_pin)) {
            break; // Target has released the SDA line
        }
    }

    // Generate the STOP condition manually: SDA low -> SCL high -> SDA high
    gpio_set_level(sda_port, sda_pin, false);
    delay_us(5);
    gpio_set_level(scl_port, scl_pin, true);
    delay_us(5);
    gpio_set_level(sda_port, sda_pin, true);
    delay_us(5);

    return HAL_OK;
}


// Polling API
hal_err_t i2c_master_transmit(I2C_TypeDef* handle, uint8_t address, const uint8_t* data, size_t size) {
    if (handle == NULL || address == 0 || address > 0x7F || data == NULL || size == 0) {
        return HAL_ERR_INVALID_ARG;
    }

    // Check if the bus is free before proceeding
    if (handle->SR2 & I2C_SR2_BUSY) {
        return HAL_ERR_I2C_BUS_BUSY;
    }

    // Start the transaction
    TRY(send_start(handle));
    TRY_WITH_FUNC(tx_trans(handle, address, data, size), send_stop(handle));
    send_stop(handle);

    return HAL_OK;
}

hal_err_t i2c_master_receive(I2C_TypeDef* handle, uint8_t address, uint8_t* data, size_t size) {
    if (handle == NULL || address == 0 || address > 0x7F || data == NULL || size == 0) {
        return HAL_ERR_INVALID_ARG;
    }

    // Check if the bus is free before proceeding
    if (handle->SR2 & I2C_SR2_BUSY) {
        return HAL_ERR_I2C_BUS_BUSY;
    }

    // Start the transaction. rx_trans(...) already sends the stop condition so no need to repeat
    TRY(send_start(handle));
    TRY(rx_trans(handle, address, data, size));

    return HAL_OK;
}

hal_err_t i2c_master_transceive(I2C_TypeDef* handle, uint8_t address, const uint8_t* tx_data, size_t tx_size, uint8_t* rx_data, size_t rx_size) {
    if (handle == NULL || address == 0 || address > 0x7F || tx_data == NULL || tx_size == 0 || rx_data == NULL || rx_size == 0) {
        return HAL_ERR_INVALID_ARG;
    }

    // Check if the bus is free before proceeding
    if (handle->SR2 & I2C_SR2_BUSY) {
        return HAL_ERR_I2C_BUS_BUSY;
    }

    // Start the transaction
    TRY(send_start(handle));
    TRY_WITH_FUNC(tx_trans(handle, address, tx_data, tx_size), send_stop(handle));
    TRY_WITH_FUNC(send_start(handle), send_stop(handle));
    TRY(rx_trans(handle, address, rx_data, rx_size));

    return HAL_OK;
}


// Helpers
static hal_err_t send_start(I2C_TypeDef* handle) {
    // Send the start condition
    handle->CR1 |= I2C_CR1_START;

    // Poll the start bit in the SR1 register till its 1
    uint32_t timeout = TIMEOUT;
    while (!(handle->SR1 & I2C_SR1_SB) && --timeout) {
        TRY(check_error_flags(handle, true));
    }

    if (!(handle->SR1 & I2C_SR1_SB) || (timeout == 0)) {
        send_stop(handle);
        return HAL_ERR_TIMEOUT;
    }

    // Read the SR1 register as part of the sequence to clear the SB flag in the SR1 register
    (void)handle->SR1;
    return HAL_OK;
}

static void send_stop(I2C_TypeDef* handle) {
    handle->CR1 |= I2C_CR1_STOP;
    __DSB();
}

static hal_err_t check_error_flags(I2C_TypeDef* handle, bool send_stop_on_error) {
    hal_err_t error = HAL_OK;

    const uint32_t status = handle->SR1;
    uint32_t       clear  = status;

    if (status & I2C_SR1_AF) {
        clear &= ~I2C_SR1_AF;
        error = HAL_ERR_I2C_DEVICE_NOT_FOUND;
    }
    if (status & I2C_SR1_BERR) {
        clear &= ~I2C_SR1_BERR;
        error = HAL_ERR_I2C_BUS_ERROR;
    }
    if (status & I2C_SR1_ARLO) {
        clear &= ~I2C_SR1_ARLO;
        error = HAL_ERR_I2C_ARBITRATION_LOST;
    }
    handle->SR1 = clear;

    if (send_stop_on_error && error != HAL_OK) {
        send_stop(handle);
    }
    return error;
}

static hal_err_t tx_trans(I2C_TypeDef* handle, uint8_t address, const uint8_t* data, size_t size) {
    // Send address and write bit
    // cppcheck-suppress badBitmaskCheck
    handle->DR = ((uint32_t)(address << 1UL) | 0UL);

    // Wait for ACK
    uint32_t timeout = TIMEOUT;
    while (!(handle->SR1 & I2C_SR1_ADDR) && --timeout) {
        TRY(check_error_flags(handle, false));
    }

    if (!(handle->SR1 & I2C_SR1_ADDR) || (timeout == 0)) {
        return HAL_ERR_TX;
    }

    // Read both registers to clear the ADDR bit
    (void)handle->SR1;
    (void)handle->SR2;

    // Start the transmission after receiving ACK
    for (size_t i = 0; i < size; i++) {
        // Wait for the data register to be empty before writing the data
        timeout = TIMEOUT;
        while (!(handle->SR1 & I2C_SR1_TXE) && --timeout) {
            TRY(check_error_flags(handle, false));
        }
        if (!(handle->SR1 & I2C_SR1_TXE) || (timeout == 0)) {
            return HAL_ERR_TX;
        }
        handle->DR = data[i];
    }

    // Wait till the last byte has been fully transmitted on the bus
    timeout = TIMEOUT;
    while (!(handle->SR1 & I2C_SR1_BTF) && --timeout) {
        TRY(check_error_flags(handle, false));
    }

    if (!(handle->SR1 & I2C_SR1_BTF) || (timeout == 0)) {
        return HAL_ERR_TX;
    }

    return HAL_OK;
}

static hal_err_t rx_trans(I2C_TypeDef* handle, uint8_t address, uint8_t* data, size_t size) {
    // Send address and read bit and set the ACK bit before starting
    handle->DR = ((uint32_t)(address << 1UL) | 1UL);

    // Wait for ACK
    uint32_t timeout = TIMEOUT;
    while (!(handle->SR1 & I2C_SR1_ADDR) && --timeout) {
        TRY(check_error_flags(handle, true));
    }

    // Return if the ADDR bit still hasn't been set
    if (!(handle->SR1 & I2C_SR1_ADDR) || (timeout == 0)) {
        send_stop(handle);
        return HAL_ERR_RX;
    }

    // Set the ACK bit and clear the POS bit
    handle->CR1 = (handle->CR1 & ~I2C_CR1_POS) | I2C_CR1_ACK;

    // Start the reception after receiving ACK
    // The data phase. Handle the cases for the different initial lengths
    switch (size) {
        // When N == 1
        case 1:
            // Clear the ACK bit so the peripheral sends a NACK after the first byte
            handle->CR1 &= ~I2C_CR1_ACK;

            // Read both registers to clear the ADDR bit
            (void)handle->SR1;
            (void)handle->SR2;

            // Send stop now so the peripheral sends the stop immediately after reception
            send_stop(handle);

            // Wait for the data register to contain the received data
            timeout = TIMEOUT;
            while (!(handle->SR1 & I2C_SR1_RXNE) && --timeout) {
                TRY(check_error_flags(handle, false));
            }

            // Return if the RXE bit still has not been set
            if (!(handle->SR1 & I2C_SR1_RXNE) || (timeout == 0)) {
                return HAL_ERR_RX;
            }

            // Finally, read the data
            data[0] = (uint8_t)handle->DR;
            return HAL_OK;

        // When N == 2
        case 2:
            // Set POS bit to apply NACK to the next byte
            handle->CR1 |= I2C_CR1_POS;

            // Clear the ACK bit so the peripheral sends a NACK after all reception has been completed
            handle->CR1 &= ~I2C_CR1_ACK;

            // Read both registers to clear the ADDR bit
            (void)handle->SR1;
            (void)handle->SR2;

            // Wait till both bytes have been fully received by the bus
            timeout = TIMEOUT;
            while (!(handle->SR1 & I2C_SR1_BTF) && --timeout) {
                TRY(check_error_flags(handle, true));
            }

            if (!(handle->SR1 & I2C_SR1_BTF) || (timeout == 0)) {
                send_stop(handle);
                return HAL_ERR_RX;
            }

            // Send stop now so the peripheral does this immediately after the transaction
            send_stop(handle);

            // Finally, read the DR twice to get the two bytes received
            data[0] = (uint8_t)handle->DR;
            data[1] = (uint8_t)handle->DR;
            return HAL_OK;

        // When N > 2
        default: {
            size_t remaining_bytes = size;
            if (remaining_bytes > 3) {
                // Read both registers to clear the ADDR bit
                (void)handle->SR1;
                (void)handle->SR2;

                // Read RXE up until remaining_bytes is 3
                for (size_t i = 0; i < (size - 3); i++) {
                    timeout = TIMEOUT;
                    while (!(handle->SR1 & I2C_SR1_RXNE) && --timeout) {
                        TRY(check_error_flags(handle, true));
                    }

                    // Return if RXNE still isn't set
                    if (!(handle->SR1 & I2C_SR1_RXNE) || (timeout == 0)) {
                        send_stop(handle);
                        return HAL_ERR_RX;
                    }

                    // Get the next data item
                    data[i] = (uint8_t)handle->DR;
                    remaining_bytes--;
                }
            }

            // If all went well, remaining_bytes should be 3
            ASSERT(remaining_bytes == 3);

            // Wait till the BTF bit has been set
            timeout = TIMEOUT;
            while (!(handle->SR1 & I2C_SR1_BTF) && --timeout) {
                TRY(check_error_flags(handle, true));
            }

            // Return if the BTF bit still has not been set
            if (!(handle->SR1 & I2C_SR1_BTF) || (timeout == 0)) {
                send_stop(handle);
                return HAL_ERR_RX;
            }

            // Clear the ACK bit so the peripheral sends a NACK after all reception has been completed
            handle->CR1 &= ~I2C_CR1_ACK;

            // Get the third to the last byte
            data[size - remaining_bytes] = (uint8_t)handle->DR;
            remaining_bytes--;

            // Wait till the BTF bit has been set, again
            timeout = TIMEOUT;
            while (!(handle->SR1 & I2C_SR1_BTF) && --timeout) {
                TRY(check_error_flags(handle, true));
            }

            // Return if the BTF bit still has not been set
            if (!(handle->SR1 & I2C_SR1_BTF) || (timeout == 0)) {
                send_stop(handle);
                return HAL_ERR_RX;
            }

            // Send stop now so the peripheral does this immediately after the transaction ends
            send_stop(handle);

            // Finally, read DR twice to get both remaining bytes
            data[size - remaining_bytes]     = (uint8_t)handle->DR;
            data[size - remaining_bytes + 1] = (uint8_t)handle->DR;
            return HAL_OK;
        }
    }
}
