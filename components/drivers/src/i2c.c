#include "stm32f411xe.h"
#include "drivers/gpio.h"
#include "utils/common.h"
#include "utils/board.h"
#include "utils/clock.h"
#include "drivers/i2c.h"
#include "utils/tick.h"
#include "utils/err.h"


// Forward declarations
[[__gnu__::__always_inline__]] static inline bool send_start(I2C_TypeDef* handle);
[[__gnu__::__always_inline__]] static inline void send_stop(I2C_TypeDef* handle);

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

    // Disable the I2C peripheral before writing to any of its registers and issue a software and hardware bus reset
    handle->CR1 &= ~I2C_CR1_PE;
    TRY(i2c_master_hardware_reset(config->scl_pin.port, config->scl_pin.pin, config->sda_pin.port, config->sda_pin.pin));
    TRY(i2c_master_software_reset(handle));

    // Get the APB1 bus frequency and cache it
    const uint32_t apb1_clk_freq_mhz = get_apb1_core_clock() / 1'000'000U;

    if (config->frequency == I2C_FREQ_100kHz) {
        if (apb1_clk_freq_mhz < MINIIMUM_I2C_100kHz_APB1_CLK_MHz) {
            return HAL_ERR_NOT_SUPPORTED;
        }
    } else if (config->frequency == I2C_FREQ_400kHz) {
        if (apb1_clk_freq_mhz < MINIIMUM_I2C_400kHz_APB1_CLK_MHz) {
            return HAL_ERR_NOT_SUPPORTED;
        }
    } else {
        return HAL_ERR_INVALID_ARG;
    }

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
    // the pins already have their ports enabled and have been set as open drain already. No need to repeat here
    TRY(gpio_set_alternate_function(config->sda_pin.port, config->sda_pin.pin, config->sda_pin.af));
    gpio_set_speed_mode(config->sda_pin.port, config->sda_pin.pin, GPIO_MEDIUM_SPEED);
    gpio_enable_pullups(config->sda_pin.port, config->sda_pin.pin, config->use_pullups);

    TRY(gpio_set_alternate_function(config->scl_pin.port, config->scl_pin.pin, config->scl_pin.af));
    gpio_set_speed_mode(config->scl_pin.port, config->scl_pin.pin, GPIO_MEDIUM_SPEED);
    gpio_enable_pullups(config->scl_pin.port, config->scl_pin.pin, config->use_pullups);

    // Enable the i2C peripheral after all setup
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
    if (!send_start(handle)) {
        return HAL_ERR_I2C_ARBITRATION_LOST;
    }

    // Transmit the data
    hal_err_t ret = tx_trans(handle, address, data, size);

    // End the transaction regardless of an error or success
    send_stop(handle);
    return ret;
}

hal_err_t i2c_master_receive(I2C_TypeDef* handle, uint8_t address, uint8_t* data, size_t size) {
    if (handle == NULL || address == 0 || address > 0x7F || data == NULL || size == 0) {
        return HAL_ERR_INVALID_ARG;
    }

    // Check if the bus is free before proceeding
    if (handle->SR2 & I2C_SR2_BUSY) {
        return HAL_ERR_I2C_BUS_BUSY;
    }

    // Start the transaction
    if (!send_start(handle)) {
        return HAL_ERR_I2C_ARBITRATION_LOST;
    }

    // Start the RX transaction. No need to call send_stop() as rx_trans() already does
    return rx_trans(handle, address, data, size);
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
    if (!send_start(handle)) {
        return HAL_ERR_I2C_ARBITRATION_LOST;
    }

    // Start the transmission
    hal_err_t ret = tx_trans(handle, address, tx_data, tx_size);
    if (ret != HAL_OK) {
        send_stop(handle);
        return ret;
    }

    // Send the repeated start
    if (!send_start(handle)) {
        send_stop(handle);
        return HAL_ERR_I2C_ARBITRATION_LOST;
    }

    // Start the RX transaction. No need to call send_stop() as rx_trans() already does
    return rx_trans(handle, address, rx_data, rx_size);
}


// Helpers
static bool send_start(I2C_TypeDef* handle) {
    // Set start bit
    handle->CR1 |= I2C_CR1_START;

    // Poll the start bit in the SR1 register
    uint32_t timeout_cycles = TIMEOUT_CYCLES;
    while (!(handle->SR1 & I2C_SR1_SB) && (--timeout_cycles)) {
        if (handle->SR1 & I2C_SR1_BERR) {
            handle->SR1 &= ~I2C_SR1_BERR;
            send_stop(handle);
            return false;
        }
        if (handle->SR1 & I2C_SR1_ARLO) {
            handle->SR1 &= ~I2C_SR1_ARLO;
            send_stop(handle);
            return false;
        }
    }
    if (!(handle->SR1 & I2C_SR1_SB) || (timeout_cycles == 0)) {
        send_stop(handle);
        return false;
    }

    // Read the SR1 register as part of the sequence to clear the SB flag in the SR1 register
    (void)handle->SR1;
    return true;
}

static void send_stop(I2C_TypeDef* handle) {
    handle->CR1 |= I2C_CR1_STOP;
}

static hal_err_t tx_trans(I2C_TypeDef* handle, uint8_t address, const uint8_t* data, size_t size) {
    // Send address and write bit
    // cppcheck-suppress badBitmaskCheck
    handle->DR = ((uint32_t)(address << 1UL) | 0UL);

    // Wait for ACK
    uint32_t timeout_cycles = TIMEOUT_CYCLES;
    while (!(handle->SR1 & I2C_SR1_ADDR) && (--timeout_cycles)) {
        // Check all error flags since the success flag doesn't get set when there's an error
        if (handle->SR1 & I2C_SR1_AF) {
            handle->SR1 &= ~I2C_SR1_AF;
            return HAL_ERR_I2C_DEVICE_NOT_FOUND;
        }
        if (handle->SR1 & I2C_SR1_BERR) {
            handle->SR1 &= ~I2C_SR1_BERR;
            continue;
        }
        if (handle->SR1 & I2C_SR1_ARLO) {
            handle->SR1 &= ~I2C_SR1_ARLO;
            return HAL_ERR_I2C_ARBITRATION_LOST;
        }
    }

    // Return if the ADDR bit still hasn't been set
    if (!(handle->SR1 & I2C_SR1_ADDR) || (timeout_cycles == 0)) {
        return HAL_FAIL;
    }

    // Read both registers to clear the ADDR bit
    (void)handle->SR1;
    (void)handle->SR2;

    // Start transmission after receiving ACK
    for (size_t i = 0; i < size; i++) {
        // Wait for TXE
        timeout_cycles = TIMEOUT_CYCLES;
        while (!(handle->SR1 & I2C_SR1_TXE) && (--timeout_cycles)) {
            // Check all error flags since the success flag doesn't get set when there's an error
            if (handle->SR1 & I2C_SR1_AF) {
                handle->SR1 &= ~I2C_SR1_AF;
                return HAL_ERR_TX;
            }
            if (handle->SR1 & I2C_SR1_BERR) {
                handle->SR1 &= ~I2C_SR1_BERR;
                continue;
            }
            if (handle->SR1 & I2C_SR1_ARLO) {
                handle->SR1 &= ~I2C_SR1_ARLO;
                return HAL_ERR_I2C_ARBITRATION_LOST;
            }
        }

        // Return if the TXE bit still has not been set
        if (!(handle->SR1 & I2C_SR1_TXE) || (timeout_cycles == 0)) {
            return HAL_ERR_TX;
        }

        // Write the byte into the I2C data register
        handle->DR = data[i];
    }

    // Wait till the last byte has been fully transmitted on the bus
    timeout_cycles = TIMEOUT_CYCLES;
    while (!(handle->SR1 & I2C_SR1_BTF) && (--timeout_cycles)) {
        // Check all error flags since the success flag doesn't get set when there's an error
        if (handle->SR1 & I2C_SR1_AF) {
            handle->SR1 &= ~I2C_SR1_AF;
            return HAL_ERR_TX;
        }
        if (handle->SR1 & I2C_SR1_BERR) {
            handle->SR1 &= ~I2C_SR1_BERR;
            continue;
        }
        if (handle->SR1 & I2C_SR1_ARLO) {
            handle->SR1 &= ~I2C_SR1_ARLO;
            return HAL_ERR_I2C_ARBITRATION_LOST;
        }
    }

    // Return if the BTF bit still has not been set
    if (!(handle->SR1 & I2C_SR1_BTF) || (timeout_cycles == 0)) {
        return HAL_ERR_TX;
    }

    return HAL_OK;
}

static hal_err_t rx_trans(I2C_TypeDef* handle, uint8_t address, uint8_t* data, size_t size) {
    // Send address and read bit and set the ACK bit before starting
    handle->DR = ((uint32_t)(address << 1UL) | 1UL);
    handle->CR1 |= I2C_CR1_ACK;

    // Wait for ACK
    uint32_t timeout_cycles = TIMEOUT_CYCLES;
    while (!(handle->SR1 & I2C_SR1_ADDR) && (--timeout_cycles)) {
        // Check all error flags since the success flag doesn't get set when there's an error
        if (handle->SR1 & I2C_SR1_AF) {
            handle->SR1 &= ~I2C_SR1_AF;
            send_stop(handle);
            return HAL_ERR_I2C_DEVICE_NOT_FOUND;
        }
        if (handle->SR1 & I2C_SR1_BERR) {
            handle->SR1 &= ~I2C_SR1_BERR;
            continue;
        }
        if (handle->SR1 & I2C_SR1_ARLO) {
            handle->SR1 &= ~I2C_SR1_ARLO;
            send_stop(handle);
            return HAL_ERR_I2C_ARBITRATION_LOST;
        }
    }

    // Return if the ADDR bit still hasn't been set
    if (!(handle->SR1 & I2C_SR1_ADDR) || (timeout_cycles == 0)) {
        send_stop(handle);
        return HAL_FAIL;
    }

    // Set the ACK bit and clear the POS in the case that the previous transaction didn't set it
    handle->CR1 |= I2C_CR1_ACK;
    handle->CR1 &= ~I2C_CR1_POS;

    // Start reception after receiving ACK
    size_t remaining_bytes = size;

    // Data phase
    // Handle the cases for the different lengths
    switch (size) {
        // Not a valid case
        case 0:
            send_stop(handle);
            return HAL_ERR_INVALID_ARG;

        // When N == 1
        case 1:
            // Clear the ACK bit so the peripheral sends a NACK after first byte
            handle->CR1 &= ~I2C_CR1_ACK;

            // Read both registers to clear the ADDR bit
            (void)handle->SR1;
            (void)handle->SR2;

            // Send stop now so the peripheral sends the stop immediately after reception
            send_stop(handle);

            // Wait for RXE
            timeout_cycles = TIMEOUT_CYCLES;
            while (!(handle->SR1 & I2C_SR1_RXNE) && (--timeout_cycles)) {
                // Check all error flags since the success flag doesn't get set when there's an error
                if (handle->SR1 & I2C_SR1_BERR) {
                    handle->SR1 &= ~I2C_SR1_BERR;
                    continue;
                }
                if (handle->SR1 & I2C_SR1_ARLO) {
                    handle->SR1 &= ~I2C_SR1_ARLO;
                    return HAL_ERR_I2C_ARBITRATION_LOST;
                }
            }

            // Return if the RXE bit still has not been set
            if (!(handle->SR1 & I2C_SR1_RXNE) || (timeout_cycles == 0)) {
                return HAL_ERR_RX;
            }

            // Finally, read byte
            data[0] = (uint8_t)handle->DR;

            // Set the ACK bit so as not to corrupt other transactions
            handle->CR1 |= I2C_CR1_ACK;

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

            // Wait till the both bytes have been fully received by the bus
            timeout_cycles = TIMEOUT_CYCLES;
            while (!(handle->SR1 & I2C_SR1_BTF) && (--timeout_cycles)) {
                if (handle->SR1 & I2C_SR1_BERR) {
                    handle->SR1 &= ~I2C_SR1_BERR;
                    continue;
                }
                if (handle->SR1 & I2C_SR1_ARLO) {
                    handle->SR1 &= ~I2C_SR1_ARLO;
                    send_stop(handle);
                    return HAL_ERR_I2C_ARBITRATION_LOST;
                }
            }

            // Return if the BTF bit still has not been set
            if (!(handle->SR1 & I2C_SR1_BTF) || (timeout_cycles == 0)) {
                send_stop(handle);
                return HAL_ERR_RX;
            }

            // Send stop now so the peripheral does this immediately after the transaction
            send_stop(handle);

            // Finally, read DR twice to get both bytes
            data[0] = (uint8_t)handle->DR;
            data[1] = (uint8_t)handle->DR;

            // Clear the POS bit and set the ACK bit so as not to corrupt other transactions
            handle->CR1 |= I2C_CR1_ACK;
            handle->CR1 &= ~I2C_CR1_POS;

            return HAL_OK;

        // When N > 2
        default:
            while (1) {
                // Handle the different cases for the remaining number of bytes
                switch (remaining_bytes) {
                    case 3:
                        // Wait till the BTF bit has been set
                        timeout_cycles = TIMEOUT_CYCLES;
                        while (!(handle->SR1 & I2C_SR1_BTF) && (--timeout_cycles)) {
                            if (handle->SR1 & I2C_SR1_BERR) {
                                handle->SR1 &= ~I2C_SR1_BERR;
                                continue;
                            }
                            if (handle->SR1 & I2C_SR1_ARLO) {
                                handle->SR1 &= ~I2C_SR1_ARLO;
                                send_stop(handle);
                                return HAL_ERR_I2C_ARBITRATION_LOST;
                            }
                        }

                        // Return if the BTF bit still has not been set
                        if (!(handle->SR1 & I2C_SR1_BTF) || (timeout_cycles == 0)) {
                            send_stop(handle);
                            return HAL_ERR_RX;
                        }

                        // Clear the ACK bit so the peripheral sends a NACK after all reception has been completed
                        handle->CR1 &= ~I2C_CR1_ACK;

                        // Get the third to the last byte
                        data[size - remaining_bytes] = (uint8_t)handle->DR;
                        remaining_bytes--;

                        break;

                    case 2:
                        // Wait till the BTF bit has been set, again
                        timeout_cycles = TIMEOUT_CYCLES;
                        while (!(handle->SR1 & I2C_SR1_BTF) && (--timeout_cycles)) {
                            if (handle->SR1 & I2C_SR1_BERR) {
                                handle->SR1 &= ~I2C_SR1_BERR;
                                continue;
                            }
                            if (handle->SR1 & I2C_SR1_ARLO) {
                                handle->SR1 &= ~I2C_SR1_ARLO;
                                send_stop(handle);
                                return HAL_ERR_I2C_ARBITRATION_LOST;
                            }
                        }

                        // Return if the BTF bit still has not been set
                        if (!(handle->SR1 & I2C_SR1_BTF) || (timeout_cycles == 0)) {
                            send_stop(handle);
                            return HAL_ERR_RX;
                        }

                        // Send stop now so the peripheral does this immediately after the transaction
                        send_stop(handle);

                        // Finally, read DR twice to get both remaining bytes
                        data[size - remaining_bytes]     = (uint8_t)handle->DR;
                        data[size - remaining_bytes + 1] = (uint8_t)handle->DR;

                        return HAL_OK;

                    default:
                        // Read both registers to clear the ADDR bit
                        (void)handle->SR1;
                        (void)handle->SR2;

                        // Read RXE up until remaining_bytes is 3
                        for (size_t i = 0; i < (size - 3); i++) {
                            timeout_cycles = TIMEOUT_CYCLES;
                            while (!(handle->SR1 & I2C_SR1_RXNE) && (--timeout_cycles)) {
                                if (handle->SR1 & I2C_SR1_BERR) {
                                    handle->SR1 &= ~I2C_SR1_BERR;
                                    continue;
                                }
                                if (handle->SR1 & I2C_SR1_ARLO) {
                                    handle->SR1 &= ~I2C_SR1_ARLO;
                                    send_stop(handle);
                                    return HAL_ERR_I2C_ARBITRATION_LOST;
                                }
                            }

                            // Return if RXNE still isn't set
                            if (!(handle->SR1 & I2C_SR1_RXNE) || (timeout_cycles == 0)) {
                                send_stop(handle);
                                return HAL_ERR_RX;
                            }

                            // Read byte
                            data[i] = (uint8_t)handle->DR;
                            remaining_bytes--;
                        }
                        break;
                }
            }
    }

    return HAL_OK;
}
