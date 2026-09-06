#include "stm32f411xe.h"
#include "drivers/gpio.h"
#include "utils/common.h"
#include "drivers/i2c.h"
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

    // Configure pins for I2C
    // Enable gpio channel clock
    TRY(gpiox_clk_enable(config->gpio_port, true));

    // Set pins to alternate function for I2C
    // I2C uses an alternate function value of 0b100
    TRY(gpio_set_alternate_function(config->gpio_port, config->sda_pin, 0b100U));
    TRY(gpio_set_alternate_function(config->gpio_port, config->scl_pin, 0b100U));

    // Set as open drain
    gpio_set_output_type(config->gpio_port, config->sda_pin, GPIO_OPEN_DRAIN);
    gpio_set_output_type(config->gpio_port, config->scl_pin, GPIO_OPEN_DRAIN);

    // Speed mode
    gpio_set_speed_mode(config->gpio_port, config->sda_pin, GPIO_MEDIUM_SPEED);
    gpio_set_speed_mode(config->gpio_port, config->scl_pin, GPIO_MEDIUM_SPEED);

    // Pullups
    gpio_enable_pullup(config->gpio_port, config->sda_pin, config->use_pullup);
    gpio_enable_pullup(config->gpio_port, config->scl_pin, config->use_pullup);

    // Disable the I2C peripheral before writing to any of its registers
    handle->CR1 &= ~I2C_CR1_PE;

    // I2C configuration
    handle->CR1 |= I2C_CR1_ACK;
    handle->CR2 &= ~I2C_CR2_FREQ;
    handle->CR2 |= (uint32_t)(config->apb1_bus_freq_mhz << I2C_CR2_FREQ_Pos);

    // Clock configuration
    handle->CCR &= ~(0xFFFUL << I2C_CCR_CCR_Pos);
    if (config->freq_type == I2C_400KHz) {
        // Enable Full mode and duty cycle mode of 16:9
        handle->CCR |= (I2C_CCR_FS | I2C_CCR_DUTY);
        // Calculate the CCR value
        const uint32_t ccr = (config->apb1_bus_freq_mhz * 1'000'000U) / (25U * 400'000UL);
        handle->CCR |= ((ccr & 0xFFFUL) << I2C_CCR_CCR_Pos);

    } else if (config->freq_type == I2C_100KHz) {
        // Enable Standard mode
        handle->CCR &= ~I2C_CCR_FS;
        // Calculate the CCR value
        const uint32_t ccr = (config->apb1_bus_freq_mhz * 1'000'000UL) / (2U * 100'000UL);
        handle->CCR |= ((ccr & 0xFFFUL) << I2C_CCR_CCR_Pos);

    } else {
        return HAL_ERR_INVALID_ARG;
    }

    // Analog and digital noise filters
    handle->FLTR &= ~I2C_FLTR_ANOFF;
    handle->FLTR |= (uint32_t)(config->digital_filter << I2C_FLTR_DNF_Pos);

    // Rise time
    const uint32_t trise_ns = (config->freq_type == I2C_400KHz) ? 300U : 1'000U;
    handle->TRISE           = (((trise_ns * config->apb1_bus_freq_mhz) / 1000U) + config->digital_filter + 1);

    // Enable the I2C peripheral
    handle->CR1 |= I2C_CR1_PE;

    return HAL_OK;
}


// Polling API
hal_err_t i2c_master_transmit(I2C_TypeDef* handle, uint8_t address, const uint8_t* data, size_t size) {
    if (handle == NULL || address == 0 || data == NULL || size == 0) {
        return HAL_ERR_INVALID_ARG;
    }

    // Check if the bus is free before proceeding
    if (handle->SR2 & I2C_SR2_BUSY) {
        return HAL_ERR_INVALID_STATE;
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
    if (handle == NULL || address == 0 || data == NULL || size == 0) {
        return HAL_ERR_INVALID_ARG;
    }

    // Check if the bus is free before proceeding
    if (handle->SR2 & I2C_SR2_BUSY) {
        return HAL_ERR_INVALID_STATE;
    }

    // Start the transaction
    if (!send_start(handle)) {
        return HAL_ERR_I2C_ARBITRATION_LOST;
    }

    // No need to call send_stop() as rx_trans() already does
    return rx_trans(handle, address, data, size);
}

hal_err_t i2c_master_transceive(I2C_TypeDef* handle, uint8_t address, const uint8_t* tx_data, size_t tx_size, uint8_t* rx_data, size_t rx_size) {
    if (handle == NULL || address == 0 || tx_data == NULL || tx_size == 0 || rx_data == NULL || rx_size == 0) {
        return HAL_ERR_INVALID_ARG;
    }

    // Check if the bus is free before proceeding
    if (handle->SR2 & I2C_SR2_BUSY) {
        return HAL_ERR_INVALID_STATE;
    }

    // Start the transaction
    if (!send_start(handle)) {
        return HAL_ERR_I2C_ARBITRATION_LOST;
    }

    // Start the transmission
    TRY_WITH_FUNC(tx_trans(handle, address, tx_data, tx_size), send_stop(handle));

    // Send the repeated start
    if (!send_start(handle)) {
        return HAL_ERR_I2C_ARBITRATION_LOST;
    }

    // Start the RX transaction
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
            // Continue with the operation as spurious bus errors don't corrupt the transaction
            continue;
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
    // Send address and read bit
    handle->DR = ((uint32_t)(address << 1UL) | 1UL);

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
