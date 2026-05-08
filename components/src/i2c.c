#include "stm32f411xe.h"
#include "gpio.h"
#include "i2c.h"


// Forward declarations
static inline bool send_start(I2C_TypeDef* handle);
static inline void send_stop(I2C_TypeDef* handle);
static hal_err_t tx_trans(I2C_TypeDef* handle, uint8_t address, const uint8_t* data, size_t len);
static hal_err_t rx_trans(I2C_TypeDef* handle, uint8_t address, uint8_t* data, size_t len);


// Public API
hal_err_t i2cx_clk_enable(I2C_TypeDef* handle, bool enable) {
    
    if (enable) {
        if (handle == I2C1) {
            RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
        } else if (handle == I2C2) {
            RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
        } else if (handle == I2C3) {
            RCC->APB1ENR |= RCC_APB1ENR_I2C3EN;
        } else {
            return HAL_INVALID_ARG;
        }
        goto done;
    }
    
    if (handle == I2C1) {
        RCC->APB1ENR &= ~RCC_APB1ENR_I2C1EN;
    } else if (handle == I2C2) {
        RCC->APB1ENR &= ~RCC_APB1ENR_I2C2EN;
    } else if (handle == I2C3) {
        RCC->APB1ENR &= ~RCC_APB1ENR_I2C3EN;
    } else {
        return HAL_INVALID_ARG;
    }
    
done:
    __DSB();
    return HAL_OK;
}

hal_err_t i2c_master_init(I2C_TypeDef* handle, const i2c_master_config_t* config) {
    // Configure pins for I2C
    // Enable gpio channel clock
    hal_err_t ret = gpiox_clk_enable(config->gpio_port, true);
    if (ret != HAL_OK) return ret;

    // Set pins to alternate function for I2C
    // I2C uses an alternate function value of 0b100
    ret = gpio_set_alternate_function(config->gpio_port, config->sda, 0b100U);
    if (ret != HAL_OK) return ret;
    ret = gpio_set_alternate_function(config->gpio_port, config->scl, 0b100U);
    if (ret != HAL_OK) return ret;

    // Set as open drain
    gpio_set_output_type(config->gpio_port, config->sda, GPIO_OPEN_DRAIN);
    gpio_set_output_type(config->gpio_port, config->scl, GPIO_OPEN_DRAIN);

    // Speed mode
    gpio_set_speed_mode(config->gpio_port, config->sda, GPIO_MEDIUM_SPEED);
    gpio_set_speed_mode(config->gpio_port, config->scl, GPIO_MEDIUM_SPEED);
    
    // Pullups
    gpio_enable_pullup(config->gpio_port, config->sda, config->use_pullup);
    gpio_enable_pullup(config->gpio_port, config->scl, config->use_pullup);
    
    // Disable the I2C peripheral
    handle->CR1 &= ~I2C_CR1_PE;

    // I2C configuration
    handle->CR1 |= I2C_CR1_ACK;
    handle->CR2 |= (config->apb1_bus_freq_mhz << I2C_CR2_FREQ_Pos);

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
        return HAL_INVALID_ARG;
    }
    
    // Digital filter value
    const uint32_t digi_filt = 5;
    
    // Analog and digital noise filters
    handle->FLTR &= ~I2C_FLTR_ANOFF;
    handle->FLTR |= (digi_filt << I2C_FLTR_DNF_Pos);

    // Rise time
    uint32_t trise_ns = (config->freq_type == I2C_400KHz) ? 300U : 1'000U;
    handle->TRISE = (((trise_ns * config->apb1_bus_freq_mhz) / 1000U) + digi_filt + 1);

    // Enable the I2C peripheral
    handle->CR1 |= I2C_CR1_PE;
    
    return HAL_OK;
}

hal_err_t i2c_master_transmit(I2C_TypeDef* handle, uint8_t address, const uint8_t* data, size_t len) {

    // Check if the bus is free before proceeding
    if (handle->SR2 & I2C_SR2_BUSY) return HAL_INVALID_STATE;
    
    // Start transaction
    if (!send_start(handle)) return HAL_I2C_ARBITRATION_LOST;
    
    // Transmit bytes
    hal_err_t ret = tx_trans(handle, address, data, len);
    
    // End transaction
    send_stop(handle);

    return ret;
}

hal_err_t i2c_master_receive(I2C_TypeDef* handle, uint8_t address, uint8_t* data, size_t len) {

    // Check if the bus is free before proceeding
    if (handle->SR2 & I2C_SR2_BUSY) return HAL_INVALID_STATE;
    
    // Start transaction
    if (!send_start(handle)) return HAL_I2C_ARBITRATION_LOST;
    
    // No need to call `send_stop()` as `rx_trans()` already does
    return rx_trans(handle, address, data, len);
}

hal_err_t i2c_master_transceive(I2C_TypeDef* handle, uint8_t address, const uint8_t* tx_data,
                                      size_t tx_len, uint8_t* rx_data, size_t rx_len) {

    // Check if the bus is free before proceeding
    if (handle->SR2 & I2C_SR2_BUSY) return HAL_INVALID_STATE;
    
    // Start transaction
    if (!send_start(handle)) return HAL_I2C_ARBITRATION_LOST;
    
    // Start TX transaction
    hal_err_t ret = tx_trans(handle, address, tx_data, tx_len);
    if (ret != HAL_OK) {
        send_stop(handle);
        return ret;
    }

    // Send a repeated start
    if (!send_start(handle)) return HAL_I2C_ARBITRATION_LOST;
    
    // Start RX transaction
    // No need to call `send_stop()` as `rx_trans()` already does
    return rx_trans(handle, address, rx_data, rx_len);
}


// Helpers
static inline bool send_start(I2C_TypeDef* handle) {
    // Set start bit
    handle->CR1 |= I2C_CR1_START;

    // Poll the start bit in the SR1 register
    uint32_t timeout = TIMEOUT_CYCLES;
    while (!(handle->SR1 & I2C_SR1_SB) && (--timeout)) {
        if (handle->SR1 & I2C_SR1_BERR) {
            handle->SR1 &= ~I2C_SR1_BERR;
            // Continue with the transfer as spurious bus errors don't corrupt the transaction
            continue;
        }
        if (handle->SR1 & I2C_SR1_ARLO) {
            handle->SR1 &= ~I2C_SR1_ARLO;
            send_stop(handle);
            return false;
        }
    }
    if (!(handle->SR1 & I2C_SR1_SB) || (timeout == 0)) return false;

    // Read the SR1 register as part of the sequence to clear the SB flag in the SR1 register
    (void)handle->SR1;
    return true;
}

static inline void send_stop(I2C_TypeDef* handle) {
    handle->CR1 |= I2C_CR1_STOP;
}

static hal_err_t tx_trans(I2C_TypeDef* handle, uint8_t address, const uint8_t* data, size_t len) {
    
    // Send address and write bit
    handle->DR = ((address << 1UL) | 0UL);
    
    // Wait for ACK
    uint32_t timeout = TIMEOUT_CYCLES;
    while (!(handle->SR1 & I2C_SR1_ADDR) && (--timeout)) {
        // Check all error flags since the success flag doesn't get set when there's an error
        if (handle->SR1 & I2C_SR1_AF) {
            handle->SR1 &= ~I2C_SR1_AF;
            return HAL_I2C_DEVICE_NOT_FOUND;
        }
        if (handle->SR1 & I2C_SR1_BERR) {
            handle->SR1 &= ~I2C_SR1_BERR;
            continue;
        }
        if (handle->SR1 & I2C_SR1_ARLO) {
            handle->SR1 &= ~I2C_SR1_ARLO;
            return HAL_I2C_ARBITRATION_LOST;
        }
    }
    
    // Return if the ADDR bit still hasn't been set
    if (!(handle->SR1 & I2C_SR1_ADDR) || (timeout == 0)) return HAL_FAIL;
    
    // Read both registers to clear the ADDR bit
    (void)handle->SR1;
    (void)handle->SR2;

    // Start transmission after receiving ACK
    for (size_t i = 0U; i < len; i++) {
        // Wait for TXE
        timeout = TIMEOUT_CYCLES;
        while (!(handle->SR1 & I2C_SR1_TXE) && (--timeout)) {
            // Check all error flags since the success flag doesn't get set when there's an error
            if (handle->SR1 & I2C_SR1_AF) {
                handle->SR1 &= ~I2C_SR1_AF;
                return HAL_TX_ERROR;
            }
            if (handle->SR1 & I2C_SR1_BERR) {
                handle->SR1 &= ~I2C_SR1_BERR;
                continue;
            }
            if (handle->SR1 & I2C_SR1_ARLO) {
                handle->SR1 &= ~I2C_SR1_ARLO;
                return HAL_I2C_ARBITRATION_LOST;
            }
        }

        // Return if the TXE bit still has not been set
        if (!(handle->SR1 & I2C_SR1_TXE) || (timeout == 0)) return HAL_TX_ERROR;

        // Transmit byte
        handle->DR = data[i];
    }

    // Wait till the last byte has been fully transmitted on the bus
    timeout = TIMEOUT_CYCLES;
    while (!(handle->SR1 & I2C_SR1_BTF) && (--timeout)) {
        // Check all error flags since the success flag doesn't get set when there's an error
        if (handle->SR1 & I2C_SR1_AF) {
            handle->SR1 &= ~I2C_SR1_AF;
            return HAL_TX_ERROR;
        }
        if (handle->SR1 & I2C_SR1_BERR) {
            handle->SR1 &= ~I2C_SR1_BERR;
            continue;
        }
        if (handle->SR1 & I2C_SR1_ARLO) {
            handle->SR1 &= ~I2C_SR1_ARLO;
            return HAL_I2C_ARBITRATION_LOST;
        }
    }

    // Return if the BTF bit still has not been set
    if (!(handle->SR1 & I2C_SR1_BTF) || (timeout == 0)) return HAL_TX_ERROR;
    
    return HAL_OK;
}

static hal_err_t rx_trans(I2C_TypeDef* handle, uint8_t address, uint8_t* data, size_t len) {
    
    // Send address and read bit
    handle->DR = ((address << 1UL) | 1UL);
    
    // Wait for ACK
    uint32_t timeout = TIMEOUT_CYCLES;
    while (!(handle->SR1 & I2C_SR1_ADDR) && (--timeout)) {
        // Check all error flags since the success flag doesn't get set when there's an error
        if (handle->SR1 & I2C_SR1_AF) {
            handle->SR1 &= ~I2C_SR1_AF;
            send_stop(handle);
            return HAL_I2C_DEVICE_NOT_FOUND;
        }
        if (handle->SR1 & I2C_SR1_BERR) {
            handle->SR1 &= ~I2C_SR1_BERR;
            continue;
        }
        if (handle->SR1 & I2C_SR1_ARLO) {
            handle->SR1 &= ~I2C_SR1_ARLO;
            send_stop(handle);
            return HAL_I2C_ARBITRATION_LOST;
        }
    }
    
    // Return if the ADDR bit still hasn't been set
    if (!(handle->SR1 & I2C_SR1_ADDR) || (timeout == 0)) {
        send_stop(handle);
        return HAL_FAIL;
    }

    // Set the ACK bit and clear the POS in the case that the previous transaction didn't set it
    handle->CR1 |= I2C_CR1_ACK;
    handle->CR1 &= ~I2C_CR1_POS;
    
    // Start reception after receiving ACK
    size_t remaining_bytes = len;
    
    // Data phase
    // Handle the cases for the different lengths
    switch (len) {
        // Not a valid case
        case 0:
            send_stop(handle);
            return HAL_INVALID_ARG;

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
            timeout = TIMEOUT_CYCLES;
            while (!(handle->SR1 & I2C_SR1_RXNE) && (--timeout)) {
                // Check all error flags since the success flag doesn't get set when there's an error
                if (handle->SR1 & I2C_SR1_BERR) {
                    handle->SR1 &= ~I2C_SR1_BERR;
                    continue;
                }
                if (handle->SR1 & I2C_SR1_ARLO) {
                    handle->SR1 &= ~I2C_SR1_ARLO;
                    return HAL_I2C_ARBITRATION_LOST;
                }
            }

            // Return if the RXE bit still has not been set
            if (!(handle->SR1 & I2C_SR1_RXNE) || (timeout == 0)) return HAL_RX_ERROR;

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
            timeout = TIMEOUT_CYCLES;
            while (!(handle->SR1 & I2C_SR1_BTF) && (--timeout)) {
                if (handle->SR1 & I2C_SR1_BERR) {
                    handle->SR1 &= ~I2C_SR1_BERR;
                    continue;
                }
                if (handle->SR1 & I2C_SR1_ARLO) {
                    handle->SR1 &= ~I2C_SR1_ARLO;
                    send_stop(handle);
                    return HAL_I2C_ARBITRATION_LOST;
                }
            }

            // Return if the BTF bit still has not been set
            if (!(handle->SR1 & I2C_SR1_BTF) || (timeout == 0)) {
                send_stop(handle);
                return HAL_RX_ERROR;
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
                        timeout = TIMEOUT_CYCLES;
                        while (!(handle->SR1 & I2C_SR1_BTF) && (--timeout)) {
                            if (handle->SR1 & I2C_SR1_BERR) {
                                handle->SR1 &= ~I2C_SR1_BERR;
                                continue;
                            }
                            if (handle->SR1 & I2C_SR1_ARLO) {
                                handle->SR1 &= ~I2C_SR1_ARLO;
                                send_stop(handle);
                                return HAL_I2C_ARBITRATION_LOST;
                            }
                        }

                        // Return if the BTF bit still has not been set
                        if (!(handle->SR1 & I2C_SR1_BTF) || (timeout == 0)) {
                            send_stop(handle);
                            return HAL_RX_ERROR;
                        }

                        // Clear the ACK bit so the peripheral sends a NACK after all reception has been completed
                        handle->CR1 &= ~I2C_CR1_ACK;

                        // Get the third to the last byte
                        data[len - remaining_bytes] = (uint8_t)handle->DR;
                        remaining_bytes--;

                        break;
                    
                    case 2:
                        // Wait till the BTF bit has been set, again
                        timeout = TIMEOUT_CYCLES;
                        while (!(handle->SR1 & I2C_SR1_BTF) && (--timeout)) {
                            if (handle->SR1 & I2C_SR1_BERR) {
                                handle->SR1 &= ~I2C_SR1_BERR;
                                continue;
                            }
                            if (handle->SR1 & I2C_SR1_ARLO) {
                                handle->SR1 &= ~I2C_SR1_ARLO;
                                send_stop(handle);
                                return HAL_I2C_ARBITRATION_LOST;
                            }
                        }
                    
                        // Return if the BTF bit still has not been set
                        if (!(handle->SR1 & I2C_SR1_BTF) || (timeout == 0)) {
                            send_stop(handle);
                            return HAL_RX_ERROR;
                        }

                        // Send stop now so the peripheral does this immediately after the transaction
                        send_stop(handle);

                        // Finally, read DR twice to get both remaining bytes
                        data[len - remaining_bytes] = (uint8_t)handle->DR;
                        data[len - remaining_bytes + 1] = (uint8_t)handle->DR;

                        return HAL_OK;
                    
                    default:
                        // Read both registers to clear the ADDR bit
                        (void)handle->SR1;
                        (void)handle->SR2;

                        // Read RXE up until remaining_bytes is 3
                        for (size_t i = 0; i < (len - 3); i++) {
                            timeout = TIMEOUT_CYCLES;
                            while (!(handle->SR1 & I2C_SR1_RXNE) && (--timeout)) {
                                if (handle->SR1 & I2C_SR1_BERR) {
                                    handle->SR1 &= ~I2C_SR1_BERR;
                                    continue;
                                }
                                if (handle->SR1 & I2C_SR1_ARLO) {
                                    handle->SR1 &= ~I2C_SR1_ARLO;
                                    send_stop(handle);
                                    return HAL_I2C_ARBITRATION_LOST;
                                }
                            }

                            // Return if RXNE still isn't set
                            if (!(handle->SR1 & I2C_SR1_RXNE) || (timeout == 0)) {
                                send_stop(handle);
                                return HAL_RX_ERROR;
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
