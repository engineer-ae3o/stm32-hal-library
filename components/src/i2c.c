#include "stm32f411xe.h"
#include "i2c.h"


// Forward declarations
static inline void send_start(I2C_TypeDef* handle);
static inline void send_stop(I2C_TypeDef* handle);
static i2c_err_t tx_trans(I2C_TypeDef* handle, uint8_t address, const uint8_t* data, size_t len);
static i2c_err_t rx_trans(I2C_TypeDef* handle, uint8_t address, uint8_t* data, size_t len);

#define TIMEOUT_CYCLES 10'000


// Public API
// I2C Master API
i2c_err_t i2c_master_init(I2C_TypeDef* handle, const i2c_master_config_t* config) {

    // Enable the I2C peripheral clock
    if (handle == I2C1) {
        RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    } else if (handle == I2C2) {
        RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
    } else if (handle == I2C3) {
        RCC->APB1ENR |= RCC_APB1ENR_I2C3EN;
    } else {
        return I2C_INVALID_ARG;
    }

    // Configure pins for I2C
    // Enable gpio channel clock
    if (config->gpio_port == GPIOA) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    } else if (config->gpio_port == GPIOB) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    } else if (config->gpio_port == GPIOC) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    } else if (config->gpio_port == GPIOD) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    } else {
        return I2C_INVALID_ARG;
    }

    // Set pins to alternate mode
    config->gpio_port->MODER |= (0b10UL << (config->sda * 2UL));
    config->gpio_port->MODER |= (0b10UL << (config->scl * 2UL));

    // Set as open drain
    config->gpio_port->OTYPER |= (0b1UL << config->sda);
    config->gpio_port->OTYPER |= (0b1UL << config->scl);

    // Speed mode
    config->gpio_port->OSPEEDR |= (0b11UL << (config->sda * 2UL));
    config->gpio_port->OSPEEDR |= (0b11UL << (config->scl * 2UL));
    
    // Pullups
    if (config->use_pullup) {
        // SDA
        config->gpio_port->PUPDR &= ~(0b11UL << (config->sda * 2UL));
        config->gpio_port->PUPDR |= (0b01UL << (config->sda * 2UL));
        // SCL
        config->gpio_port->PUPDR &= ~(0b11UL << (config->scl * 2UL));
        config->gpio_port->PUPDR |= (0b01UL << (config->scl * 2UL));
    }
    
    // Set the alternate function
    const uint32_t alt_val = 0b100UL;
    // SDA
    if (config->sda <= 7) {
        config->gpio_port->AFR[0] &= ~(0b1111UL << (config->sda * 4UL));
        config->gpio_port->AFR[0] |= (alt_val << (config->sda * 4UL));
    } else if (config->sda <= 15) {
        config->gpio_port->AFR[1] &= ~(0b1111UL << ((config->sda - 8) * 4UL));
        config->gpio_port->AFR[1] |= (alt_val << ((config->sda - 8) * 4UL));
    } else {
        return I2C_INVALID_ARG;
    }
    // SCL
    if (config->scl <= 7) {
        config->gpio_port->AFR[0] &= ~(0b1111UL << (config->scl * 4UL));
        config->gpio_port->AFR[0] |= (alt_val << (config->scl * 4UL));
    } else if (config->scl <= 15) {
        config->gpio_port->AFR[1] &= ~(0b1111UL << ((config->scl - 8) * 4UL));
        config->gpio_port->AFR[1] |= (alt_val << ((config->scl - 8) * 4UL));
    } else {
        return I2C_INVALID_ARG;
    }
    
    // I2C configuration
    handle->CR1 |= I2C_CR1_ACK;
    handle->CR2 |= (config->apb1_bus_freq_mhz << I2C_CR2_FREQ_Pos);

    // Clock configuration
    // TODO: Calcukate CCR properly
    const uint32_t clk_speed_hz = (config->freq_type == I2C_400KHz) ? 400'000U : 100'000U;
    (void)clk_speed_hz;
    const uint32_t ccr = 0;
    handle->CCR |= ((config->freq_type << I2C_CCR_FS_Pos) | (((ccr) & 0xFFFU) << I2C_CCR_CCR_Pos) | I2C_CCR_DUTY);

    // Rise time
    // TODO: Account for digital filter for the TRISE value
    uint32_t trise_ns = (config->freq_type == I2C_400KHz) ? 300U : 1'000U;
    handle->TRISE |= (((trise_ns * config->apb1_bus_freq_mhz) / 1000U) + 1);
    
    // Analog and digital noise filters
    handle->FLTR &= ~I2C_FLTR_ANOFF;
    handle->FLTR |= (5U << I2C_FLTR_DNF_Pos);

    // Enable the I2C peripheral
    handle->CR1 |= I2C_CR1_PE;
    
    return I2C_OK;
}

i2c_err_t i2c_master_transmit(I2C_TypeDef* handle, uint8_t address, const uint8_t* data, size_t len) {

    // Check if the bus is free before proceeding
    if (handle->SR2 & I2C_SR2_BUSY) return I2C_INVALID_STATE;
    
    // Start transaction
    send_start(handle);
    
    // Transmit bytes
    i2c_err_t ret = tx_trans(handle, address, data, len);
    
    // End transaction
    send_stop(handle);

    return ret;
}

i2c_err_t i2c_master_receive(I2C_TypeDef* handle, uint8_t address, uint8_t* data, size_t len) {

    // Check if the bus is free before proceeding
    if (handle->SR2 & I2C_SR2_BUSY) return I2C_INVALID_STATE;
    
    // Start transaction
    send_start(handle);
    
    // No need to call `send_stop()` as `rx_trans()` already does
    return rx_trans(handle, address, data, len);
}

i2c_err_t i2c_master_transmit_receive(I2C_TypeDef* handle, uint8_t address, const uint8_t* tx_data,
                                      size_t tx_len, uint8_t* rx_data, size_t rx_len) {

    // Check if the bus is free before proceeding
    if (handle->SR2 & I2C_SR2_BUSY) return I2C_INVALID_STATE;
    
    // Start transaction
    send_start(handle);
    
    // Start TX transaction
    i2c_err_t ret = tx_trans(handle, address, tx_data, tx_len);
    if (ret != I2C_OK) {
        send_stop(handle);
        return ret;
    }

    // Send a repeated start
    send_start(handle);
    
    // Start RX transaction
    // No need to call `send_stop()` as `rx_trans()` already does
    return rx_trans(handle, address, rx_data, rx_len);
}

// I2C Slave API
i2c_err_t i2c_slave_init(I2C_TypeDef* handle, const i2c_slave_config_t* config) {

    (void)handle;
    (void)config;

    return I2C_OK;
}

i2c_err_t i2c_slave_transmit(I2C_TypeDef* handle, const uint8_t* data, size_t len) {
    
    (void)handle;
    (void)data;
    (void)len;

    return I2C_OK;
}

i2c_err_t i2c_slave_receive(I2C_TypeDef* handle, uint8_t* data, size_t len) {
    
    (void)handle;
    (void)data;
    (void)len;

    return I2C_OK;
}


// Helpers
static inline void send_start(I2C_TypeDef* handle) {
    handle->CR1 |= I2C_CR1_START;
    while (!(handle->SR1 & I2C_SR1_SB));
    // Read the SR1 register as part of the sequence to clear the SB flag
    (void)handle->SR1;
}

static inline void send_stop(I2C_TypeDef* handle) {
    handle->CR1 |= I2C_CR1_STOP;
}

static i2c_err_t tx_trans(I2C_TypeDef* handle, uint8_t address, const uint8_t* data, size_t len) {
    
    // Send address and write bit
    handle->DR = ((address << 1UL) | 0UL);
    
    // Wait for ACK
    uint32_t timeout = TIMEOUT_CYCLES;
    while (!(handle->SR1 & I2C_SR1_ADDR) && (timeout--)) {
        // Check all error flags since the success flag doesn't get set when there's an error
        if (handle->SR1 & I2C_SR1_AF) {
            handle->SR1 &= ~I2C_SR1_AF;
            return I2C_DEVICE_NOT_FOUND;
        }
        if (handle->SR1 & I2C_SR1_BERR) {
            handle->SR1 &= ~I2C_SR1_BERR;
            // Continue with the transfer as spurious bus errors don't corrupt the transaction
            continue;
        }
        if (handle->SR1 & I2C_SR1_ARLO) {
            handle->SR1 &= ~I2C_SR1_ARLO;
            return I2C_ARBITRATION_LOST;
        }
    }
    
    // Return if the ADDR bit still hasn't been set
    if (!(handle->SR1 & I2C_SR1_ADDR) || (timeout == 0)) return I2C_FAIL;
    
    // Read both registers to clear the ADDR bit
    (void)handle->SR1;
    (void)handle->SR2;

    // Start transmission after receiving ACK
    for (size_t i = 0U; i < len; i++) {
        // Wait for TXE
        timeout = TIMEOUT_CYCLES;
        while (!(handle->SR1 & I2C_SR1_TXE) && (timeout--)) {
            // Check all error flags since the success flag doesn't get set when there's an error
            if (handle->SR1 & I2C_SR1_AF) {
                handle->SR1 &= ~I2C_SR1_AF;
                return I2C_TX_ERROR;
            }
            if (handle->SR1 & I2C_SR1_BERR) {
                handle->SR1 &= ~I2C_SR1_BERR;
                continue;
            }
            if (handle->SR1 & I2C_SR1_ARLO) {
                handle->SR1 &= ~I2C_SR1_ARLO;
                return I2C_ARBITRATION_LOST;
            }
        }

        // Return if the TXE bit still has not been set
        if (!(handle->SR1 & I2C_SR1_TXE) || (timeout == 0)) return I2C_TX_ERROR;

        // Transmit byte
        handle->DR = data[i];
    }

    // Wait till the last byte has been fully transmitted on the bus
    timeout = TIMEOUT_CYCLES;
    while (!(handle->SR1 & I2C_SR1_BTF) && (timeout--)) {
        // Check all error flags since the success flag doesn't get set when there's an error
        if (handle->SR1 & I2C_SR1_AF) {
            handle->SR1 &= ~I2C_SR1_AF;
            return I2C_TX_ERROR;
        }
        if (handle->SR1 & I2C_SR1_BERR) {
            handle->SR1 &= ~I2C_SR1_BERR;
            continue;
        }
        if (handle->SR1 & I2C_SR1_ARLO) {
            handle->SR1 &= ~I2C_SR1_ARLO;
            return I2C_TX_ERROR;
        }
    }

    // Return if the BTF bit still has not been set
    if (!(handle->SR1 & I2C_SR1_BTF) || (timeout == 0)) return I2C_TX_ERROR;
    
    return I2C_OK;
}

static i2c_err_t rx_trans(I2C_TypeDef* handle, uint8_t address, uint8_t* data, size_t len) {
    
    // Send address and read bit
    handle->DR = ((address << 1UL) | 1UL);
    
    // Wait for ACK
    uint32_t timeout = TIMEOUT_CYCLES;
    while (!(handle->SR1 & I2C_SR1_ADDR) && (timeout--)) {
        // Check all error flags since the success flag doesn't get set when there's an error
        if (handle->SR1 & I2C_SR1_AF) {
            handle->SR1 &= ~I2C_SR1_AF;
            send_stop(handle);
            return I2C_DEVICE_NOT_FOUND;
        }
        if (handle->SR1 & I2C_SR1_BERR) {
            handle->SR1 &= ~I2C_SR1_BERR;
            // Continue with the transfer as spurious bus errors don't corrupt the transaction
            continue;
        }
        if (handle->SR1 & I2C_SR1_ARLO) {
            handle->SR1 &= ~I2C_SR1_ARLO;
            send_stop(handle);
            return I2C_ARBITRATION_LOST;
        }
    }
    
    // Return if the ADDR bit still hasn't been set
    if (!(handle->SR1 & I2C_SR1_ADDR) || (timeout == 0)) {
        send_stop(handle);
        return I2C_FAIL;
    }

    // Set the ACK bit in the case that the previous transaction didn't set it
    handle->CR1 |= I2C_CR1_ACK;
    
    // Start reception after receiving ACK
    size_t remaining_bytes = len;
    
    // Data phase
    // Handle the cases for the different lengths
    switch (len) {
        // Not a valid case
        case 0:
            send_stop(handle);
            return I2C_INVALID_ARG;

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
            while (!(handle->SR1 & I2C_SR1_RXNE) && (timeout--)) {
                // Check all error flags since the success flag doesn't get set when there's an error
                if (handle->SR1 & I2C_SR1_BERR) {
                    handle->SR1 &= ~I2C_SR1_BERR;
                    continue;
                }
                if (handle->SR1 & I2C_SR1_ARLO) {
                    handle->SR1 &= ~I2C_SR1_ARLO;
                    return I2C_ARBITRATION_LOST;
                }
            }

            // Return if the RXE bit still has not been set
            if (!(handle->SR1 & I2C_SR1_RXNE) || (timeout == 0)) return I2C_RX_ERROR;

            // Finally, read byte
            data[0] = (uint8_t)handle->DR;

            // Set the ACK bit so as not to corrupt other transactions
            handle->CR1 |= I2C_CR1_ACK;
            
            return I2C_OK;

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
            while (!(handle->SR1 & I2C_SR1_BTF) && (timeout--)) {
                if (handle->SR1 & I2C_SR1_BERR) {
                    handle->SR1 &= ~I2C_SR1_BERR;
                    continue;
                }
                if (handle->SR1 & I2C_SR1_ARLO) {
                    handle->SR1 &= ~I2C_SR1_ARLO;
                    send_stop(handle);
                    return I2C_RX_ERROR;
                }
            }

            // Return if the BTF bit still has not been set
            if (!(handle->SR1 & I2C_SR1_BTF) || (timeout == 0)) return I2C_RX_ERROR;

            // Send stop now so the peripheral does this immediately after the transaction
            send_stop(handle);

            // Finally, read DR twice to get both bytes
            data[0] = (uint8_t)handle->DR;
            data[1] = (uint8_t)handle->DR;

            // Clear the POS bit and set the ACK bit so as not to corrupt other transactions
            handle->CR1 |= I2C_CR1_ACK;
            handle->CR1 &= ~I2C_CR1_POS;
            
            return I2C_OK;

        // When N > 2
        default:
            while (1) {
                // Handle the different cases for the remaining number of bytes
                switch (remaining_bytes) {
                    case 3:
                        // Wait till the BTF bit has been set
                        timeout = TIMEOUT_CYCLES;
                        while (!(handle->SR1 & I2C_SR1_BTF) && (timeout--)) {
                            if (handle->SR1 & I2C_SR1_BERR) {
                                handle->SR1 &= ~I2C_SR1_BERR;
                                continue;
                            }
                            if (handle->SR1 & I2C_SR1_ARLO) {
                                handle->SR1 &= ~I2C_SR1_ARLO;
                                send_stop(handle);
                                return I2C_RX_ERROR;
                            }
                        }
                    
                        // Return if the BTF bit still has not been set
                        if (!(handle->SR1 & I2C_SR1_BTF) || (timeout == 0)) return I2C_RX_ERROR;

                        // Clear the ACK bit so the peripheral sends a NACK after all reception has been completed
                        handle->CR1 &= ~I2C_CR1_ACK;

                        // Get the third to the last byte
                        data[len - remaining_bytes] = (uint8_t)handle->DR;
                        remaining_bytes--;

                        break;
                    
                    case 2:
                        // Wait till the BTF bit has been set, again
                        timeout = TIMEOUT_CYCLES;
                        while (!(handle->SR1 & I2C_SR1_BTF) && (timeout--)) {
                            if (handle->SR1 & I2C_SR1_BERR) {
                                handle->SR1 &= ~I2C_SR1_BERR;
                                continue;
                            }
                            if (handle->SR1 & I2C_SR1_ARLO) {
                                handle->SR1 &= ~I2C_SR1_ARLO;
                                send_stop(handle);
                                return I2C_RX_ERROR;
                            }
                        }
                    
                        // Return if the BTF bit still has not been set
                        if (!(handle->SR1 & I2C_SR1_BTF) || (timeout == 0)) return I2C_RX_ERROR;

                        // Send stop now so the peripheral does this immediately after the transaction
                        send_stop(handle);

                        // Finally, read DR twice to get both remaining bytes
                        data[len - remaining_bytes] = (uint8_t)handle->DR;
                        data[len - remaining_bytes + 1] = (uint8_t)handle->DR;

                        return I2C_OK;
                    
                    default:
                        // Read both registers to clear the ADDR bit
                        (void)handle->SR1;
                        (void)handle->SR2;

                        // Read RXE up until remaining_bytes is 3
                        for (size_t i = 0; i < (len - 3); i++) {
                            timeout = TIMEOUT_CYCLES;
                            while (!(handle->SR1 & I2C_SR1_RXNE) && (timeout--)) {
                                if (handle->SR1 & I2C_SR1_BERR) {
                                    handle->SR1 &= ~I2C_SR1_BERR;
                                    continue;
                                }
                                if (handle->SR1 & I2C_SR1_ARLO) {
                                    handle->SR1 &= ~I2C_SR1_ARLO;
                                    send_stop(handle);
                                    return I2C_ARBITRATION_LOST;
                                }
                            }

                            // Return if RXNE still isn't set
                            if (!(handle->SR1 & I2C_SR1_RXNE) || (timeout == 0)) return I2C_RX_ERROR;

                            // Read byte
                            data[i] = (uint8_t)handle->DR;
                            remaining_bytes--;
                        }
                        break;
                }
            }
    }
    
    return I2C_OK;
}
