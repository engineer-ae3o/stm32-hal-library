#include "stm32f411xe.h"
#include "i2c.h"


// Helpers
static inline void send_start(I2C_TypeDef* handle) {
    handle->CR1 |= I2C_CR1_START;
    while (!(handle->SR1 & I2C_SR1_SB));
}

static inline void send_stop(I2C_TypeDef* handle) {
    handle->CR1 |= I2C_CR1_STOP;
}

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

    // Pullups
    if (config->pullups_enable) {
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
    handle->CR1 |= (I2C_CR1_ACK | I2C_CR1_POS);
    handle->CR2 |= (I2C_CR2_ITERREN | I2C_CR2_ITEVTEN | (config->apb1_bus_freq_mhz << I2C_CR2_FREQ_Pos));

    // Clock configuration
    const uint32_t clk_speed_hz = (config->freq_type == I2C_400KHz) ? 400'000U : 100'000U;
    const uint32_t ccr = 0;
    handle->CCR |= ((config->freq_type << I2C_CCR_FS_Pos) | (((ccr) & 0xFFFU) << I2C_CCR_CCR_Pos) | I2C_CCR_DUTY);

    // Rise time
    uint32_t trise_ns = (config->freq_type == I2C_400KHz) ? 300U : 1'000U;
    handle->TRISE |= (((trise_ns * config->apb1_bus_freq_mhz) / 1000U) + 1);

    // 7 bit addresses only
    handle->OAR1 &= ~I2C_OAR1_ADDMODE;
    handle->OAR1 |= (1U << 14U);

    // Analog and digital noise filters
    handle->FLTR &= ~I2C_FLTR_ANOFF;
    handle->FLTR |= I2C_FLTR_DNF;

    // Enable the I2C peripheral
    handle->CR1 |= I2C_CR1_PE;
    
    return I2C_OK;
}

i2c_err_t i2c_master_transmit(I2C_TypeDef* handle, uint8_t address, const uint8_t* data, size_t len) {
    
    // Set address
    handle->OAR1 |= ((address & 0x7FU) << 1U);

    // Start transaction
    send_start(handle);
    
    // Send address and write bit
    handle->DR = ((address << 1UL) | 1U);

    // TODO: Wait for ACK or NACK
    
    for (size_t i = 0U; i < len; i++) {
        // Transmit byte
        handle->DR = data[i];

        // TODO: Handle ACK and NACK responses
    }

    // End transaction
    send_stop(handle);

    return I2C_OK;
}

i2c_err_t i2c_master_receive(I2C_TypeDef* handle, uint8_t address, uint8_t* data, size_t len) {
    
    // Set address
    handle->OAR1 |= ((address & 0x7FU) << 1U) | (1U << 14U);

    // Start transaction
    send_start(handle);

    // Send address and read bit
    handle->DR = ((address << 1UL) | 0U);

    // TODO: Wait for ACK or NACK

    for (size_t i = 0U; i < len; i++) {
        // Read byte
        data[i] = handle->DR;

        // TODO: Send ACK and NACK responses
    }

    // End transaction
    send_stop(handle);

    return I2C_OK;
}

i2c_err_t i2c_master_transmit_receive(I2C_TypeDef* handle, uint8_t address, const uint8_t* tx_data,
                                      size_t tx_len, uint8_t* rx_data, size_t rx_len) {
    // Transmit first
    i2c_err_t ret = i2c_master_transmit(handle, address, tx_data, tx_len);
    if (ret != I2C_OK) return ret;
    
    // Then receive
    return i2c_master_receive(handle, address, rx_data, rx_len);
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
