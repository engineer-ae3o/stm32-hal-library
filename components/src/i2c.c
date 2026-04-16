#include "stm32f411xe.h"
#include "i2c.h"


// Public API
// I2C Master API
i2c_err_t i2c_master_init(I2C_TypeDef* handle, const i2c_master_config_t* config) {

    (void)handle;
    (void)config;
    
    return I2C_OK;
}

i2c_err_t i2c_master_transmit(I2C_TypeDef* handle, uint8_t address, const uint8_t* data, size_t len) {
    
    (void)handle;
    (void)address;
    (void)data;
    (void)len;

    return I2C_OK;
}

i2c_err_t i2c_master_receive(I2C_TypeDef* handle, uint8_t address, uint8_t* data, size_t len) {

    (void)handle;
    (void)address;
    (void)data;
    (void)len;

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
