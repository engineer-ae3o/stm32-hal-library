#ifndef _I2C_H_
#define _I2C_H_


#include "stm32f411xe.h"
#include "stddef.h"
#include "stdint.h"


typedef enum {
    I2C_OK = 0,
    I2C_FAIL,
    TX_ERROR,
    RX_ERROR,
    I2C_INVALID_ARG,
    I2C_INVALID_STATE,
    I2C_DEVICE_NOT_FOUND
} i2c_err_t;

typedef struct {
    bool pullups_enable;
    uint32_t clock_speed_hz;

    uint32_t sda;
    uint32_t scl;
    GPIO_TypeDef* gpio_port;
} i2c_master_config_t;

i2c_err_t i2c_master_init(I2C_TypeDef* handle, const i2c_master_config_t* config);
i2c_err_t i2c_master_transmit(I2C_TypeDef* handle, uint8_t address, const uint8_t* data, size_t len);
i2c_err_t i2c_master_receive(I2C_TypeDef* handle, uint8_t address, uint8_t* data, size_t len);
i2c_err_t i2c_master_transmit_receive(I2C_TypeDef* handle, uint8_t address, const uint8_t* tx_data,
                                      size_t tx_len, uint8_t* rx_data, size_t rx_len);

typedef struct {
    bool pullups_enable;
    uint32_t clock_speed_hz;

    uint32_t sda;
    uint32_t scl;
    GPIO_TypeDef* gpio_port;
} i2c_slave_config_t;

i2c_err_t i2c_slave_init(I2C_TypeDef* handle, const i2c_slave_config_t* config);
i2c_err_t i2c_slave_transmit(I2C_TypeDef* handle, const uint8_t* data, size_t len);
i2c_err_t i2c_slave_receive(I2C_TypeDef* handle, uint8_t* data, size_t len);


#endif // _I2C_H_