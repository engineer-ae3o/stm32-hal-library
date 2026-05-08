#ifndef _I2C_H_
#define _I2C_H_


#ifdef __cplusplus
extern "C" {
#endif


#include "stm32f411xe.h"
#include "common.h"

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>


typedef enum : uint8_t {
    I2C_100KHz = 0,
    I2C_400KHz
} i2c_freq_mode_t;

typedef struct {
    bool use_pullup;
    i2c_freq_mode_t freq_type;
    uint8_t apb1_bus_freq_mhz;

    uint8_t sda;
    uint8_t scl;
    GPIO_TypeDef* gpio_port;
} i2c_master_config_t;

hal_err_t i2cx_clk_enable(I2C_TypeDef* handle, bool enable);
hal_err_t i2c_master_init(I2C_TypeDef* handle, const i2c_master_config_t* config);
hal_err_t i2c_master_transmit(I2C_TypeDef* handle, uint8_t address, const uint8_t* data, size_t len);
hal_err_t i2c_master_receive(I2C_TypeDef* handle, uint8_t address, uint8_t* data, size_t len);
hal_err_t i2c_master_transceive(I2C_TypeDef* handle, uint8_t address, const uint8_t* tx_data,
                                size_t tx_len, uint8_t* rx_data, size_t rx_len);


#ifdef __cplusplus
}
#endif


#endif // _I2C_H_