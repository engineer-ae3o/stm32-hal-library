#ifndef I2C_H_
#define I2C_H_


#ifdef __cplusplus
extern "C" {
#endif


#include "stm32f411xe.h"
#include "drivers/gpio.h"
#include "utils/err.h"

#include <stddef.h>
#include <stdint.h>


typedef enum : uint32_t {
    I2C_FREQ_100KHz = 100'000,
    I2C_FREQ_400KHz = 400'000,
} i2c_frequency_t;

typedef enum : uint8_t {
    I2C_DIGITAL_FILTER_0 = 0, // Digiter filter disabled
    I2C_DIGITAL_FILTER_1,
    I2C_DIGITAL_FILTER_2,
    I2C_DIGITAL_FILTER_3,
    I2C_DIGITAL_FILTER_4,
    I2C_DIGITAL_FILTER_5,
    I2C_DIGITAL_FILTER_6,
    I2C_DIGITAL_FILTER_7,
    I2C_DIGITAL_FILTER_8,
    I2C_DIGITAL_FILTER_9,
    I2C_DIGITAL_FILTER_10,
    I2C_DIGITAL_FILTER_11,
    I2C_DIGITAL_FILTER_12,
    I2C_DIGITAL_FILTER_13,
    I2C_DIGITAL_FILTER_14,
    I2C_DIGITAL_FILTER_15,
} i2c_digi_filt_t;

typedef struct {
    bool use_pullups;

    i2c_frequency_t frequency;
    i2c_digi_filt_t digital_filter;

    gpio_pin_t    sda_pin;
    gpio_pin_t    scl_pin;
    GPIO_TypeDef* gpio_port;
} i2c_master_config_t;

// General API
hal_err_t i2cx_clk_enable(I2C_TypeDef* handle, bool enable);
hal_err_t i2c_master_init(I2C_TypeDef* handle, const i2c_master_config_t* config);
hal_err_t i2c_master_deinit(I2C_TypeDef* handle);

// Polling API. DMA not supported
hal_err_t i2c_master_transmit(I2C_TypeDef* handle, uint8_t address, const uint8_t* data, size_t size);
hal_err_t i2c_master_receive(I2C_TypeDef* handle, uint8_t address, uint8_t* data, size_t size);
hal_err_t i2c_master_transceive(I2C_TypeDef* handle, uint8_t address, const uint8_t* tx_data, size_t tx_size, uint8_t* rx_data, size_t rx_size);


#ifdef __cplusplus
}
#endif


#endif // I2C_H_