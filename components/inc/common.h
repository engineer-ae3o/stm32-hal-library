#ifndef _COMMON_H_
#define _COMMON_H_


#ifdef __cplusplus
extern "C" {
#endif


#include <stdint.h>


typedef enum hal_err_t : uint8_t {
    HAL_OK                    = 0x00U,
    HAL_FAIL                  = 0x01U,

    HAL_INVALID_ARG           = 0x02U,
    HAL_INVALID_STATE         = 0x03U,
    HAL_TIMEOUT               = 0x04U,

    HAL_TX_ERROR              = 0x05U,
    HAL_RX_ERROR              = 0x06U,

    HAL_I2C_DEVICE_NOT_FOUND  = 0x07U,
    HAL_I2C_ARBITRATION_LOST  = 0x08U
} hal_err_t;


static inline const char* hal_err_to_string(hal_err_t err) {
    switch (err) {
        case HAL_OK:                   return "HAL_OK";
        case HAL_FAIL:                 return "HAL_FAIL";
        case HAL_INVALID_ARG:          return "HAL_INVALID_ARG";
        case HAL_INVALID_STATE:        return "HAL_INVALID_STATE";
        case HAL_TIMEOUT:              return "HAL_TIMEOUT";
        case HAL_TX_ERROR:             return "HAL_TX_ERROR";
        case HAL_RX_ERROR:             return "HAL_RX_ERROR";
        case HAL_I2C_DEVICE_NOT_FOUND: return "HAL_I2C_DEVICE_NOT_FOUND";
        case HAL_I2C_ARBITRATION_LOST: return "HAL_I2C_ARBITRATION_LOST";
        default:                       return "";
    }
}


#ifdef __cplusplus
}
#endif


#endif // _COMMON_H_