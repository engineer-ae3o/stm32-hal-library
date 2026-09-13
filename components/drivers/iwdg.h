#ifndef IWDG_H_
#define IWDG_H_


#ifdef __cplusplus
extern "C" {
#endif


#include "utils/err.h"

#include <stdint.h>


typedef enum : uint8_t {
    IWDG_PRESCALER_DIV4   = 0b000,
    IWDG_PRESCALER_DIV8   = 0b001,
    IWDG_PRESCALER_DIV16  = 0b010,
    IWDG_PRESCALER_DIV32  = 0b011,
    IWDG_PRESCALER_DIV64  = 0b100,
    IWDG_PRESCALER_DIV128 = 0b101,
    IWDG_PRESCALER_DIV256 = 0b110,
} iwdg_prescaler_t;

hal_err_t iwdg_start(iwdg_prescaler_t prescaler, uint32_t timeout_ms);
void      iwdg_kick(void);


#ifdef __cplusplus
}
#endif


#endif // IWDG_H_