#ifndef IWDG_H_
#define IWDG_H_


#ifdef __cplusplus
extern "C" {
#endif


#include "utils/err.h"

#include <stdint.h>


#define IWDG_PRESCALER 0b110U

hal_err_t iwdg_start(uint32_t reload_value_s);
void      iwdg_kick(void);


#ifdef __cplusplus
}
#endif


#endif // IWDG_H_