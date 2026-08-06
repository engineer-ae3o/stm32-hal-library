#ifndef IWDG_H_
#define IWDG_H_


#ifdef __cplusplus
extern "C" {
#endif


#include <stdint.h>


void iwdg_start(uint32_t reload_val_s);
void iwdg_kick(void);


#ifdef __cplusplus
}
#endif


#endif // IWDG_H_