#ifndef _IWDG_H_
#define _IWDG_H_


#ifdef __cplusplus
extern "C" {
#endif


#include <stdint.h>


void iwdg_start(uint32_t reload_val_s);
void iwdg_kick(void);


#ifdef __cplusplus
}
#endif


#endif // _IWDG_H_