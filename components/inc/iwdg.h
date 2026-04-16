#ifndef _IWDG_H_
#define _IWDG_H_


#include "stdint.h"


void iwdg_start(uint32_t reload_val_s);
void iwdg_kick(void);


#endif // _IWDG_H_