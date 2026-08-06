#ifndef _TICK_H_
#define _TICK_H_


#ifdef __cplusplus
extern "C" {
#endif


#include <stdint.h>


__attribute__((constructor)) void tick_init(void);

uint32_t ticks_since_boot_ms(void);

void delay_ms(uint32_t ms);

void delay_us(uint32_t us);


#ifdef __cplusplus
}
#endif


#endif // _TICK_H_