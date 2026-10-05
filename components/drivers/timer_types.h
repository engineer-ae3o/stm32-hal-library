#ifndef TIMER_TYPES_H_
#define TIMER_TYPES_H_


#ifdef __cplusplus
extern "C" {
#endif


#include "stm32f411xe.h"

#include <stdint.h>


// The extended timer module
// Implements extra functionality from the timer(s)


// Output Compare
typedef enum : uint8_t {
    OC_CHANNEL_1,
    OC_CHANNEL_2,
    OC_CHANNEL_3,
    OC_CHANNEL_4,
} timer_oc_channel_t;

typedef struct {

} timer_oc_config_t;


// Input Capture


// Pulse Counter


// Quadrature decoder (encoder mode)


#ifdef __cplusplus
}
#endif


#endif // TIMER_TYPES_H_