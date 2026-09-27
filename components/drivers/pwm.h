#ifndef PWM_H_
#define PWM_H_


#ifdef __cplusplus
extern "C" {
#endif


#include "stm32f411xe.h"
#include "utils/board.h"
#include "utils/err.h"

#include <stdint.h>
#include <stddef.h>


// Total PWM channels available across all timers is 24 channels
#define MAX_TIM1_CHANNELS (4U)
#define MAX_TIM2_CHANNELS (4U)
#define MAX_TIM3_CHANNELS (4U)
#define MAX_TIM4_CHANNELS (4U)
#define MAX_TIM5_CHANNELS (4U)
#define MAX_TIM9_CHANNELS (2U)
#define MAX_TIM10_CHANNELS (1U)
#define MAX_TIM11_CHANNELS (1U)

typedef struct {
    board_pin_t channels[MAX_TIM1_CHANNELS];
    uint32_t    num_of_channels;
} pwm_config_t;

hal_err_t pwm_init(TIM_TypeDef* handle, const pwm_config_t* config);
hal_err_t pwm_deinit(TIM_TypeDef* handle);

hal_err_t pwm_dma_init(TIM_TypeDef* handle);
hal_err_t pwm_dma_deinit(TIM_TypeDef* handle);

hal_err_t pwm_start(TIM_TypeDef* handle, uint32_t freq_hz);
hal_err_t pwm_set_duty_cycle(TIM_TypeDef* handle, uint32_t duty_cycle);
hal_err_t pwm_send_data_dma(TIM_TypeDef* handle, const uint32_t* samples, size_t size);

hal_err_t pwm_pause(TIM_TypeDef* handle);
hal_err_t pwm_resume(TIM_TypeDef* handle);


#ifdef __cplusplus
}
#endif


#endif // PWM_H_