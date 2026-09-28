#include "stm32f411xe.h"
#include "drivers/timer_internals.h"
#include "drivers/pwm.h"


hal_err_t pwm_init(TIM_TypeDef* handle, const pwm_config_t* config) {
    (void)handle;
    (void)config;
    return HAL_OK;
}

hal_err_t pwm_deinit(TIM_TypeDef* handle) {
    (void)handle;
    return HAL_OK;
}

hal_err_t pwm_dma_init(TIM_TypeDef* handle) {
    (void)handle;
    return HAL_OK;
}

hal_err_t pwm_dma_deinit(TIM_TypeDef* handle) {
    (void)handle;
    return HAL_OK;
}

hal_err_t pwm_start(TIM_TypeDef* handle, uint32_t freq_hz) {
    (void)handle;
    (void)freq_hz;
    return HAL_OK;
}

hal_err_t pwm_set_duty_cycle(TIM_TypeDef* handle, uint32_t duty_cycle) {
    (void)handle;
    (void)duty_cycle;
    return HAL_OK;
}

hal_err_t pwm_send_data_dma(TIM_TypeDef* handle, const uint32_t* samples, size_t size) {
    (void)handle;
    (void)samples;
    (void)size;
    return HAL_OK;
}

hal_err_t pwm_pause(TIM_TypeDef* handle) {
    (void)handle;
    return HAL_OK;
}

hal_err_t pwm_resume(TIM_TypeDef* handle) {
    (void)handle;
    return HAL_OK;
}
