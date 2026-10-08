#include "stm32f411xe.h"
#include "drivers/timer_extended.h"
#include "drivers/timer.h"
#include "utils/common.h"
#include "utils/err.h"


hal_err_t encoder_init(TIM_TypeDef* handle, const encoder_config_t* config) {
    if (handle == NULL || config == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    // Clear all residual state
    TRY(pcnt_deinit(handle));

    // TODO: Handle the initialization for the timer to function as a quadrature decoder

    return HAL_OK;
}

hal_err_t encoder_deinit(TIM_TypeDef* handle) {
    return timer_deinit(handle);
}

hal_err_t encoder_start(TIM_TypeDef* handle) {
    if (handle == NULL) {
        return HAL_ERR_INVALID_ARG;
    }
    handle->CR1 |= TIM_CR1_CEN;
    return HAL_OK;
}

hal_err_t encoder_stop(TIM_TypeDef* handle) {
    if (handle == NULL) {
        return HAL_ERR_INVALID_ARG;
    }
    handle->CR1 &= ~TIM_CR1_CEN;
    return HAL_OK;
}
