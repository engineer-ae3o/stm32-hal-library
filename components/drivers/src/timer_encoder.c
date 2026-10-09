#include "stm32f411xe.h"
#include "drivers/timer_extended.h"
#include "utils/common.h"
#include "utils/err.h"


hal_err_t encoder_init(TIM_TypeDef* handle, const encoder_config_t* config) {
    if (handle == NULL || config == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    // Clear all residual state
    TRY(encoder_deinit(handle));

    // TODO: Handle the initialization for the timer to function as a quadrature decoder

    return HAL_OK;
}

hal_err_t encoder_deinit(TIM_TypeDef* handle) {
    return pcnt_deinit(handle);
}

hal_err_t encoder_start(TIM_TypeDef* handle) {
    return pcnt_start(handle);
}

hal_err_t encoder_stop(TIM_TypeDef* handle) {
    return pcnt_stop(handle);
}

hal_err_t encoder_set_count(TIM_TypeDef* handle, uint32_t count) {
    return pcnt_set_count(handle, count);
}

hal_err_t encoder_get_count(TIM_TypeDef* handle, uint32_t* count) {
    return pcnt_get_count(handle, count);
}
