#include "stm32f411xe.h"
#include "drivers/timer_internals.h"
#include "drivers/timer_ext.h"
#include "drivers/timer.h"
#include "utils/err.h"


hal_err_t pcnt_init(TIM_TypeDef* handle, const pcnt_config_t* config) {
    if (handle == NULL || config == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    return HAL_OK;
}

hal_err_t pcnt_deinit(TIM_TypeDef* handle) {
    return timer_deinit(handle);
}

hal_err_t pcnt_set_count(TIM_TypeDef* handle, uint32_t count) {
    if (handle == NULL || count > (is_timer_32_bits(handle) ? UINT32_MAX : UINT16_MAX)) {
        return HAL_ERR_INVALID_ARG;
    }
    handle->CNT = count;
    return HAL_OK;
}

hal_err_t pcnt_get_count(TIM_TypeDef* handle, uint32_t* count) {
    if (handle == NULL || count == NULL) {
        return HAL_ERR_INVALID_ARG;
    }
    *count = handle->CNT;
    return HAL_OK;
}
