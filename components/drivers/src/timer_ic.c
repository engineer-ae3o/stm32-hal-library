#include "stm32f411xe.h"
#include "drivers/timer_extended.h"
#include "drivers/timer.h"
#include "utils/err.h"


hal_err_t input_capture_init(TIM_TypeDef* handle, const input_capture_config_t* config) {
    if (handle == NULL || config == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    return HAL_OK;
}

hal_err_t input_capture_deinit(TIM_TypeDef* handle) {
    return timer_deinit(handle);
}
