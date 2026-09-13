#include "stm32f411xe.h"
#include "drivers/iwdg.h"
#include "utils/common.h"
#include "utils/err.h"


hal_err_t iwdg_start(uint32_t reload_value_s) {
    // Find the new clock speed
    const uint32_t clk_speed_hz     = 32'768U;
    const uint32_t prescaler        = 0b110U;
    const uint32_t new_clk_speed_hz = clk_speed_hz / (1U << (prescaler + 2));

    if (reload_value_s > (IWDG_RLR_RL_Msk / new_clk_speed_hz)) {
        return HAL_ERR_INVALID_ARG;
    }

    // Enable the key register
    IWDG->KR = 0xCCCCU;

    // Unlock prescaler and reload registers
    IWDG->KR = 0x5555U;

    // Find the actual reload value for the RLR and clamp to 12 bits
    uint32_t actual_reload_val = new_clk_speed_hz * reload_value_s;

    // Set a prescaler of 256: 3 bits
    IWDG->PR = (prescaler & IWDG_PR_PR_Msk) | (IWDG->PR & ~IWDG_PR_PR_Msk);

    // Reload value
    IWDG->RLR |= actual_reload_val & IWDG_RLR_RL_Msk;

    // Wait until PVU and RVU bits are 0 after modifying the PR and RLR
    uint32_t timeout = TIMEOUT_CYCLES;
    while (((IWDG->SR & IWDG_SR_PVU) || (IWDG->SR & IWDG_SR_RVU)) && --timeout);
    if (timeout == 0) {
        return HAL_ERR_TIMEOUT;
    }

    // Freeze the WDG when there is a breakpoint
    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;

    // Kick the watchdog
    IWDG->KR = 0xAAAAU;

    return HAL_OK;
}

void iwdg_kick(void) {
    IWDG->KR = 0xAAAAU;
}
