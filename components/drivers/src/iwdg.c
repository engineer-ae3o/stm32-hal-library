#include "stm32f411xe.h"
#include "drivers/iwdg.h"
#include "utils/common.h"
#include "utils/board.h"
#include "utils/err.h"


hal_err_t iwdg_start(iwdg_prescaler_t prescaler, uint32_t timeout_ms) {
    if (prescaler > IWDG_PRESCALER_DIV256) {
        return HAL_ERR_INVALID_ARG;
    }

    // Find the new clock speed
    const uint32_t new_clk_speed_hz = LSI_VALUE_Hz / (1U << (prescaler + 2));

    if (timeout_ms > ((IWDG_RLR_RL_Msk * 1000U) / new_clk_speed_hz)) {
        return HAL_ERR_INVALID_ARG;
    }

    // Enable the key register and unlock the prescaler and reload registers
    IWDG->KR = 0xCCCCU;
    IWDG->KR = 0x5555U;

    // Find the actual reload value for the RLR and clamp to 12 bits
    uint32_t actual_reload_val = (new_clk_speed_hz * timeout_ms) / 1000U;

    // Wait until the PVU and RVU bits read 0 before we can safely modify the PR and RLR
    uint32_t timeout = TIMEOUT_CYCLES;
    while (((IWDG->SR & IWDG_SR_PVU) || (IWDG->SR & IWDG_SR_RVU)) && --timeout);
    if (timeout == 0) {
        return HAL_ERR_TIMEOUT;
    }

    // Set the prescaler and reload value
    IWDG->PR  = prescaler & IWDG_PR_PR_Msk;
    IWDG->RLR = actual_reload_val & IWDG_RLR_RL_Msk;

    // Wait until the PVU and RVU bits read 0 again since we have modified the PR and RLR
    timeout = TIMEOUT_CYCLES;
    while (((IWDG->SR & IWDG_SR_PVU) || (IWDG->SR & IWDG_SR_RVU)) && --timeout);
    if (timeout == 0) {
        return HAL_ERR_TIMEOUT;
    }

    // Freeze the IWDG if a breakpoint is active
    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;

    // Kick the watchdog
    IWDG->KR = 0xAAAAU;

    return HAL_OK;
}

void iwdg_kick(void) {
    IWDG->KR = 0xAAAAU;
}
