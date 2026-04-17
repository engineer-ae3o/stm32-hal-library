#include "stm32f411xe.h"
#include "iwdg.h"


void iwdg_start(uint32_t reload_val_s) {
    // Enable key register
    IWDG->KR = 0xCCCCU;

    // Unlock prescaler and reload registers
    IWDG->KR = 0x5555U;
    
    // Find the new clock speed
    const uint32_t clk_speed_hz = 32'768U;
    const uint32_t prescaler = 0b110U;
    const uint32_t new_clk_speed_hz = clk_speed_hz / (1U << (prescaler + 2));

    // Find the actual reload value for the RLR and clamp to 12 bits
    uint32_t actual_reload_val = new_clk_speed_hz * reload_val_s;
    if (actual_reload_val > 0xFFFU) actual_reload_val = 0xFFFU;
    
    // Set prescaler of 256: 3 bits
    IWDG->PR &= ~0b111U;
    IWDG->PR |= (prescaler & 0b111U);

    // Reload value: 12 bits
    IWDG->RLR = (actual_reload_val & 0xFFFU);

    // Wait until PVU and RVU bits are 0 after modifying the PR and RLR
    while ((IWDG->SR & IWDG_SR_PVU) || (IWDG->SR & IWDG_SR_RVU));

    // Freeze the WDG when there is a breakpoint
    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;

    // Kick the watchdog
    IWDG->KR = 0xAAAAU;
}

void iwdg_kick(void) {
    IWDG->KR = 0xAAAAU;
}
