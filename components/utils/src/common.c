#include "stm32f411xe.h"
#include "utils/common.h"
#include "utils/log.h"


void panic(const char* function, const char* file, uint32_t line) {
    LOGE("Panic", "System ran into a fatal error from %s (%s:%lu)", function, file, line);
    __asm volatile("bkpt #0");
    while (true);
}

void restart(const char* function, const char* file, uint32_t line) {
    LOGI("Restart", "System reboot requested from %s (%s:%lu)", function, file, line);
    NVIC_SystemReset();
}
