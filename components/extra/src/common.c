#include "stm32f411xe.h"
#include "extra/common.h"
#include "extra/log.h"


void panic(const char* function, const char* file, uint32_t line) {
    LOGE("Panic", "System ran into a fatal error from %s (%s:%lu)", function, file, line);
    __asm volatile("bkpt #0");
    while (true);
}

void restart(const char* function, const char* file, uint32_t line) {
    LOGI("Restart", "System reboot requested from %s (%s:%lu)", function, file, line);
    NVIC_SystemReset();
    while (true);
}

void assert_check(bool cond, const char* msg, const char* function, const char* file, uint32_t line) {
    if (!cond) {
        LOGE("Assert", "Assert (%s) failed", msg);
        panic(function, file, line);
    }
}
