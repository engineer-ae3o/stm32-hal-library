#include "stm32f411xe.h"
#include "utils/common.h"
#include "utils/log.h"


[[__gnu__::__noreturn__]] void halt(void) {
    __disable_irq();
#if defined(DEBUG)
    __BKPT(0);
#endif
    while (true) {
        __WFI();
    }
}

void assert_check(bool cond, const char* msg, const char* function, const char* file, int line) {
    if (gnu_unlikely(!cond)) {
        LOGE("Assert", "Assert (%s) failed", msg);
        panic(function, file, line);
    }
}

void panic(const char* function, const char* file, int line) {
    LOGE("Panic", "System ran into a fatal error from %s (%s:%d)", function, file, line);
    halt();
}

void restart(const char* function, const char* file, int line) {
    LOGI("Restart", "System reboot requested from %s (%s:%d)", function, file, line);
    NVIC_SystemReset();
}
