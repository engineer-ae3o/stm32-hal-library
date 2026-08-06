#include "extra/tick.h"
#include "extra/log.h"


extern "C" {
[[noreturn]] int main() {
    while (true) {
        LOGE("Error", "Testing error logging");
        LOGW("Warn", "Testing warn logging");
        LOGI("Info", "Testing info logging");

        LOGE_ISR("Testing ISR safe error logging");
        LOGW_ISR("Testing ISR safe warn logging");
        LOGI_ISR("Testing ISR safe info logging");

        delay_ms(3'000);
    }
}
}
