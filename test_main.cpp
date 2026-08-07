#include "extra/common.h"
#include "extra/tick.h"
#include "extra/heap.h"
#include "extra/log.h"


extern "C" {
[[noreturn]] int main() {

    // Stress test the heap
    print_heap_stats();

    // 51 (512 - 8) byte allocations
    ASSERT(lib_malloc(512 - 8));
    ASSERT(lib_malloc(512 - 8));
    ASSERT(lib_malloc(512 - 8));
    ASSERT(lib_malloc(512 - 8));
    ASSERT(lib_malloc(512 - 8));
    ASSERT(lib_malloc(512 - 8));
    ASSERT(lib_malloc(512 - 8));
    ASSERT(lib_malloc(512 - 8));
    ASSERT(lib_malloc(512 - 8));
    ASSERT(lib_malloc(512 - 8));
    ASSERT(lib_malloc(512 - 8));
    ASSERT(lib_malloc(512 - 8));
    ASSERT(lib_malloc(512 - 8));
    ASSERT(lib_malloc(512 - 8));
    ASSERT(lib_malloc(512 - 8));
    ASSERT(lib_malloc(512 - 8));
    ASSERT(lib_malloc(512 - 8));
    ASSERT(lib_malloc(512 - 8));
    ASSERT(lib_malloc(512 - 8));
    ASSERT(lib_malloc(512 - 8));
    ASSERT(lib_malloc(512 - 8));
    ASSERT(lib_malloc(512 - 8));
    ASSERT(lib_malloc(512 - 8));
    ASSERT(lib_malloc(512 - 8));
    ASSERT(lib_malloc(512 - 8));
    ASSERT(lib_malloc(512 - 8));
    ASSERT(lib_malloc(512 - 8));
    ASSERT(lib_malloc(512 - 8));
    ASSERT(lib_malloc(512 - 8));
    ASSERT(lib_malloc(512 - 8));
    ASSERT(lib_malloc(512 - 8));
    ASSERT(lib_malloc(512 - 8));
    ASSERT(lib_malloc(512 - 8));
    ASSERT(lib_malloc(512 - 8));
    ASSERT(lib_malloc(512 - 8));
    ASSERT(lib_malloc(512 - 8));
    ASSERT(lib_malloc(512 - 8));
    ASSERT(lib_malloc(512 - 8));
    ASSERT(lib_malloc(512 - 8));
    ASSERT(lib_malloc(512 - 8));
    ASSERT(lib_malloc(512 - 8));
    ASSERT(lib_malloc(512 - 8));
    ASSERT(lib_malloc(512 - 8));
    ASSERT(lib_malloc(512 - 8));
    ASSERT(lib_malloc(512 - 8));
    ASSERT(lib_malloc(512 - 8));
    ASSERT(lib_malloc(512 - 8));
    ASSERT(lib_malloc(512 - 8));
    ASSERT(lib_malloc(512 - 8));
    ASSERT(lib_malloc(512 - 8));
    ASSERT(lib_malloc(512 - 8));

    // 6 (1024 - 8) byte allocations
    ASSERT(lib_malloc(1024 - 8));
    ASSERT(lib_malloc(1024 - 8));
    ASSERT(lib_malloc(1024 - 8));
    ASSERT(lib_malloc(1024 - 8));
    ASSERT(lib_malloc(1024 - 8));
    ASSERT(lib_malloc(1024 - 8));

    print_heap_stats();

    while (true) {
        LOGE("Error", "Testing error logging");
        LOGW("Warn", "Testing warn logging");
        LOGI("Info", "Testing info logging");

        delay_us(3'000'000);
    }
}
}
