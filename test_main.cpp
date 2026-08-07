#include "extra/common.h"
#include "extra/tick.h"
#include "extra/heap.h"
#include "extra/log.h"


extern "C" {
[[noreturn]] int main() {

    // Stress test the heap
    print_heap_stats();

    void* buffer1 = lib_malloc(15 * 1024);
    ASSERT(buffer1 != NULL);

    print_heap_stats();

    void* buffer2 = lib_malloc(7 * 1024);
    ASSERT(buffer2 != NULL);

    print_heap_stats();

    void* buffer3 = lib_malloc(3 * 1024);
    ASSERT(buffer3 != NULL);

    print_heap_stats();

    void* buffer4 = lib_malloc(1 * 1024);
    ASSERT(buffer4 != NULL);

    print_heap_stats();

    void* buffer5 = lib_malloc(511);
    ASSERT(buffer5 != NULL);

    print_heap_stats();

    void* buffer6 = lib_malloc(254);
    ASSERT(buffer6 != NULL);

    print_heap_stats();

    void* buffer7 = lib_malloc(127);
    ASSERT(buffer7 != NULL);

    print_heap_stats();

    void* buffer8 = lib_malloc(31);
    ASSERT(buffer8 != NULL);

    print_heap_stats();

    lib_free(buffer8);
    lib_free(buffer7);
    lib_free(buffer6);
    lib_free(buffer5);
    lib_free(buffer4);
    lib_free(buffer3);
    lib_free(buffer2);
    lib_free(buffer1);

    print_heap_stats();

    while (true) {
        LOGE("Error", "Testing error logging");
        LOGW("Warn", "Testing warn logging");
        LOGI("Info", "Testing info logging");

        delay_us(3'000'000);
    }
}
}
