#include "test/runner.hpp"
#include "utils/heap.h"
#include "utils/log.h"


extern "C" {
[[noreturn]] int main() {
    // Run all the tests and halt.
    test::runner();

    // Halt once tests are finished since nothing else to do.
    LOGI("Main", "Done with all tests. Halting...");
    while (true);
}
}
