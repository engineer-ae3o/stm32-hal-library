#include "test/runner.hpp"

extern "C" {
[[noreturn]] int main() {
    // Run all the tests and halt.
    test::runner();

    // Halt once tests are finished since nothing else to do.
    while (true);
}
}
