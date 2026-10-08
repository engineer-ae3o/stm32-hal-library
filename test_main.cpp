#include "test/profile.hpp"
#include "test/runner.hpp"
#include "utils/common.h"
#include "test/demo.hpp"
#include "utils/log.h"


extern "C" {
int main() {
    // Run all the hardware driver tests
    test::runner();

    // Run the profile tests
    // profile::all();

    // Run the hardware demo
    // demo::all();

    // Halt once tests are finished since nothing else to do.
    LOGI("Main", "Done with all tests. Halting...");
    halt();
}
}
