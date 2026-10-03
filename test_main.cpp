#include "drivers/pwm_types.h"
#include "test/profile.hpp"
#include "test/runner.hpp"
#include "utils/board.h"
#include "utils/common.h"
#include "utils/log.h"

#include "drivers/pwm.h"


extern "C" {
int main() {
    // Run all the hardware driver tests
    test::runner();

    // Run the profile tests
    // profile::all();

    // Halt once tests are finished since nothing else to do.
    LOGI("Main", "Done with all tests. Halting...");
    halt();
}
}
