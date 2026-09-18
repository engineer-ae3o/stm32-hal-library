#include "stm32f411xe.h"
#include "Unity/unity.h"

#include "drivers/timer.h"
#include "test/timer.hpp"
#include "utils/log.h"


namespace test::timer {

    constexpr const char* TAG = "Timer_Test";

    void all() {
        LOGI(TAG, "Starting the tests on the timer driver");
        UNITY_BEGIN();

        UNITY_END();
        LOGI(TAG, "Done with all tests on the timer driver");
    }

} // namespace test::timer
