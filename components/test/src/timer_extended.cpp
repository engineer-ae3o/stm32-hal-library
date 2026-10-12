#include "stm32f411xe.h"
#include "Unity/unity.h"

#include "drivers/timer_extended.h"
#include "test/timer_extended.hpp"
#include "utils/log.h"


namespace test::timer_ext {

    namespace {

        constexpr const char* TAG = "Timer_ext_Test";

    } // namespace

    void all() {
        LOGI(TAG, "Starting the tests on the extended timer driver");
        UNITY_BEGIN();

        UNITY_END();
        LOGI(TAG, "Done with all tests on the extended timer driver");
    }

} // namespace test::timer_ext
