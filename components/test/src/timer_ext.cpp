#include "stm32f411xe.h"
#include "Unity/unity.h"

#include "drivers/timer_ext.h"
#include "test/timer_ext.hpp"
#include "utils/log.h"


namespace test::timer_ext {

    constexpr const char* TAG = "Timer extended Test";

    void all() {
        LOGI(TAG, "Starting the tests on the extended timer driver");
        UNITY_BEGIN();

        UNITY_END();
        LOGI(TAG, "Done with all tests on the extended timer driver");
    }

} // namespace test::timer_ext
