#include "stm32f411xe.h"
#include "Unity/unity.h"

#include "drivers/timer_pcnt.h"
#include "test/timer_pcnt.hpp"
#include "utils/log.h"


namespace test::timer_pcnt {

    namespace {

        constexpr const char* TAG = "PCNT_Test";

    } // namespace

    void all() {
        LOGI(TAG, "Starting the tests on the PCNT driver");
        UNITY_BEGIN();

        UNITY_END();
        LOGI(TAG, "Done with all tests on the PCNT driver");
    }

} // namespace test::timer_pcnt
