#include "stm32f411xe.h"
#include "Unity/unity.h"

#include "test/pwm.hpp"
#include "utils/log.h"


namespace test::pwm {

    constexpr const char* TAG = "PWM_Test";

    void all() {
        LOGI(TAG, "Starting the tests on the PWM driver");
        UNITY_BEGIN();

        UNITY_END();
        LOGI(TAG, "Done with all tests on the PWM driver");
    }

} // namespace test::pwm
