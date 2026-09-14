#include "stm32f411xe.h"
#include "Unity/unity.h"

#include "test/i2s.hpp"
#include "utils/log.h"


namespace test::i2s {

    constexpr const char* TAG = "I2S_Test";

    void all() {
        LOGI(TAG, "Starting the tests on the I2S driver");
        UNITY_BEGIN();

        UNITY_END();
        LOGI(TAG, "Done with all tests on the I2S driver");
    }

} // namespace test::i2s
