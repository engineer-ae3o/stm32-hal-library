#include "unity.h"

#include "test/adc.hpp"
#include "test/dma.hpp"
#include "test/i2c.hpp"
#include "test/i2s.hpp"
#include "test/pwm.hpp"
#include "test/spi.hpp"
#include "test/rtc.hpp"
#include "test/heap.hpp"
#include "test/iwdg.hpp"
#include "test/wwdg.hpp"
#include "test/uart.hpp"
#include "test/gpio.hpp"
#include "test/timer.hpp"
#include "test/runner.hpp"


extern "C" {
void setUp() {
}

void tearDown() {
}
}

namespace test {

    void runner() {
        UNITY_BEGIN();

        // Test runners for all components
        adc::all();
        dma::all();
        i2c::all();
        i2s::all();
        pwm::all();
        spi::all();
        rtc::all();
        iwdg::all();
        wwdg::all();
        heap::all();
        uart::all();
        gpio::all();
        timer::all();

        UNITY_END();
    }

} // namespace test
