#include "Unity/unity.h"

#include "test/adc.hpp"
#include "test/dma.hpp"
#include "test/i2c.hpp"
#include "test/i2s.hpp"
#include "test/pwm.hpp"
#include "test/spi.hpp"
#include "test/crc.hpp"
#include "test/heap.hpp"
#include "test/iwdg.hpp"
#include "test/uart.hpp"
#include "test/gpio.hpp"
#include "test/timer.hpp"
#include "test/clock.hpp"
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
        crc::all();
        iwdg::all();
        heap::all();
        uart::all();
        gpio::all();
        clock::all();
        timer::all();

        UNITY_END();
    }

} // namespace test
