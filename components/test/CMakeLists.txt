cmake_minimum_required(VERSION 4.0)

if (BUILD_TESTS)

set(SRCS
    src/test_i2c.cpp
    src/test_i2s.cpp
    src/test_rtc.cpp
    src/test_spi.cpp
    src/test_pwm.cpp
    src/test_adc.cpp
    src/test_dma.cpp
    src/test_gpio.cpp
    src/test_uart.cpp
    src/test_wwdg.cpp
    src/test_iwdg.cpp
    src/test_timer.cpp
)

set(INC
    inc/
    ../third-party/o1heap/
    ../third-party/RTT/RTT/
    ../third-party/Unity/src/
    ../third-party/CMSIS/Include/
    ../third-party/CMSIS/Core/Include/
    ../third-party/CMSIS/Device/ST/STM32F4xx/Include/
)

# Create library
add_library(tests STATIC ${SRCS})
target_include_directories(tests PUBLIC ${INC})

set_target_properties(tests PROPERTIES
    CXX_STANDARD 23
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS ON
)

endif()
