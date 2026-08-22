cmake_minimum_required(VERSION 4.0)

if (BUILD_TESTS)

set(SRCS
    src/i2c.cpp
    src/i2s.cpp
    src/rtc.cpp
    src/spi.cpp
    src/pwm.cpp
    src/adc.cpp
    src/dma.cpp
    src/heap.cpp
    src/gpio.cpp
    src/uart.cpp
    src/wwdg.cpp
    src/iwdg.cpp
    src/timer.cpp
    src/runner.cpp
)

set(INC
    ../
    ../third-party/
    ../third-party/CMSIS/Include/
    ../third-party/CMSIS/Core/Include/
    ../third-party/CMSIS/Device/ST/STM32F4xx/Include/
)

set(DEFINES STM32F411xE __ARM_ARCH_7M__=1 __ARM_ARCH_7EM__=1 RTT_USE_ASM=1)

# Create library
add_library(tests STATIC ${SRCS})
target_include_directories(tests PUBLIC ${INC})
target_compile_definitions(tests PUBLIC ${DEFINES})

set_target_properties(tests PROPERTIES
    CXX_STANDARD 23
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS ON
)

endif()
