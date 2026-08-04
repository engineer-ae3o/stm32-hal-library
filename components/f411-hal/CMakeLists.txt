cmake_minimum_required(VERSION 4.0)

set(SRCS
    src/i2c.c
    src/i2s.c
    src/rtc.c
    src/spi.c
    src/pwm.c
    src/adc.c
    src/dma.c
    src/gpio.c
    src/uart.c
    src/wwdg.c
    src/iwdg.c
    src/timer.c
    src/syscalls.c
)

set(INC
    external/CMSIS/Device/ST/STM32F4xx/Include/
    external/CMSIS/Include/
    inc/
)

# Create library
add_library(hals STATIC ${SRCS})
target_include_directories(hals PUBLIC ${INC})

set_target_properties(hals PROPERTIES
    C_STANDARD 23
    C_STANDARD_REQUIRED ON
    C_EXTENSIONS ON
    CXX_STANDARD 23
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS ON
)
