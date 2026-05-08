cmake_minimum_required(VERSION 4.1)

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
