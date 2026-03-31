set(SRCS
    src/i2c.c
    src/spi.c
    src/pwm.c
    src/wdt.c
    src/adc.c
    src/gpio.c
    src/uart.c
    src/timer.c
)

set(INC
    external/CMSIS/Device/ST/STM32F4xx/Include/
    external/CMSIS/Include/
    inc/
)

# Create library
add_library(hals STATIC ${SRCS})
target_include_directories(hals PUBLIC ${INC})
