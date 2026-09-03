cmake_minimum_required(VERSION 4.0)

set(SRCS
    src/i2c.c
    src/i2s.c
    src/crc.c
    src/spi.c
    src/pwm.c
    src/adc.c
    src/dma.c
    src/gpio.c
    src/uart.c
    src/iwdg.c
    src/timer.c
)

set(INC
    ../
    ../third-party/CMSIS/Include/
    ../third-party/CMSIS/Core/Include/
    ../third-party/CMSIS/Device/ST/STM32F4xx/Include/
)

set(DEFINES STM32F411xE __ARM_ARCH_7M__=1 __ARM_ARCH_7EM__=1 RTT_USE_ASM=1)

# Create library
add_library(drivers STATIC ${SRCS})
target_include_directories(drivers PUBLIC ${INC})
target_compile_definitions(drivers PUBLIC ${DEFINES})

set_target_properties(drivers PROPERTIES
    C_STANDARD 23
    C_STANDARD_REQUIRED ON
    C_EXTENSIONS ON
)
