cmake_minimum_required(VERSION 4.0)

set(SRCS
    src/log.c
    src/err.c
    src/heap.c
    src/tick.c
    src/random.c
    src/common.c
    src/syscalls.c
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
add_library(utils STATIC ${SRCS})
target_include_directories(utils PUBLIC ${INC})
target_compile_definitions(utils PUBLIC ${DEFINES})

set_target_properties(utils PROPERTIES
    C_STANDARD 23
    C_STANDARD_REQUIRED ON
    C_EXTENSIONS ON
)
