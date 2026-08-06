cmake_minimum_required(VERSION 4.0)

set(SRCS
    src/log.c
    src/err.c
    src/heap.c
    src/tick.c
    src/syscalls.c
)

set(INC
    ../
    ../third-party/o1heap/
    ../third-party/RTT/RTT/
    ../third-party/CMSIS/Include/
    ../third-party/CMSIS/Core/Include/
    ../third-party/CMSIS/Device/ST/STM32F4xx/Include/
)

# Create library
add_library(extra STATIC ${SRCS})
target_include_directories(extra PUBLIC ${INC})

set_target_properties(extra PROPERTIES
    C_STANDARD 23
    C_STANDARD_REQUIRED ON
    C_EXTENSIONS ON
)
