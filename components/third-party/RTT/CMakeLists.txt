cmake_minimum_required(VERSION 4.0)

add_library(rtt STATIC
    SEGGER_RTT_ASM_ARMv7M.S
    SEGGER_RTT_printf.c
    SEGGER_RTT.c
)

target_include_directories(rtt PUBLIC .)

set_target_properties(rtt PROPERTIES
    C_STANDARD 23
    C_STANDARD_REQUIRED ON
    C_EXTENSIONS ON
)
