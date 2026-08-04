cmake_minimum_required(VERSION 3.16...4.1)

add_library(rtt STATIC
    RTT/SEGGER_RTT_ASM_ARMv7M.S
    RTT/SEGGER_RTT_printf.c
    RTT/SEGGER_RTT.c
)

target_include_directories(rtt PUBLIC RTT/)

set_target_properties(rtt PROPERTIES
    C_STANDARD 23
    C_STANDARD_REQUIRED ON
    C_EXTENSIONS ON
)
