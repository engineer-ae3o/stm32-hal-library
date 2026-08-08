cmake_minimum_required(VERSION 4.0)

add_library(printf STATIC printf.c)
target_include_directories(printf PUBLIC .)

set_target_properties(printf PROPERTIES
    C_STANDARD 23
    C_STANDARD_REQUIRED ON
    C_EXTENSIONS ON
)
