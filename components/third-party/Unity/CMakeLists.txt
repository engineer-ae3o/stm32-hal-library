cmake_minimum_required(VERSION 4.0)

add_library(unity STATIC unity.c)
target_include_directories(unity PUBLIC .)

set_target_properties(unity PROPERTIES
    C_STANDARD 23
    C_STANDARD_REQUIRED ON
    C_EXTENSIONS ON
)
