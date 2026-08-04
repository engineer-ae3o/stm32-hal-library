cmake_minimum_required(VERSION 3.16...4.1)

add_library(unity STATIC src/unity.c)
target_include_directories(unity PUBLIC src/)

set_target_properties(unity PROPERTIES
    C_STANDARD 23
    C_STANDARD_REQUIRED ON
    C_EXTENSIONS ON
)
