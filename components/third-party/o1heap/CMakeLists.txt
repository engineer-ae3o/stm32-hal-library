cmake_minimum_required(VERSION 4.0)

add_library(o1heap STATIC o1heap.c)
target_include_directories(o1heap PUBLIC .)

set_target_properties(o1heap PROPERTIES
    C_STANDARD 23
    C_STANDARD_REQUIRED ON
    C_EXTENSIONS ON
)
