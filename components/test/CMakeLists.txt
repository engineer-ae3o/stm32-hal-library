cmake_minimum_required(VERSION 4.0)

if (BUILD_TESTS)

add_library(tests STATIC
    src/test_adc.cpp
)

set_target_properties(tests PROPERTIES
    CXX_STANDARD 23
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS ON
)

target_compile_options(tests PRIVATE -ffast-math
                                       -fno-math-errno
                                       -Wall
                                       -fdata-sections
                                       -Wnoexcept
                                       -Werror
                                       -Wextra
                                       -Wshadow
                                       -Wwrite-strings
                                       -Wformat=2
                                       -ffunction-sections
                                       -fno-common
                                       -fstrict-aliasing
                                       -Wdouble-promotion
                                       -fstack-usage
                                       -Wnull-dereference
                                       -fstrict-overflow
                                       -fno-exceptions
                                       -fno-rtti
                                       -Wstrict-null-sentinel
                                       -Wduplicated-branches
                                       -Wswitch-enum
                                       -Wlogical-op
                                       -Wduplicated-cond
                                       -Wstrict-aliasing=2
                                       -fno-threadsafe-statics
)

target_include_directories(tests PUBLIC inc/
                                          ../drivers/inc/
                                          ../Unity/src/
                                          ../cmsis-device-f1/Core/Include/m-profile/
                                          ../cmsis-device-f1/Core/Include/
                                          ../cmsis-device-f1/Include/
                                          ../RTT/RTT/
)

endif()