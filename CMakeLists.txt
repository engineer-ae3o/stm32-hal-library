cmake_minimum_required(VERSION 4.0)

set(CMAKE_C_STANDARD 23)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

set(CMAKE_PROJECT_NAME f411-hal-library)

project(${CMAKE_PROJECT_NAME})
enable_language(C CXX ASM)

set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


# Build flags for the different build modes
set(CMAKE_C_FLAGS_DEBUG "-O0 -g3 -ggdb -DDEBUG")
set(CMAKE_C_FLAGS_RELEASE "-O2 -Wl,-s -DNDEBUG")

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG}")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE}")


# Location of linker script
set(LINKER_SCRIPT "${CMAKE_SOURCE_DIR}/linker.ld")


# Build flags
set(CMAKE_C_FLAGS "-Wall -Wstrict-aliasing=2 -Wimplicit-int -Wduplicated-cond \
                   -Werror -Wextra -Wshadow -Wconversion -Wwrite-strings -Wformat=2 \
                   -Wsign-conversion -fstrict-aliasing -Wdouble-promotion -fstack-usage \
                   -Wstrict-prototypes -Wnull-dereference -Wcast-align -fstrict-overflow  \
                   -Wduplicated-branches -Wswitch-enum -Wlogical-op")

set(CMAKE_CXX_FLAGS "${MCU_FLAGS}")

set(CMAKE_EXE_LINKER_FLAGS "-T${LINKER_SCRIPT} -Wl,-Map=${CMAKE_PROJECT_NAME}.map -Wl,--cref \
                            -Wl,--gc-sections --specs=nano.specs")

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
endif()

message("Build type: ${CMAKE_BUILD_TYPE}")

target_link_libraries(${CMAKE_PROJECT_NAME} PUBLIC
    hals
    external
)

target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC components/external/CMSIS/Device/ST/STM32F4xx/Include/
                                                        components/external/CMSIS/Include/
)

target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE STM32F411xE)

add_subdirectory(components/hal/)
add_subdirectory(components/test/)
add_subdirectory(components/external/RTT/)
add_subdirectory(components/external/CMSIS/)
add_subdirectory(components/external/Unity/)

# Commands
add_custom_target(build
    COMMENT "Building project"
    DEPENDS ${CMAKE_PROJECT_NAME}
    COMMAND ${CMAKE_OBJCOPY} -O binary ${CMAKE_PROJECT_NAME} ${CMAKE_PROJECT_NAME}.bin
    COMMAND ${CMAKE_OBJDUMP} -D -S ${CMAKE_PROJECT_NAME} > ${CMAKE_PROJECT_NAME}.s
    COMMAND ${CMAKE_SIZE} ${CMAKE_PROJECT_NAME}
)

# Flash via openocd
add_custom_target(flash
    COMMAND openocd -f ${CMAKE_SOURCE_DIR}/openocd.cfg -c "program ${CMAKE_PROJECT_NAME} verify reset exit"
    DEPENDS ${CMAKE_PROJECT_NAME}
    COMMENT "Flashing device"
)

# Start serial monitor
add_custom_target(monitor
    COMMAND 
    MESSAGE "Loading serial monitor"
)

# Start an OpenOCD server
add_custom_target(openocd
    COMMAND openocd -f ${CMAKE_SOURCE_DIR}/openocd.cfg
    DEPENDS ${CMAKE_PROJECT_NAME}
    COMMENT "Starting OpenOCD server"
)

# Launch GDB
add_custom_target(gdb
    COMMAND ${CMAKE_GDB} ${CMAKE_PROJECT_NAME} -ex "target remote: 1234"
    DEPENDS ${CMAKE_PROJECT_NAME}
    COMMENT "Launching GDB"
)
