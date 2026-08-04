cmake_minimum_required(VERSION 4.0)

# Add default toolchain file if one not added yet
if(NOT CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/toolchain.cmake)
endif()

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)


# Create project
set(CMAKE_PROJECT_NAME f411-hal-library)

project(${CMAKE_PROJECT_NAME})
enable_language(C CXX ASM)

set(EXECUTABLE_NAME f411-hal)
set(EXECUTABLE_ASM_NAME "${EXECUTABLE_NAME}.s")
set(EXECUTABLE_ELF_NAME "${EXECUTABLE_NAME}.elf")
set(EXECUTABLE_BIN_NAME "${EXECUTABLE_NAME}.bin")
set(EXECUTABLE_MAP_NAME "${EXECUTABLE_NAME}.map")


set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


# Build flags for the different build modes
set(CMAKE_C_FLAGS_DEBUG "-O0 -g3 -ggdb -DDEBUG")
set(CMAKE_C_FLAGS_RELEASE "-O2 -Wl,-s -DNDEBUG")

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG}")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE}")


# Build flags
set(CMAKE_C_FLAGS "-Wall -Wstrict-aliasing=2 -Wimplicit-int -Wduplicated-cond -Werror \
                   -Wlogical-op -Wextra -Wshadow -Wconversion -Wwrite-strings -Wformat=2 \
                   -Wsign-conversion -fstrict-aliasing -Wdouble-promotion -fstack-usage \
                   -Wstrict-prototypes -Wnull-dereference -Wcast-align -fstrict-overflow  \
                   -Wduplicated-branches -Wswitch-enum")

set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -fno-exceptions -fno-rtti")


# Set linker flags
set(LINKER_SCRIPT "${CMAKE_SOURCE_DIR}/stm32411-linker.ld")
set(CMAKE_EXE_LINKER_FLAGS "-T${LINKER_SCRIPT} -Wl,-Map=${EXECUTABLE_MAP_NAME} -Wl,--cref \
                            -Wl,--gc-sections --specs=nano.specs")

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
endif()

message("Build type: ${CMAKE_BUILD_TYPE}")


# Enable tests to be built. Overridable via -DBUILD_TESTS=ON/OFF.
option(BUILD_TESTS "Build the Unity test suite" ON)

# Tests pull in Unity, take up extra flash/RAM, and are meant for interactive
# debug sessions. A release image with tests baked in isn't a real release
# artifact, so refuse to configure that combination.
if(BUILD_TESTS AND CMAKE_BUILD_TYPE STREQUAL "Release")
    message(FATAL_ERROR
        "BUILD_TESTS=ON is incompatible with CMAKE_BUILD_TYPE=Release. "
        "Tests are for Debug builds only; use -DBUILD_TESTS=OFF for Release, "
        "or build Debug if you need the test suite.")
endif()


# Process subdirectories
add_subdirectory(components/f411-hal/)
add_subdirectory(components/third-party/RTT/)
add_subdirectory(components/third-party/o1heap/)

if (BUILD_TESTS)
    add_subdirectory(components/test/)
    add_subdirectory(components/third-party/Unity/)
endif()


# Create the final static library
add_executable(${EXECUTABLE_ELF_NAME} src/main.cpp src/startup.s)
target_compile_definitions(${EXECUTABLE_ELF_NAME} PUBLIC STM32F411xE)


# Link components
if (BUILD_TESTS)
    target_link_libraries(${EXECUTABLE_ELF_NAME} PUBLIC tests unity hals rtt o1heap)
else()
    target_link_libraries(${EXECUTABLE_ELF_NAME} PUBLIC hals rtt o1heap)
endif()


# Set the language standards
set_target_properties(${EXECUTABLE_ELF_NAME} PROPERTIES
    CXX_STANDARD 23
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS ON
)

# Build project to .elf, produce the .bin, dump to get the disassembly and print size
add_custom_target(build
    COMMENT "Building project"
    DEPENDS ${EXECUTABLE_ELF_NAME}
    COMMAND ${CMAKE_OBJCOPY} -O binary ${EXECUTABLE_ELF_NAME} ${EXECUTABLE_BIN_NAME}
    COMMAND ${CMAKE_OBJDUMP} -D -S ${EXECUTABLE_ELF_NAME} > ${EXECUTABLE_ASM_NAME}
    COMMAND ${CMAKE_SIZE} ${EXECUTABLE_ELF_NAME}
)

# Static analysis with cppcheck
find_program(CPPCHECK cppcheck)

if(CPPCHECK)
    message(STATUS "Cppcheck found: ${CPPCHECK}")

    add_custom_target(cppcheck
    COMMAND mkdir -p ${CMAKE_BINARY_DIR}/.cppcheck
    COMMAND ${CPPCHECK} --enable=all
        --inline-suppr
        --cppcheck-build-dir=${CMAKE_BINARY_DIR}/.cppcheck
        --suppressions-list=${CMAKE_SOURCE_DIR}/cppcheck_suppressions.txt
        --error-exitcode=1
        --check-level=exhaustive
        -DSTM32F411xE -D__GNUC__ -D__ARM_ARCH_7M__ -D__ARM_ARCH_PROFILE=77
        -I ${CMAKE_SOURCE_DIR}/components/drivers/inc/
        -I ${CMAKE_SOURCE_DIR}/components/test/inc/
        -I ${CMAKE_SOURCE_DIR}/components/Unity/src/
        -I ${CMAKE_SOURCE_DIR}/components/f103-hal/Inc/
        -I ${CMAKE_SOURCE_DIR}/components/cmsis-device-f1/Core/Include/m-profile/
        -I ${CMAKE_SOURCE_DIR}/components/cmsis-device-f1/Core/Include/
        -I ${CMAKE_SOURCE_DIR}/components/cmsis-device-f1/Include/
        -I ${CMAKE_SOURCE_DIR}/components/freertos/include/
        -I ${CMAKE_SOURCE_DIR}/components/ascon-suite/src/
        -I ${CMAKE_SOURCE_DIR}/components/RTT/RTT/
        -i ${CMAKE_SOURCE_DIR}/components/cmsis-device-f1/
        -i ${CMAKE_SOURCE_DIR}/components/littlefs/
        -i ${CMAKE_SOURCE_DIR}/components/freertos/
        -i ${CMAKE_SOURCE_DIR}/components/Unity/
        -i ${CMAKE_SOURCE_DIR}/components/etl/
        -i ${CMAKE_SOURCE_DIR}/components/f103-hal/
        ${CMAKE_SOURCE_DIR}/main.cpp
        ${CMAKE_SOURCE_DIR}/components/drivers/src/
        ${CMAKE_SOURCE_DIR}/components/test/src/
    COMMENT "Running static analysis..."
    )
else()
    message(WARNING "Cppcheck not found. Static analysis will not be available for this target.")
endif()
