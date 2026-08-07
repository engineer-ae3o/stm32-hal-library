cmake_minimum_required(VERSION 4.0)

# Add default toolchain file if one not added yet
if(NOT CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/gcc_toolchain.cmake)
endif()

set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)


# Create project
set(CMAKE_PROJECT_NAME f411-hal-library)

project(${CMAKE_PROJECT_NAME})
enable_language(C CXX ASM)

set(EXE_NAME f411-hal)
set(LIB_ELF_NAME "${EXE_NAME}") # CMake already appends .a automatically
set(EXE_ASM_NAME "${EXE_NAME}.s")
set(EXE_ELF_NAME "${EXE_NAME}.elf")
set(EXE_BIN_NAME "${EXE_NAME}.bin")
set(EXE_MAP_NAME "${EXE_NAME}.map")


set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)


# Build flags for the different build modes
set(CMAKE_C_FLAGS_DEBUG "-O0 -g3 -ggdb -DDEBUG")
set(CMAKE_C_FLAGS_RELEASE "-O2 -Wl,-s -DNDEBUG")

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG}")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE}")


# Build flags
set(CMAKE_C_FLAGS "-mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 \
                   -Wall -Wstrict-aliasing=2 -Wduplicated-branches -Wduplicated-cond -Werror \
                   -Wlogical-op -Wextra -Wshadow -Wno-conversion -Wwrite-strings -Wformat=2 \
                   -fstrict-overflow -fstrict-aliasing -Wdouble-promotion -fstack-usage \
                   -Wswitch-enum -Wnull-dereference -Wcast-align -Wno-sign-conversion")

set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -fno-exceptions -fno-rtti")


# Set linker flags
set(LINKER_SCRIPT "${CMAKE_SOURCE_DIR}/stm32411_linker.ld")
set(CMAKE_EXE_LINKER_FLAGS "-T${LINKER_SCRIPT} -Wl,-Map=${EXE_MAP_NAME} -Wl,--cref \
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


if (BUILD_TESTS)

# Process all components
add_subdirectory(components/test/)
add_subdirectory(components/extra/)
add_subdirectory(components/drivers/)
add_subdirectory(components/third-party/RTT/)
add_subdirectory(components/third-party/Unity/)
add_subdirectory(components/third-party/o1heap/)

# Build an executable only if building with tests. 
add_executable(${EXE_ELF_NAME} test_main.cpp stm32f411_startup.s)
target_link_libraries(${EXE_ELF_NAME} PUBLIC tests unity drivers extra rtt o1heap)
target_compile_definitions(${EXE_ELF_NAME} PUBLIC STM32F411xE)
target_link_options(${EXE_ELF_NAME} PRIVATE -Wl,--no-warn-rwx-segments)

# The tests are C++
set_target_properties(${EXE_ELF_NAME} PROPERTIES
    CXX_STANDARD 23
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS ON
)

# Get the .bin file, the disassembly as well as print the space occupied by the executable
add_custom_command(
    TARGET ${EXE_ELF_NAME} POST_BUILD
    COMMENT "Building tests..."
    COMMAND ${CMAKE_OBJCOPY} -O binary ${EXE_ELF_NAME} ${EXE_BIN_NAME}
    COMMAND ${CMAKE_OBJDUMP} -D -S ${EXE_ELF_NAME} > ${EXE_ASM_NAME}
    COMMAND ${CMAKE_SIZE} ${EXE_ELF_NAME}
)

else()

# Leave the tests out
add_subdirectory(components/extra/)
add_subdirectory(components/drivers/)
add_subdirectory(components/third-party/RTT/)
add_subdirectory(components/third-party/Unity/)
add_subdirectory(components/third-party/o1heap/)

# Only produce a static library.
add_library(${LIB_ELF_NAME} STATIC stm32f411_startup.s)
target_compile_definitions(${LIB_ELF_NAME} PUBLIC STM32F411xE)

# The core library is C
set_target_properties(${LIB_ELF_NAME} PROPERTIES
    C_STANDARD 23
    C_STANDARD_REQUIRED ON
    C_EXTENSIONS ON
)

# No point in getting the disassembly of a static library. Just produce an archive of the relevant components
add_custom_command(
    TARGET ${LIB_ELF_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E echo "CREATE $<TARGET_FILE:${LIB_ELF_NAME}>.tmp.a" > combine.mri
    COMMAND ${CMAKE_COMMAND} -E echo "ADDLIB $<TARGET_FILE:${LIB_ELF_NAME}>" >> combine.mri
    COMMAND ${CMAKE_COMMAND} -E echo "ADDLIB $<TARGET_FILE:drivers>" >> combine.mri
    COMMAND ${CMAKE_COMMAND} -E echo "ADDLIB $<TARGET_FILE:extra>" >> combine.mri
    COMMAND ${CMAKE_COMMAND} -E echo "ADDLIB $<TARGET_FILE:unity>" >> combine.mri
    COMMAND ${CMAKE_COMMAND} -E echo "ADDLIB $<TARGET_FILE:rtt>" >> combine.mri
    COMMAND ${CMAKE_COMMAND} -E echo "ADDLIB $<TARGET_FILE:o1heap>" >> combine.mri
    COMMAND ${CMAKE_COMMAND} -E echo "SAVE" >> combine.mri
    COMMAND ${CMAKE_COMMAND} -E echo "END" >> combine.mri
    COMMAND ${CMAKE_AR} -M < combine.mri
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${LIB_ELF_NAME}>.tmp.a $<TARGET_FILE:${LIB_ELF_NAME}>
    COMMAND ${CMAKE_COMMAND} -E remove $<TARGET_FILE:${LIB_ELF_NAME}>.tmp.a combine.mri
    COMMENT "Bundling static dependencies into lib${LIB_ELF_NAME}.a..."
    COMMAND ${CMAKE_SIZE} "lib${LIB_ELF_NAME}.a"
    VERBATIM
)

endif()

# Static analysis with cppcheck
find_program(CPPCHECK cppcheck)
find_program(CLANG_FORMAT clang-format)

if (CPPCHECK)

add_custom_target(cppcheck
    COMMAND mkdir -p ${CMAKE_BINARY_DIR}/.cppcheck
    COMMAND ${CPPCHECK} --enable=all
        --inline-suppr
        --cppcheck-build-dir=${CMAKE_BINARY_DIR}/.cppcheck
        --suppressions-list=${CMAKE_SOURCE_DIR}/cppcheck_suppressions.txt
        --error-exitcode=1
        --check-level=exhaustive
        -D__GNUC__
        --project=${CMAKE_BINARY_DIR}/compile_commands.json
        --file-filter=${CMAKE_SOURCE_DIR}/*.cpp
        --file-filter=${CMAKE_SOURCE_DIR}/components/drivers/**
        --file-filter=${CMAKE_SOURCE_DIR}/components/test/**
        --output-file=${CMAKE_BINARY_DIR}/cppcheck_report.txt
        --checkers-report=${CMAKE_BINARY_DIR}/cppcheck_checks.txt
    COMMENT "Running static analysis..."
)

else()

message(WARNING "Cppcheck not found. Static analysis will not be available for this target.")

endif()

if(CLANG_FORMAT)

add_custom_target(format
    COMMAND git ls-files -- '*.c' '*.h' '*.cpp' '*.hpp' '*.cc' '*.cxx' | xargs -r ${CLANG_FORMAT} -i
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    COMMENT "Formatting version controlled files...."
)

else()

message(WARNING "clang-format not found. Formatting will not be availabe for this target.")

endif()
