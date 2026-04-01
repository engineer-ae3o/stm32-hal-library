cmake_minimum_required(VERSION 4.1)

set(CMAKE_C_STANDARD 23)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR ARM)

set(CMAKE_C_COMPILER   C:/Users/user/arm-gnu-toolchain/bin/arm-none-eabi-gcc.exe)
set(CMAKE_CXX_COMPILER C:/Users/user/arm-gnu-toolchain/bin/arm-none-eabi-g++.exe)
set(CMAKE_ASM_COMPILER C:/Users/user/arm-gnu-toolchain/bin/arm-none-eabi-gcc.exe)
set(CMAKE_AR           C:/Users/user/arm-gnu-toolchain/bin/arm-none-eabi-ar.exe)
set(CMAKE_OBJCOPY      C:/Users/user/arm-gnu-toolchain/bin/arm-none-eabi-objcopy.exe)
set(CMAKE_OBJDUMP      C:/Users/user/arm-gnu-toolchain/bin/arm-none-eabi-objdump.exe)
set(CMAKE_SIZE         C:/Users/user/arm-gnu-toolchain/bin/arm-none-eabi-size.exe)
set(CMAKE_GDB          C:/Users/user/arm-gnu-toolchain/bin/arm-none-eabi-gdb.exe)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

set(CMAKE_PROJECT_NAME stm32-hal)

project(${CMAKE_PROJECT_NAME})
enable_language(C ASM)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

message("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(MCU_FLAGS "-mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -Og -g3 -Wall \
               -Wpedantic -Werror -Wextra -Wshadow -Wconversion -Wwrite-strings -Wformat=2 \
               -Wsign-conversion -fno-common -fstrict-aliasing -std=gnu23")

set(LINKER_SCRIPT "${CMAKE_SOURCE_DIR}/linker.ld")

set(CMAKE_C_FLAGS "${MCU_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "-T${LINKER_SCRIPT} -Wl,-Map=${CMAKE_PROJECT_NAME}.map -Wl,--cref \
                            -Wl,--gc-sections --specs=nano.specs --specs=nosys.specs")

add_subdirectory(components/)
add_subdirectory(components/external/)

set(Defines
	STM32F411xE
    $<$<CONFIG:Debug>:DEBUG>
)

set(SRCS
    startup.s
    main/main.c
)

add_executable(${CMAKE_PROJECT_NAME} ${SRCS})

target_link_libraries(${CMAKE_PROJECT_NAME} PUBLIC
    hals
    external
    -lm
    -lc
    -lnosys
)

target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE ${Defines})

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
