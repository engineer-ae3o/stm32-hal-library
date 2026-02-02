cmake_minimum_required(VERSION 3.22)
# Enable CMake support for ASM, C and C++
enable_language(C CXX ASM)
# STM32CubeMX generated symbols (macros)
set(MX_Defines_Syms
	STM32F411xE
    $<$<CONFIG:Debug>:DEBUG>
)

# STM32CubeMX generated include paths
set(MX_Include_Dirs
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/CMSIS/Device/ST/STM32F4xx/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/CMSIS/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../components/include
)

# STM32CubeMX generated application sources
set(MX_Application_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../main/main.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../startup.s
    ${CMAKE_CURRENT_SOURCE_DIR}/../../components/src/gpio.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../components/src/timer.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../components/src/uart.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../components/src/i2c.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../components/src/spi.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../components/src/pwm.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../components/src/wdt.c
)

# Link directories setup
set(MX_LINK_DIRS

)

# Project static libraries
set(MX_LINK_LIBS
    ${TOOLCHAIN_LINK_LIBRARIES}
)

# Interface library for includes and symbols
add_library(stm32cubemx INTERFACE)
target_include_directories(stm32cubemx INTERFACE ${MX_Include_Dirs})
target_compile_definitions(stm32cubemx INTERFACE ${MX_Defines_Syms})

# Add STM32CubeMX generated application sources to the project
target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${MX_Application_Src})

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE ${MX_LINK_DIRS})

# Add libraries to the project
target_link_libraries(${CMAKE_PROJECT_NAME} ${MX_LINK_LIBS})

# Add the map file to the list of files to be removed with 'clean' target
set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES ADDITIONAL_CLEAN_FILES ${CMAKE_PROJECT_NAME}.map)

# Validate that STM32CubeMX code is compatible with C standard
if((CMAKE_C_STANDARD EQUAL 90) OR (CMAKE_C_STANDARD EQUAL 99))
    message(ERROR "Generated code requires C11 or higher")
endif()
