set(SRCS
    printf/printf.c
)

set(INC
    CMSIS/Device/ST/STM32F4xx/Include/
    CMSIS/Include/
    printf/
)

add_library(external STATIC ${SRCS})
target_include_directories(external PUBLIC ${INC})
