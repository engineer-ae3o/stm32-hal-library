#ifndef _COMMON_H_
#define _COMMON_H_


#ifdef __cplusplus
extern "C" {
#endif


#include <stdint.h>


// Core clock and audio PLL frequencies
#define CLOCK_SPEED_HZ 100'000'000
#define AUDIO_PLL_HZ 76'800'000

// Use the HSE
#define USE_HSE

// HSI and HSE clocks
#define HSE_VALUE_MHZ 25
#define HSI_VALUE_MHZ 16

// At 100MHz, this is 100us: suitable for most use cases
#define TIMEOUT_CYCLES 10'000

// SPI DMA has the highest priority
#define SPI_DMA_NVIC_IRQ_PRIORITY 6
#define UART_DMA_NVIC_IRQ_PRIORITY 8
#define I2S_DMA_NVIC_IRQ_PRIORITY 12
#define ADC_DMA_NVIC_IRQ_PRIORITY 12

#ifdef __cplusplus
}
#endif

#endif // _COMMON_H_