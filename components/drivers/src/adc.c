#include "stm32f411xe.h"
#include "utils/common.h"
#include "drivers/adc.h"
#include "utils/err.h"

#include <stddef.h>


// ADC DMA stream settings
#define ADC_DMA_CONTROLLER (DMA2)
#define ADC_DMA_STREAM (DMA2_Stream0)
#define ADC_DMA_STREAM_NO (0)
#define ADC_DMA_CHANNEL (0)
#define ADC_DMA_IRQ_TYPE (DMA2_Stream0_IRQn)

// To save user passed callbacks
static dma_trans_done_cb_t s_dma_trans_done_cb = NULL;
static void*               s_dma_trans_cb_arg  = NULL;

static adc_injected_done_cb_t s_injected_done_cb  = NULL;
static void*                  s_injected_done_arg = NULL;


// Public API
void adc_clk_enable(bool enable) {
    if (enable) {
        RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    } else {
        RCC->APB2ENR &= ~RCC_APB2ENR_ADC1EN;
    }
    __DSB();
}

void adc_power_on(bool on) {
    if (on) {
        ADC1->CR2 |= ADC_CR2_ADON;
    } else {
        ADC1->CR2 &= ~ADC_CR2_ADON;
    }
}

void adc_configure_analog_clk(uint8_t prescaler) {
    (void)prescaler;
}


// For use with the regular group
void adc_regular_mode_init(void) {
}

void adc_regular_mode_deinit(void) {
}

hal_err_t adc_regular_mode_get_oneshot(adc_channel_t channel, uint16_t* data) {
    if (data == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    // Set the channel
    (void)channel;

    // Start the conversion
    ADC1->CR2 |= ADC_CR2_SWSTART;

    // Poll till conversion done
    uint32_t timeout = TIMEOUT_CYCLES;
    while (ADC1->SR & ADC_SR_EOC && timeout--);

    if (timeout == 0) {
        return HAL_ERR_TIMEOUT;
    }

    // Get the result
    *data = (uint16_t)ADC1->DR;

    return HAL_OK;
}

hal_err_t adc_regular_mode_start_conv(const adc_channel_t* channels, size_t num_of_channels, dma_trans_done_cb_t cb, void* arg) {
    (void)channels;
    (void)num_of_channels;

    if (cb) {
        s_dma_trans_done_cb = cb;
        s_dma_trans_cb_arg  = arg;
    }

    return HAL_OK;
}


// For use with the injected group
void adc_injected_mode_init(void) {
}

void adc_injected_mode_deinit(void) {
}

hal_err_t adc_injected_mode_start_conv(const adc_channel_t* channels, size_t num_of_channels, adc_injected_done_cb_t cb, void* arg) {
    if (channels == NULL || num_of_channels > MAX_INJECTED_CHANNELS || num_of_channels == 0) {
        return HAL_ERR_INVALID_ARG;
    }

    // Single conversion mode
    if (num_of_channels == 1) {
        ADC1->CR2 |= ADC_CR2_JSWSTART;
    }

    if (cb) {
        s_injected_done_cb  = cb;
        s_injected_done_arg = arg;
    }

    return HAL_OK;
}

hal_err_t adc_injected_mode_get_result(uint16_t* buffer, size_t num_of_channels) {
    if (buffer == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    if (ADC1->SR & ADC_SR_JEOC) {
        return HAL_ERR_NOT_FOUND;
    }

    switch (num_of_channels) {
        case 1:
            buffer[0] = (uint16_t)ADC1->JDR1;
            break;
        case 2:
            buffer[0] = (uint16_t)ADC1->JDR1;
            buffer[1] = (uint16_t)ADC1->JDR2;
            break;
        case 3:
            buffer[0] = (uint16_t)ADC1->JDR1;
            buffer[1] = (uint16_t)ADC1->JDR2;
            buffer[2] = (uint16_t)ADC1->JDR3;
            break;
        case 4:
            buffer[0] = (uint16_t)ADC1->JDR1;
            buffer[1] = (uint16_t)ADC1->JDR2;
            buffer[2] = (uint16_t)ADC1->JDR3;
            buffer[3] = (uint16_t)ADC1->JDR4;
            break;
        default:
            return HAL_ERR_INVALID_ARG;
    }

    return HAL_OK;
}


// Interrupt handlers
void ADC_IRQHandler(void) {
    // Invoke user cb once the ADC sampling on the injected group is complete
    if (s_injected_done_cb) {
        s_injected_done_cb(s_injected_done_arg);
        s_injected_done_cb  = NULL;
        s_injected_done_arg = NULL;
    }
}

void DMA2_Stream0_IRQHandler(void) {
    // Invoke user cb once the ADC sampling and DMA transfer on the regular group is complete
    if (s_dma_trans_done_cb) {
        s_dma_trans_done_cb(s_dma_trans_cb_arg, HAL_OK);
        s_dma_trans_done_cb = NULL;
        s_dma_trans_cb_arg  = NULL;
    }
}
