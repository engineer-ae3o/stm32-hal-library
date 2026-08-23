#include "stm32f411xe.h"
#include "utils/common.h"
#include "drivers/adc.h"
#include "utils/tick.h"
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

static adc_awdg_isr_t s_analog_wdg_cb  = NULL;
static void*          s_analog_wdg_arg = NULL;


// Helper macro
#define POLL_TILL_CONV_DONE()                                                                                                                        \
    do {                                                                                                                                             \
        uint32_t timeout_cycles = TIMEOUT_CYCLES;                                                                                                    \
        while (!(ADC1->SR & ADC_SR_EOC) && --timeout_cycles);                                                                                        \
        if (timeout_cycles == 0) {                                                                                                                   \
            return HAL_ERR_TIMEOUT;                                                                                                                  \
        }                                                                                                                                            \
    } while (0)


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

void adc_enable_nvic_irq(void) {
    NVIC_SetPriority(ADC_IRQn, ADC_DMA_NVIC_IRQ_PRIORITY);
    NVIC_EnableIRQ(ADC_IRQn);
}

void adc_disable_nvic_irq(void) {
    NVIC_DisableIRQ(ADC_IRQn);
}

void adc_configure_analog_clk(uint8_t prescaler) {
    // TODO: COnfigure the prescaler
    (void)prescaler;
}

void adc_configure_sample_time(uint8_t cycles) {
    // TODO: COnfigure the sample time
    (void)cycles;
}

// For use with the regular group
hal_err_t adc_regular_mode_get_oneshot(adc_ext_channel_t channel, uint16_t* data) {
    if (data == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    // Set the channel
    ADC1->SQR1 |= channel;
    ADC1->SQR1 |= channel;

    // Start the conversion and disable continuous conversion mode since we are only sampling a single channel
    ADC1->CR2 &= ~ADC_CR2_CONT;
    ADC1->CR2 |= ADC_CR2_SWSTART;

    // Poll till the conversion is complete
    POLL_TILL_CONV_DONE();

    // Get the final result
    *data = (uint16_t)ADC1->DR;
    return HAL_OK;
}

hal_err_t adc_regular_mode_start_conv(const adc_ext_channel_t* channels, size_t num_of_channels, dma_trans_done_cb_t cb, void* arg) {
    if (channels == NULL || num_of_channels == 0 || num_of_channels > MAX_INJECTED_CHANNELS) {
        return HAL_ERR_INVALID_ARG;
    }

    if (cb) {
        s_dma_trans_done_cb = cb;
        s_dma_trans_cb_arg  = arg;
    }

    return HAL_OK;
}

// For use with the injected group
hal_err_t adc_injected_mode_start_conv(const adc_ext_channel_t* channels, size_t num_of_channels, adc_injected_done_cb_t cb, void* arg) {
    if (channels == NULL || num_of_channels == 0 || num_of_channels > MAX_INJECTED_CHANNELS) {
        return HAL_ERR_INVALID_ARG;
    }

    // Save user callback
    if (cb) {
        s_injected_done_cb  = cb;
        s_injected_done_arg = arg;
    }

    // Enable interrupts for the injected mode and set the number of channels in the [0:1] bit positions of the JSQR register
    ADC1->CR1 |= ADC_CR1_JEOCIE;
    ADC1->JSQR |= (num_of_channels & 0b11);

    if (num_of_channels == 1) {
        // Single conversion mode
        ADC1->CR2 |= ADC_CR2_JSWSTART;
    } else {
        // Enable scan mode if we have more than one channel
        ADC1->CR2 |= ADC_CR1_SCAN;
    }

    return HAL_OK;
}

hal_err_t adc_injected_mode_get_result(uint16_t* buffer, size_t num_of_channels) {
    if (buffer == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    // If the JEOC bit is not set, conversion is not yet done
    if (!(ADC1->SR & ADC_SR_JEOC)) {
        return HAL_ERR_NOT_DONE;
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

// For use with the internal channels
// The internal channels
typedef enum : uint8_t {
    ADC_CHANNEL_TEMP = 16, // Temperature sensor
    ADC_CHANNEL_VREF = 17, // V_refint
    ADC_CHANNEL_VBAT = 18, // V_bat
} adc_int_channel_t;

hal_err_t get_v_bat(uint16_t* v_bat) {
    if (v_bat == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    // Set the channel
    ADC1->SQR1 |= ADC_CHANNEL_VBAT;
    ADC1->SQR1 |= ADC_CHANNEL_VBAT;

    // Switch to V_bat so the ADC can measure it
    ADC->CCR |= ADC_CCR_VBATE;

    // Start the conversion
    ADC1->CR2 &= ~ADC_CR2_CONT;
    ADC1->CR2 |= ADC_CR2_SWSTART;

    // Poll till the conversion is complete
    POLL_TILL_CONV_DONE();

    // Get V_bat: The ADC only ever sees (V_bat / VBAT_DIVIDER_RATIO) so
    // we have to multiply by VBAT_DIVIDER_RATIO to get the actual V_bat
    *v_bat = (uint16_t)(ADC1->DR * VBAT_DIVIDER_RATIO);

    // Disconnect V_bat once done
    ADC->CCR &= ~ADC_CCR_VBATE;
    return HAL_OK;
}

hal_err_t get_temperature(uint16_t* temperature) {
    if (temperature == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    // Set the channel
    ADC1->SQR1 |= ADC_CHANNEL_TEMP;
    ADC1->SQR1 |= ADC_CHANNEL_TEMP;

    // Set a sampling time greater than MIN_SAMPLING_TIME_US
    // TODO: Set the sampling time

    // Wake the temperature sensor if it's not already powered on
    if (!(ADC->CCR & ADC_CCR_TSVREFE)) {
        ADC->CCR |= ADC_CCR_TSVREFE;
        delay_us(TEMP_SENSOR_STARUP_TIME_US);
    }

    // Start the conversion
    ADC1->CR2 &= ~ADC_CR2_CONT;
    ADC1->CR2 |= ADC_CR2_SWSTART;

    // Poll till the conversion is complete
    POLL_TILL_CONV_DONE();

    // Calculate the temperature
    *temperature = (uint16_t)(((float)ADC1->DR - TEMP_SENSOR_VSENSE_AT_25C) / TEMP_SENSOR_AVERAGE_SLOPE) + 25;
    return HAL_OK;
}

hal_err_t get_v_ref_internal(uint16_t* v_ref_int) {
    if (v_ref_int == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    // Set the channel
    ADC1->SQR1 |= ADC_CHANNEL_VREF;
    ADC1->SQR1 |= ADC_CHANNEL_VREF;

    // The TSVREFE bit also enables measurement of VREFINT
    if (!(ADC->CCR & ADC_CCR_TSVREFE)) {
        ADC->CCR |= ADC_CCR_TSVREFE;
        delay_us(TEMP_SENSOR_STARUP_TIME_US);
    }

    // Start the conversion
    ADC1->CR2 &= ~ADC_CR2_CONT;
    ADC1->CR2 |= ADC_CR2_SWSTART;

    // Poll till the conversion is complete
    POLL_TILL_CONV_DONE();

    // Get V_ref
    *v_ref_int = (uint16_t)ADC1->DR;
    return HAL_OK;
}

// For use of control of the analog watchdog
hal_err_t adc_analog_wdg_start(uint16_t min_volt, uint16_t max_volt, bool monitor_regular, bool monitor_injected, adc_awdg_isr_t cb, void* arg) {
    if ((cb == NULL) || (min_volt >= max_volt)) {
        // The use of interrupts is effectively mandatory as that's the
        // only way for the watchdog to inform the caller of a violation
        return HAL_ERR_INVALID_ARG;
    }

    // Clear the AWDSGL bit since the monitoring is not only on an single channel
    // and clear all state before modifying any of the bits in the register(s)
    ADC1->CR1 &= ~(ADC_CR1_AWDSGL | ADC_CR1_AWDEN | ADC_CR1_JAWDEN | ADC_CR1_AWDIE);

    if (monitor_regular && monitor_injected) {
        // Enable monitoring on all channels
        ADC1->CR1 |= (ADC_CR1_AWDEN | ADC_CR1_JAWDEN);
    } else if (monitor_injected) {
        // Enable monitoring on only the injected channels
        ADC1->CR1 |= ADC_CR1_JAWDEN;
    } else if (monitor_regular) {
        // Enable monitoring on only the regular channels
        ADC1->CR1 |= ADC_CR1_AWDEN;
    } else {
        return HAL_ERR_INVALID_ARG;
    }

    // Set the voltage thresholds
    ADC1->HTR = max_volt & 0xFFFU; // Only the lower 12 bits are used
    ADC1->LTR = min_volt & 0xFFFU; // Only the lower 12 bits are used

    // Save the user passed callback
    s_analog_wdg_cb  = cb;
    s_analog_wdg_arg = arg;

    // Enable the analog watchdog interrupt
    ADC1->CR1 |= ADC_CR1_AWDIE;

    // Enable the ADC interrupt with the NVIC if it hadn't been enabled before
    if (NVIC_GetEnableIRQ(ADC_IRQn) == 0) {
        adc_enable_nvic_irq();
    }

    return HAL_OK;
}

void adc_analog_wdg_stop(void) {
    // Disable the analog watchdog interrupt and disable monitoring on all channels
    ADC1->CR1 &= ~(ADC_CR1_AWDIE | ADC_CR1_JAWDEN | ADC_CR1_AWDEN | ADC_CR1_AWDSGL);

    // Clear the voltage thresholds
    ADC1->HTR = 0;
    ADC1->LTR = 0;

    // Clear the user passed callback
    s_analog_wdg_cb  = NULL;
    s_analog_wdg_arg = NULL;
}


// Interrupt handlers
void ADC_IRQHandler(void) {
    // Check if end of conversion for the injected mode is reached
    if (ADC1->SR & ADC_SR_JEOC) {
        // Invoke user callback once the ADC sampling on the injected group is complete
        if (s_injected_done_cb) {
            s_injected_done_cb(s_injected_done_arg);
            // Clear user passed callbcak simce this is a one-off event
            s_injected_done_cb  = NULL;
            s_injected_done_arg = NULL;
        }
    }

    // Check if the AWD bit is set. This means the analog watchdog has fired an interrupt
    if (ADC1->SR & ADC_SR_AWD) {
        if (s_analog_wdg_cb) {
            // Invoke the user callback since the analog watchdog has fired an interrupt
            s_analog_wdg_cb(s_analog_wdg_arg);
            // The callback isn't cleared since this is not a one-off event. To clear, call adc_analog_wdg_stop()
        }
    }
}

void DMA2_Stream0_IRQHandler(void) {
    // Invoke user callback once the ADC sampling and DMA transfer on the regular group is complete
    if (s_dma_trans_done_cb) {
        s_dma_trans_done_cb(s_dma_trans_cb_arg, HAL_OK);
        s_dma_trans_done_cb = NULL;
        s_dma_trans_cb_arg  = NULL;
    }
}
