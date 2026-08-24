#include "stm32f411xe.h"
#include "utils/common.h"
#include "drivers/adc.h"
#include "utils/tick.h"
#include "utils/err.h"

#include <stddef.h>
#include <stdint.h>


// ADC DMA stream settings
#define ADC_DMA_CONTROLLER (DMA2)
#define ADC_DMA_STREAM (DMA2_Stream0)
#define ADC_DMA_STREAM_NO (0)
#define ADC_DMA_CHANNEL (0)
#define ADC_DMA_IRQ_TYPE (DMA2_Stream0_IRQn)


// To save user passed callbacks
// For ADC1
static dma_trans_done_cb_t s_dma_trans_done_cb[NUM_OF_ADC_CONTROLLERS] = {};
static void*               s_dma_trans_cb_arg[NUM_OF_ADC_CONTROLLERS]  = {};

static adc_callback_t s_injected_done_cb[NUM_OF_ADC_CONTROLLERS]  = {};
static void*          s_injected_done_arg[NUM_OF_ADC_CONTROLLERS] = {};

static adc_callback_t s_analog_wdg_cb[NUM_OF_ADC_CONTROLLERS]  = {};
static void*          s_analog_wdg_arg[NUM_OF_ADC_CONTROLLERS] = {};


// Helper macro
#define POLL_TILL_CONV_DONE()                                                                                                                        \
    do {                                                                                                                                             \
        uint32_t timeout_cycles = TIMEOUT_CYCLES;                                                                                                    \
        while (!(handle->SR & ADC_SR_EOC) && --timeout_cycles);                                                                                      \
        if (timeout_cycles == 0) {                                                                                                                   \
            return HAL_ERR_TIMEOUT;                                                                                                                  \
        }                                                                                                                                            \
    } while (0)


static __attribute__((__always_inline__)) inline uint8_t get_index(const ADC_TypeDef* handle) {
    if (handle == ADC1) {
        return 0U;
    }
    // Uncomment if your mcu supports ADC2 and ADC3
    /* else if (handle == ADC2) {
        return 1U;
    } else if (handle == ADC3) {
        return 2U;
    } */
    else {
        return 0xFFU;
    }
}


// Public API
void adc_clk_enable(ADC_TypeDef* handle, bool enable) {
    if (enable) {
        if (handle == ADC1) {
            RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
        }
    } else {
        if (handle == ADC1) {
            RCC->APB2ENR &= ~RCC_APB2ENR_ADC1EN;
        }
    }
    __DSB();
}

void adc_power_on(ADC_TypeDef* handle, bool on) {
    if (on) {
        handle->CR2 |= ADC_CR2_ADON;
    } else {
        handle->CR2 &= ~ADC_CR2_ADON;
    }
}

void adc_enable_nvic_irq(void) {
    NVIC_SetPriority(ADC_IRQn, ADC_DMA_NVIC_IRQ_PRIORITY);
    NVIC_EnableIRQ(ADC_IRQn);
}

void adc_disable_nvic_irq(void) {
    NVIC_DisableIRQ(ADC_IRQn);
}

hal_err_t adc_configure_analog_clk(ADC_TypeDef* handle, const adc_clk_config_t* config) {
    if (config == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    // Set the ADC clock prescaler
    ADC->CCR &= ~ADC_CCR_ADCPRE;
    ADC->CCR |= ((uint32_t)config->clk_prescaler << ADC_CCR_ADCPRE_Pos);

    // Set the ADC resolution
    handle->CR1 &= ~ADC_CR1_RES;
    handle->CR1 |= ((uint32_t)config->resolution << ADC_CR1_RES_Pos);

    // Set the sampling cycles for all channels
    // Clear all the bit positions first
    handle->SMPR1 &= ~(ADC_SMPR1_SMP10 | ADC_SMPR1_SMP11 | ADC_SMPR1_SMP12 | ADC_SMPR1_SMP13 | ADC_SMPR1_SMP14 | ADC_SMPR1_SMP15 | ADC_SMPR1_SMP16 |
                       ADC_SMPR1_SMP17 | ADC_SMPR1_SMP18);
    handle->SMPR2 &= ~(ADC_SMPR2_SMP0 | ADC_SMPR2_SMP1 | ADC_SMPR2_SMP2 | ADC_SMPR2_SMP3 | ADC_SMPR2_SMP4 | ADC_SMPR2_SMP5 | ADC_SMPR2_SMP6 |
                       ADC_SMPR2_SMP7 | ADC_SMPR2_SMP8 | ADC_SMPR2_SMP9);

    const uint32_t time = (uint32_t)config->sampling_cycles;

    // Apply the sample cycles to all channels
    handle->SMPR1 |= ((time << ADC_SMPR1_SMP10_Pos) | (time << ADC_SMPR1_SMP11_Pos) | (time << ADC_SMPR1_SMP12_Pos) | (time << ADC_SMPR1_SMP13_Pos) |
                      (time << ADC_SMPR1_SMP14_Pos) | (time << ADC_SMPR1_SMP15_Pos) | (time << ADC_SMPR1_SMP16_Pos) | (time << ADC_SMPR1_SMP17_Pos) |
                      (time << ADC_SMPR1_SMP18_Pos));

    handle->SMPR2 |= ((time << ADC_SMPR2_SMP0_Pos) | (time << ADC_SMPR2_SMP1_Pos) | (time << ADC_SMPR2_SMP2_Pos) | (time << ADC_SMPR2_SMP3_Pos) |
                      (time << ADC_SMPR2_SMP4_Pos) | (time << ADC_SMPR2_SMP5_Pos) | (time << ADC_SMPR2_SMP6_Pos) | (time << ADC_SMPR2_SMP7_Pos) |
                      (time << ADC_SMPR2_SMP8_Pos) | (time << ADC_SMPR2_SMP9_Pos));

    return HAL_OK;
}


// For use with the regular group
hal_err_t adc_regular_group_get_oneshot(ADC_TypeDef* handle, adc_channels_t channel, uint16_t* data) {
    if (data == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    // Set the number of channels to be converted
    // An L[3:0] value of 0b0000 means 1 channel/conversion, avalue of 0b0011 means 4
    // conversions etc. Which is why 0b0000 is used despite the sampling being for 1 channel
    handle->SQR1 &= ~ADC_SQR1_L;
    handle->SQR1 |= 0b0000U << ADC_SQR1_L_Pos;

    // Set the channel
    handle->SQR3 &= ~ADC_SQR3_SQ1;
    handle->SQR3 |= (uint32_t)channel << ADC_SQR3_SQ1_Pos;

    // Disable continuous conversion and scan mode since we are sampling only one channel
    // Then clear the EXTEN bit since the triggering is by software
    handle->CR1 &= ~ADC_CR1_SCAN;
    handle->CR2 &= ~(ADC_CR2_CONT | ADC_CR2_EXTEN);

    // Start the conversion
    handle->CR2 |= ADC_CR2_SWSTART;

    // Poll till the conversion is complete
    POLL_TILL_CONV_DONE();

    // Get the final result
    *data = (uint16_t)handle->DR;

    return HAL_OK;
}

hal_err_t adc_regular_group_start_conv(ADC_TypeDef* handle, const adc_channels_t* channels, size_t size, dma_trans_done_cb_t cb, void* arg) {
    if (channels == NULL || size == 0 || size > MAX_REGULAR_CHANNELS) {
        return HAL_ERR_INVALID_ARG;
    }

    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU) {
        return HAL_ERR_INVALID_ARG;
    }

    (void)handle;

    if (cb) {
        s_dma_trans_done_cb[idx] = cb;
        s_dma_trans_cb_arg[idx]  = arg;
    }

    return HAL_OK;
}


// For use with the injected group
hal_err_t adc_injected_group_start_conv(ADC_TypeDef* handle, const adc_channels_t* channels, size_t size, adc_callback_t cb, void* arg) {
    if (channels == NULL || size == 0 || size > MAX_INJECTED_CHANNELS) {
        return HAL_ERR_INVALID_ARG;
    }

    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU) {
        return HAL_ERR_INVALID_ARG;
    }

    // Save the user passed callback
    if (cb) {
        s_injected_done_cb[idx]  = cb;
        s_injected_done_arg[idx] = arg;
    }

    // Enable interrupts for the injected group when conversion is complete
    handle->CR1 |= ADC_CR1_JEOCIE;

    // Set the number of channels/conversions in the JL bit positions of the
    // JSQR register. The JL bit positions are zero indexed. That is, 1 channel
    // means a JL value of 0b00, 3 channels means a JL value of 0b10 etc.
    handle->JSQR &= ~ADC_JSQR_JL;
    handle->JSQR |= (((size - 1) & 0x3U) << ADC_JSQR_JL_Pos);

    // TODO: Set the channel sequence

    // Clear all offsets registers since not used
    handle->JOFR1 &= ~ADC_JOFR1_JOFFSET1;
    handle->JOFR2 &= ~ADC_JOFR2_JOFFSET2;
    handle->JOFR3 &= ~ADC_JOFR3_JOFFSET3;
    handle->JOFR4 &= ~ADC_JOFR4_JOFFSET4;

    // Enable scan mode if we have more than one channel, but disable otherwise
    if (size > 1) {
        handle->CR1 |= ADC_CR1_SCAN;
    } else {
        handle->CR1 &= ~ADC_CR1_SCAN;
    }

    // Clear the JEXTEN bit since the triggering is from software. Also,
    // the JAUTO bit is cleared so the ADC interrupts any ongoing regular
    // group conversion to perform this injected group conversion.
    handle->CR2 &= ~ADC_CR2_JEXTEN;
    handle->CR1 &= ~ADC_CR1_JAUTO;

    // Start the conversion and return. The JEOC interrupt will fire
    // when the ADC controller is finished with the conversion(s).
    handle->CR2 |= ADC_CR2_JSWSTART;

    return HAL_OK;
}

hal_err_t adc_injected_group_get_result(ADC_TypeDef* handle, uint16_t* buffer, size_t size) {
    if (buffer == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU) {
        return HAL_ERR_INVALID_ARG;
    }

    // If the JEOC bit is not set, conversion is not yet done
    if (!(handle->SR & ADC_SR_JEOC)) {
        return HAL_ERR_NOT_DONE;
    }

    // Read the injected group data registers. Can switch over them since only 4 data registers
    switch (size) {
        case 1:
            buffer[0] = (uint16_t)handle->JDR1;
            break;
        case 2:
            buffer[0] = (uint16_t)handle->JDR1;
            buffer[1] = (uint16_t)handle->JDR2;
            break;
        case 3:
            buffer[0] = (uint16_t)handle->JDR1;
            buffer[1] = (uint16_t)handle->JDR2;
            buffer[2] = (uint16_t)handle->JDR3;
            break;
        case 4:
            buffer[0] = (uint16_t)handle->JDR1;
            buffer[1] = (uint16_t)handle->JDR2;
            buffer[2] = (uint16_t)handle->JDR3;
            buffer[3] = (uint16_t)handle->JDR4;
            break;
        default:
            return HAL_ERR_INVALID_ARG;
    }

    return HAL_OK;
}


// For use with the internal channels
hal_err_t adc_get_v_bat(ADC_TypeDef* handle, uint16_t* raw_v_bat) {
    if (raw_v_bat == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    // Set the number of channels to be converted
    handle->SQR1 &= ~ADC_SQR1_L;
    handle->SQR1 |= 0b0000U << ADC_SQR1_L_Pos;

    // Set the channel
    handle->SQR3 &= ~ADC_SQR3_SQ1;
    handle->SQR3 |= ADC_CHANNEL_VBAT << ADC_SQR3_SQ1_Pos;

    // Switch to V_bat so the ADC can measure it
    ADC->CCR |= ADC_CCR_VBATE;

    // Disable continuous conversion and scan mode since we are sampling only one channel
    // Then clear the EXTEN bit since the triggering is by software
    handle->CR1 &= ~ADC_CR1_SCAN;
    handle->CR2 &= ~(ADC_CR2_CONT | ADC_CR2_EXTEN);

    // Start the conversion
    handle->CR2 |= ADC_CR2_SWSTART;

    // Poll till the conversion is complete
    POLL_TILL_CONV_DONE();

    // Get V_bat: The ADC only ever sees (V_bat / VBAT_DIVIDER_RATIO) so we
    // have to multiply by VBAT_DIVIDER_RATIO to get the actual V_bat. This
    // is done by the hardware because V_bat could exceed the operating voltage
    // range of the ADC, so it is divided so the ADC can function correctly
    *raw_v_bat = (uint16_t)(handle->DR * VBAT_DIVIDER_RATIO);

    // Disconnect V_bat once done
    ADC->CCR &= ~ADC_CCR_VBATE;

    return HAL_OK;
}

hal_err_t adc_get_temperature(ADC_TypeDef* handle, uint16_t* raw_temp) {
    if (raw_temp == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    // Set the number of channels to be converted
    handle->SQR1 &= ~ADC_SQR1_L;
    handle->SQR1 |= 0b0000U << ADC_SQR1_L_Pos;

    // Set the channel
    handle->SQR3 &= ~ADC_SQR3_SQ1;
    handle->SQR3 |= ADC_CHANNEL_TEMP << ADC_SQR3_SQ1_Pos;

    // Wake the raw_temp sensor if it's not already powered on
    if (!(ADC->CCR & ADC_CCR_TSVREFE)) {
        ADC->CCR |= ADC_CCR_TSVREFE;
        delay_us(TEMP_SENSOR_STARUP_TIME_US);
    }

    // Disable continuous conversion and scan mode since we are sampling only one channel
    // Then clear the EXTEN bit since the triggering is by software
    handle->CR1 &= ~ADC_CR1_SCAN;
    handle->CR2 &= ~(ADC_CR2_CONT | ADC_CR2_EXTEN);

    // Start the conversion
    handle->CR2 |= ADC_CR2_SWSTART;

    // Poll till the conversion is complete
    POLL_TILL_CONV_DONE();

    // Get the raw value
    *raw_temp = (uint16_t)handle->DR;

    return HAL_OK;
}

hal_err_t adc_get_v_ref_internal(ADC_TypeDef* handle, uint16_t* raw_v_ref_int) {
    if (raw_v_ref_int == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    // Set the number of channels to be converted
    handle->SQR1 &= ~ADC_SQR1_L;
    handle->SQR1 |= 0b0000U << ADC_SQR1_L_Pos;

    // Set the channel
    handle->SQR3 &= ~ADC_SQR3_SQ1;
    handle->SQR3 |= ADC_CHANNEL_VREF << ADC_SQR3_SQ1_Pos;

    // The TSVREFE bit also enables measurement of VREFINT
    if (!(ADC->CCR & ADC_CCR_TSVREFE)) {
        ADC->CCR |= ADC_CCR_TSVREFE;
        delay_us(TEMP_SENSOR_STARUP_TIME_US);
    }

    // Disable continuous conversion and scan mode since we are sampling only one channel
    // Then clear the EXTEN bit since the triggering is by software
    handle->CR1 &= ~ADC_CR1_SCAN;
    handle->CR2 &= ~(ADC_CR2_CONT | ADC_CR2_EXTEN);

    // Start the conversion
    handle->CR2 |= ADC_CR2_SWSTART;

    // Poll till the conversion is complete
    POLL_TILL_CONV_DONE();

    // Get V_ref
    *raw_v_ref_int = (uint16_t)handle->DR;

    return HAL_OK;
}


// For use of control of the analog watchdog
hal_err_t adc_analog_wdg_start(ADC_TypeDef* handle, uint16_t min, uint16_t max, bool regular, bool injected, adc_callback_t cb, void* arg) {
    if ((cb == NULL) || (min >= max)) {
        // The use of interrupts is effectively mandatory as that's the
        // only way for the watchdog to inform the caller of a violation
        return HAL_ERR_INVALID_ARG;
    }

    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU) {
        return HAL_ERR_INVALID_ARG;
    }

    // Clear the AWDSGL bit since the monitoring is not only on an single channel
    // and clear all state before modifying any of the bits in the register(s)
    handle->CR1 &= ~(ADC_CR1_AWDSGL | ADC_CR1_AWDEN | ADC_CR1_JAWDEN | ADC_CR1_AWDIE);

    if (regular && injected) {
        // Enable monitoring on all channels
        handle->CR1 |= (ADC_CR1_AWDEN | ADC_CR1_JAWDEN);
    } else if (injected) {
        // Enable monitoring on only the injected channels
        handle->CR1 |= ADC_CR1_JAWDEN;
    } else if (regular) {
        // Enable monitoring on only the regular channels
        handle->CR1 |= ADC_CR1_AWDEN;
    } else {
        return HAL_ERR_INVALID_ARG;
    }

    // Set the voltage thresholds
    handle->HTR = max & 0xFFFU; // Only the lower 12 bits are used
    handle->LTR = min & 0xFFFU; // Only the lower 12 bits are used

    // Save the user passed callback
    s_analog_wdg_cb[idx]  = cb;
    s_analog_wdg_arg[idx] = arg;

    // Enable the analog watchdog interrupt
    handle->CR1 |= ADC_CR1_AWDIE;

    return HAL_OK;
}

hal_err_t adc_analog_wdg_stop(ADC_TypeDef* handle) {
    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU) {
        return HAL_ERR_INVALID_ARG;
    }

    // Disable the analog watchdog interrupt and disable monitoring on all channels
    handle->CR1 &= ~(ADC_CR1_AWDIE | ADC_CR1_JAWDEN | ADC_CR1_AWDEN | ADC_CR1_AWDSGL);

    // Clear the voltage thresholds
    handle->HTR = 0;
    handle->LTR = 0;

    // Clear the user passed callback
    s_analog_wdg_cb[idx]  = NULL;
    s_analog_wdg_arg[idx] = NULL;

    return HAL_OK;
}


// Interrupt handlers
void ADC_IRQHandler(void) {
    // There's only ADC1 on the STM32F411. Add the right number of ISRs for the number of ADC peripherals your mcu supports

    // Check if end of conversion for the injected mode is reached
    if (ADC1->SR & ADC_SR_JEOC) {
        // Clear the JEOCIE bit so this interrupt doesn't fire again, since it has been serviced
        ADC1->CR1 &= ~ADC_CR1_JEOCIE;

        // Invoke the user callback once the ADC sampling on the injected group is complete
        if (s_injected_done_cb[0]) {
            s_injected_done_cb[0](s_injected_done_arg[0]);

            // Clear the user passed callback since this is a one-off event
            s_injected_done_cb[0]  = NULL;
            s_injected_done_arg[0] = NULL;
        }
    }

    // Check if the AWD bit is set. This means the analog watchdog has fired an interrupt
    if (ADC1->SR & ADC_SR_AWD) {
        if (s_analog_wdg_cb[0]) {
            // Invoke the user callback since the analog watchdog has fired an interrupt
            s_analog_wdg_cb[0](s_analog_wdg_arg[0]);
            // The callback isn't cleared since this is not a one-off event. To clear, call adc_analog_wdg_stop()
        }
    }
}

// ADC1 DMA interrupt handler
void DMA2_Stream0_IRQHandler(void) {
    // Invoke user callback once the ADC sampling and DMA transfer on the regular group is complete
    if (s_dma_trans_done_cb[0]) {
        s_dma_trans_done_cb[0](s_dma_trans_cb_arg[0], HAL_OK);
        s_dma_trans_done_cb[0] = NULL;
        s_dma_trans_cb_arg[0]  = NULL;
    }
}
