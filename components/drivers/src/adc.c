#include "drivers/dma.h"
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

// The internal channels
#define ADC_CHANNEL_TEMP (16U) // Temperature sensor
#define ADC_CHANNEL_VREF (17U) // V_refint
#define ADC_CHANNEL_VBAT (18U) // V_bat


// To save user passed callbacks
static dma_trans_done_cb_t s_dma_trans_done_cb[NUM_OF_ADC_CONTROLLERS] = {};
static void*               s_dma_trans_cb_arg[NUM_OF_ADC_CONTROLLERS]  = {};

static adc_callback_t s_injected_done_cb[NUM_OF_ADC_CONTROLLERS]  = {};
static void*          s_injected_done_arg[NUM_OF_ADC_CONTROLLERS] = {};

static adc_callback_t s_analog_wdg_cb[NUM_OF_ADC_CONTROLLERS]  = {};
static void*          s_analog_wdg_arg[NUM_OF_ADC_CONTROLLERS] = {};


// Inline helpers
[[__gnu__::__always_inline__]] static inline uint8_t get_index(const ADC_TypeDef* handle) {
#if defined(ADC1)
    if (handle == ADC1) {
        return 0U;
    }
#endif

#if defined(ADC2)
    if (handle == ADC2) {
        return 1U;
    }
#endif

#if defined(ADC3)
    if (handle == ADC3) {
        return 2U;
    }
#endif

    return 0xFFU;
}

[[__gnu__::__always_inline__]] static inline void adcx_isr_helper(ADC_TypeDef* handle) {
    const uint8_t idx = get_index(handle);
    // Assert since we can't return the error anywhere
    ASSERT(idx != 0xFFU);

    // Check if end of conversion for the injected mode is reached and if interrupts for the injected group are enabled
    if ((handle->SR & ADC_SR_JEOC) && (handle->CR1 & ADC_CR1_JEOCIE)) {
        // Invoke the user callback since the sampling on the injected group is complete
        if (s_injected_done_cb[idx]) {
            // Save the user callback so we can clear it's global array position
            adc_callback_t local_cb  = s_injected_done_cb[idx];
            void*          local_arg = s_injected_done_arg[idx];

            // Clear the user passed callback since this is a one-off event
            s_injected_done_cb[idx]  = NULL;
            s_injected_done_arg[idx] = NULL;

            // Finally, invoke the user callback
            local_cb(local_arg);
        }

        // Clear the JEOCIE and JEOC bits since the interrupt has been serviced
        handle->SR &= ~ADC_SR_JEOC;
        handle->CR1 &= ~ADC_CR1_JEOCIE;
    }

    // Check if the AWD bit is set and interrupts for the analog watchdog are enabled.
    if ((handle->SR & ADC_SR_AWD) && (handle->CR1 & ADC_CR1_AWDIE)) {
        // Invoke the user callback since the analog watchdog has fired an interrupt
        if (s_analog_wdg_cb[idx]) {
            // The callback isn't cleared since this is not a one-off
            // event. To clear, adc_analog_wdg_stop() should be used.
            s_analog_wdg_cb[idx](s_analog_wdg_arg[idx]);
        }

        // Clear the AWD bit since the interrupt has already been serviced
        // The AWDIE bit is left since the watchdog can still trigger an interrupt
        handle->SR &= ~ADC_SR_AWD;
    }
}

[[__gnu__::__always_inline__]] static inline uint16_t oneshot_regular_group(ADC_TypeDef* handle, adc_channels_t channel) {
    // Set the number of channels to be converted
    // An L[3:0] value of 0b0000 means 1 channel/conversion, a value of 0b0011 means 4
    // conversions etc. Which is why 0b0000 is used despite the sampling being for 1 channel
    handle->SQR1 &= ~ADC_SQR1_L;
    handle->SQR1 |= 0b0000U << ADC_SQR1_L_Pos;

    // Set the channel
    handle->SQR3 &= ~ADC_SQR3_SQ1;
    handle->SQR3 |= (uint32_t)channel << ADC_SQR3_SQ1_Pos;

    // Disable continuous conversion and scan mode since we are sampling only one
    // channel. Then clear the EXTEN bit since the triggering is by the software
    handle->CR1 &= ~ADC_CR1_SCAN;
    handle->CR2 &= ~(ADC_CR2_CONT | ADC_CR2_EXTEN);

    // Start the conversion
    handle->CR2 |= ADC_CR2_SWSTART;

    // Poll till the conversion is complete. That is, till the EOC bit is set
    uint32_t timeout_cycles = TIMEOUT_CYCLES;
    while (!(handle->SR & ADC_SR_EOC) && --timeout_cycles);
    if (timeout_cycles == 0) {
        return UINT16_MAX; // The max value of the ADC is 2^12 (4096). It is fine to use UINT16_MAX as an error sentinel value
    }

    // Get the final result
    return (uint16_t)handle->DR & 0xFFFU;
}


// General ADC use
hal_err_t adcx_clk_enable(ADC_TypeDef* handle, bool enable) {
    if (enable) {
        if (handle == ADC1) {
            RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
        } else {
            return HAL_ERR_INVALID_ARG;
        }

    } else {
        if (handle == ADC1) {
            RCC->APB2ENR &= ~RCC_APB2ENR_ADC1EN;
        } else {
            return HAL_ERR_INVALID_ARG;
        }
    }

    __DSB();
    return HAL_OK;
}

hal_err_t adc_configure(ADC_TypeDef* handle, const adc_config_t* config) {
    if (config == NULL || handle == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    // Set the ADC resolution
    handle->CR1 &= ~ADC_CR1_RES;
    handle->CR1 |= ((uint32_t)config->resolution << ADC_CR1_RES_Pos);

    // Set the sampling cycles for all external channels
    // Clear all the bit positions first
    handle->SMPR1 &= ~(ADC_SMPR1_SMP10 | ADC_SMPR1_SMP11 | ADC_SMPR1_SMP12 | ADC_SMPR1_SMP13 | ADC_SMPR1_SMP14 | ADC_SMPR1_SMP15);
    handle->SMPR2 &= ~(ADC_SMPR2_SMP0 | ADC_SMPR2_SMP1 | ADC_SMPR2_SMP2 | ADC_SMPR2_SMP3 | ADC_SMPR2_SMP4 | ADC_SMPR2_SMP5 | ADC_SMPR2_SMP6 |
                       ADC_SMPR2_SMP7 | ADC_SMPR2_SMP8 | ADC_SMPR2_SMP9);

    const uint32_t time = config->sampling_cycles;

    // Apply the sample cycles to all external channels
    handle->SMPR1 |= ((time << ADC_SMPR1_SMP10_Pos) | (time << ADC_SMPR1_SMP11_Pos) | (time << ADC_SMPR1_SMP12_Pos) | (time << ADC_SMPR1_SMP13_Pos) |
                      (time << ADC_SMPR1_SMP14_Pos) | (time << ADC_SMPR1_SMP15_Pos));

    handle->SMPR2 |= ((time << ADC_SMPR2_SMP0_Pos) | (time << ADC_SMPR2_SMP1_Pos) | (time << ADC_SMPR2_SMP2_Pos) | (time << ADC_SMPR2_SMP3_Pos) |
                      (time << ADC_SMPR2_SMP4_Pos) | (time << ADC_SMPR2_SMP5_Pos) | (time << ADC_SMPR2_SMP6_Pos) | (time << ADC_SMPR2_SMP7_Pos) |
                      (time << ADC_SMPR2_SMP8_Pos) | (time << ADC_SMPR2_SMP9_Pos));

    // Set the data register alignment
    if (config->alignment == ADC_RIGHT_ALIGN) {
        handle->CR2 &= ~ADC_CR2_ALIGN;
    } else if (config->alignment == ADC_LEFT_ALIGN) {
        handle->CR2 |= ADC_CR2_ALIGN;
    } else {
        return HAL_ERR_INVALID_ARG;
    }

    return HAL_OK;
}

hal_err_t adc_power_on(ADC_TypeDef* handle, bool on) {
    if (handle == NULL) {
        return HAL_ERR_INVALID_ARG;
    }
    if (on) {
        handle->CR2 |= ADC_CR2_ADON;
    } else {
        handle->CR2 &= ~ADC_CR2_ADON;
    }
    return HAL_OK;
}


// Control of all the ADC instances
void adc_clk_configure(adc_prescaler_t clk_prescaler) {
    // Set the ADC clock prescaler
    ADC->CCR &= ~ADC_CCR_ADCPRE;
    ADC->CCR |= ((uint32_t)clk_prescaler << ADC_CCR_ADCPRE_Pos);
}

void adc_enable_nvic_irq(bool enable) {
    if (enable) {
        NVIC_SetPriority(ADC_IRQn, ADC_DMA_NVIC_IRQ_PRIORITY);
        NVIC_EnableIRQ(ADC_IRQn);
    } else {
        NVIC_DisableIRQ(ADC_IRQn);
    }
}

void adc_power_on_temp_sensor(bool on) {
    if (on) {
        ADC->CCR |= ADC_CCR_TSVREFE;
        delay_us(TEMP_SENSOR_STARUP_TIME_US);
    } else {
        ADC->CCR &= ~ADC_CCR_TSVREFE;
    }
}


// For use with the regular group and external channels
hal_err_t adc_regular_group_get_oneshot(ADC_TypeDef* handle, adc_channels_t channel, uint16_t* raw_data) {
    if (handle == NULL || raw_data == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    // Get the raw ADC data
    const uint16_t raw = oneshot_regular_group(handle, channel);
    if (raw == UINT16_MAX) {
        return HAL_ERR_TIMEOUT;
    }

    *raw_data = raw;
    return HAL_OK;
}

hal_err_t adc_regular_group_cont_start_conv(ADC_TypeDef* handle, const adc_continuous_config_t* config, dma_trans_done_cb_t cb, void* arg) {
    if (handle == NULL || config == NULL || config->channels == NULL || config->num_of_channels == 0 ||
        config->num_of_channels > MAX_REGULAR_CHANNELS || config->dma_buffer1 == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU) {
        return HAL_ERR_INVALID_ARG;
    }

    if (cb) {
        s_dma_trans_done_cb[idx] = cb;
        s_dma_trans_cb_arg[idx]  = arg;
    }

    // Enable the DMA clock and disable the stream
    TRY(dmax_clk_enable(ADC_DMA_CONTROLLER, true));
    TRY(dma_disable_stream(ADC_DMA_STREAM));

    // Clear the global DMA interrupt flags
    TRY(dma_clear_flags(ADC_DMA_CONTROLLER, ADC_DMA_STREAM_NO));

    // Configuration of the DMA stream
    dma_set_direction(ADC_DMA_STREAM, DMA_DIR_P_M);
    dma_set_direct_mode(ADC_DMA_STREAM, true);
    dma_set_channel(ADC_DMA_STREAM, ADC_DMA_CHANNEL);
    dma_set_trans_length(ADC_DMA_STREAM, config->buf_size);
    dma_set_increment(ADC_DMA_STREAM, false, true);
    dma_set_flow_controller(ADC_DMA_STREAM, true);
    dma_set_stream_priority(ADC_DMA_STREAM, DMA_PRIORITY_HIGH);
    dma_enable_irqs(ADC_DMA_STREAM, true, true, false, true);
    dma_set_per_mem_size(ADC_DMA_STREAM, DMA_SIZE_HWORD, DMA_SIZE_HWORD);

    if (config->use_double_buffers) {
        dma_enable_circm_dbm(ADC_DMA_STREAM, true, true);
        dma_set_addresses(ADC_DMA_STREAM, &handle->DR, config->dma_buffer1, config->dma_buffer2);
    } else if (config->dma_wraparound_when_done) {
        dma_enable_circm_dbm(ADC_DMA_STREAM, true, false);
        dma_set_addresses(ADC_DMA_STREAM, &handle->DR, config->dma_buffer1, NULL);
    } else {
        dma_enable_circm_dbm(ADC_DMA_STREAM, false, false);
        dma_set_addresses(ADC_DMA_STREAM, &handle->DR, config->dma_buffer1, NULL);
    }

    // Enable DMA stream interrupts
    NVIC_SetPriority(ADC_DMA_IRQ_TYPE, ADC_DMA_NVIC_IRQ_PRIORITY);
    NVIC_EnableIRQ(ADC_DMA_IRQ_TYPE);

    return dma_enable_stream(ADC_DMA_STREAM);
}

hal_err_t adc_regular_group_cont_end_conv(ADC_TypeDef* handle) {
    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU) {
        return HAL_ERR_INVALID_ARG;
    }

    TRY(dma_disable_stream(ADC_DMA_STREAM));

    // Clear user callback
    s_dma_trans_done_cb[idx] = NULL;
    s_dma_trans_cb_arg[idx]  = NULL;

    return HAL_OK;
}


// For use with the injected group and external channels
hal_err_t adc_injected_group_start_conv(ADC_TypeDef* handle, const adc_channels_t* channels, size_t size, adc_callback_t cb, void* arg) {
    if (handle == NULL || channels == NULL || size == 0 || size > MAX_INJECTED_CHANNELS) {
        return HAL_ERR_INVALID_ARG;
    }

    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU) {
        return HAL_ERR_INVALID_ARG;
    }

    if (cb) {
        // Save the user passed callback
        s_injected_done_cb[idx]  = cb;
        s_injected_done_arg[idx] = arg;

        // Enable interrupts for the injected group on conversion
        // completion only if the user passed in a callback.
        handle->CR1 |= ADC_CR1_JEOCIE;
    }

    // Set the number of channels/conversions in the JL bit positions of the
    // JSQR register. The JL bit positions are zero indexed. That is, 1 channel
    // means a JL value of 0b00, 3 channels means a JL value of 0b10 etc.
    handle->JSQR &= ~(ADC_JSQR_JL | ADC_JSQR_JSQ1 | ADC_JSQR_JSQ2 | ADC_JSQR_JSQ3 | ADC_JSQR_JSQ4);
    handle->JSQR |= (((size - 1) & 0x3U) << ADC_JSQR_JL_Pos);

    // Set the channel sequence in the JSQ register
    // As per the TRM, there are only 4 injected channels, and they have to be filled from the last
    // slot, that is, JSQ4. This is because all conversions in the injected group must end at JSQ4
    switch (size) {
        case 1:
            handle->JSQR |= (uint32_t)(channels[0] << ADC_JSQR_JSQ4_Pos);
            break;
        case 2:
            handle->JSQR |= (uint32_t)((channels[0] << ADC_JSQR_JSQ3_Pos) | (channels[1] << ADC_JSQR_JSQ4_Pos));
            break;
        case 3:
            handle->JSQR |= (uint32_t)((channels[0] << ADC_JSQR_JSQ2_Pos) | (channels[1] << ADC_JSQR_JSQ3_Pos) | (channels[2] << ADC_JSQR_JSQ4_Pos));
            break;
        case 4:
            handle->JSQR |= (uint32_t)((channels[0] << ADC_JSQR_JSQ1_Pos) | (channels[1] << ADC_JSQR_JSQ2_Pos) | (channels[2] << ADC_JSQR_JSQ3_Pos) |
                                       (channels[3] << ADC_JSQR_JSQ4_Pos));
            break;
        default:
            return HAL_ERR_INVALID_ARG;
    }

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

hal_err_t adc_injected_group_get_result(ADC_TypeDef* handle, uint16_t* raw_data, size_t size) {
    if (handle == NULL || raw_data == NULL) {
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
            raw_data[0] = (uint16_t)handle->JDR1;
            break;
        case 2:
            raw_data[0] = (uint16_t)handle->JDR1;
            raw_data[1] = (uint16_t)handle->JDR2;
            break;
        case 3:
            raw_data[0] = (uint16_t)handle->JDR1;
            raw_data[1] = (uint16_t)handle->JDR2;
            raw_data[2] = (uint16_t)handle->JDR3;
            break;
        case 4:
            raw_data[0] = (uint16_t)handle->JDR1;
            raw_data[1] = (uint16_t)handle->JDR2;
            raw_data[2] = (uint16_t)handle->JDR3;
            raw_data[3] = (uint16_t)handle->JDR4;
            break;
        default:
            return HAL_ERR_INVALID_ARG;
    }

    return HAL_OK;
}


// For use with the internal channels. These only provide the raw ADC values. They all make use of the regular group
hal_err_t adc_get_v_bat(ADC_TypeDef* handle, uint16_t* raw_data) {
    if (handle == NULL || raw_data == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    // Enable V_bat so the ADC can measure it
    ADC->CCR |= ADC_CCR_VBATE;

    // Set the number of sampling cycles to 480 cycles.
    // This is done because the internal channels require
    // a much higher sampling time than the external channels
    handle->SMPR1 &= ~ADC_SMPR1_SMP18;
    handle->SMPR1 |= (ADC_SAMPLE_480_CYCLES << ADC_SMPR1_SMP18_Pos);

    // Get the raw ADC data
    const uint16_t raw = oneshot_regular_group(handle, ADC_CHANNEL_VBAT);

    // Disconnect V_bat when done
    ADC->CCR &= ~ADC_CCR_VBATE;

    if (raw == UINT16_MAX) {
        return HAL_ERR_TIMEOUT;
    }

    // Get V_bat: The ADC only ever sees (V_bat / VBAT_DIVIDER_RATIO) so we
    // have to multiply by VBAT_DIVIDER_RATIO to get the actual V_bat value.
    // This is done by the hardware because V_bat could exceed the operating voltage
    // range of the ADC, so it is divided so the ADC can function correctly
    *raw_data = raw * VBAT_DIVIDER_RATIO;
    return HAL_OK;
}

hal_err_t adc_get_temperature(ADC_TypeDef* handle, uint16_t* raw_data) {
    if (handle == NULL || raw_data == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    // Wake the temperature sensor if it's not already powered on
    if (!(ADC->CCR & ADC_CCR_TSVREFE)) {
        adc_power_on_temp_sensor(true);
    }

    // Set the number of sampling cycles to 480 cycles.
    // This is done because the internal channels require
    // a much higher sampling time than the external channels
    handle->SMPR1 &= ~ADC_SMPR1_SMP16;
    handle->SMPR1 |= (ADC_SAMPLE_480_CYCLES << ADC_SMPR1_SMP16_Pos);

    // Get the raw ADC data
    const uint16_t raw = oneshot_regular_group(handle, ADC_CHANNEL_TEMP);
    if (raw == UINT16_MAX) {
        return HAL_ERR_TIMEOUT;
    }

    // Get the raw ADC value
    *raw_data = raw;
    return HAL_OK;
}

hal_err_t adc_get_v_ref_internal(ADC_TypeDef* handle, uint16_t* raw_data) {
    if (handle == NULL || raw_data == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    // Measuring VREFINT requires the temperature sensor to be powered on
    if (!(ADC->CCR & ADC_CCR_TSVREFE)) {
        adc_power_on_temp_sensor(true);
    }

    // Set the number of sampling cycles to 480 cycles.
    // This is done because the internal channels require
    // a much higher sampling time than the external channels
    handle->SMPR1 &= ~ADC_SMPR1_SMP17;
    handle->SMPR1 |= (ADC_SAMPLE_480_CYCLES << ADC_SMPR1_SMP17_Pos);

    // Get the raw ADC data
    const uint16_t raw = oneshot_regular_group(handle, ADC_CHANNEL_VREF);
    if (raw == UINT16_MAX) {
        return HAL_ERR_TIMEOUT;
    }

    // Get the raw ADC value
    *raw_data = raw;
    return HAL_OK;
}


// For use of getting the actual voltages read by the ADC
hal_err_t adc_get_vdda(ADC_TypeDef* handle, float* vdda) {
    if (handle == NULL || vdda == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    uint16_t raw_vref = 0;
    TRY(adc_get_v_ref_internal(handle, &raw_vref));

    // Read the V_refint calibraion value. It is stored at the address 0x1FFF7A2A
    // It is the value of V_ref_int measured at V_dda = 3.3V and 30C
    const uint16_t v_refint_cal = *(volatile uint16_t*)0x1FFF7A2AU;

    // Calculate the actual VDDA from the calibration data
    *vdda = (3.3F * (float)v_refint_cal) / (float)raw_vref;

    return HAL_OK;
}

hal_err_t adc_get_temp_celsius(ADC_TypeDef* handle, float* temp_celsius) {
    if (handle == NULL || temp_celsius == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    // Get the V_ref_int and V_ref_int_calibration data
    const uint16_t v_refint_cal = *(volatile uint16_t*)0x1FFF7A2AU;
    uint16_t       v_ref_int    = 0;
    TRY(adc_get_v_ref_internal(handle, &v_ref_int));

    // Read the raw ADC temperature value next
    uint16_t raw_temp = 0;
    TRY(adc_get_temperature(handle, &raw_temp));

    // Normalize the raw temperature sensor data read
    const float normalized = ((float)v_refint_cal / (float)v_ref_int) * (float)raw_temp;

    // Get the temperature calibration values from their locations in memory
    // They represent the temperature values measured at 30C and 110C respectively when VDDA is 3.3V
    const uint16_t temp_cal_1 = *(volatile uint16_t*)0x1FFF7A2CU;
    const uint16_t temp_cal_2 = *(volatile uint16_t*)0x1FFF7A2EU;

    // Get the temperature using linear interpolation with the calibration data at 110C and 30C
    *temp_celsius = (((110.0F - 30.0F) / (float)(temp_cal_2 - temp_cal_1)) * (normalized - (float)temp_cal_1)) + 30.0F;

    return HAL_OK;
}

hal_err_t adc_get_value_right_aligned(ADC_TypeDef* handle, uint16_t raw_data, adc_resolution_t resolution, float* voltage) {
    if (handle == NULL || voltage == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    // Get the VDDA first
    float vdda = 0.0F;
    TRY(adc_get_vdda(handle, &vdda));

    uint16_t resolution_value = 0;
    switch (resolution) {
        case ADC_RES_6_BITS:
            resolution_value = 6;
            break;
        case ADC_RES_8_BITS:
            resolution_value = 8;
            break;
        case ADC_RES_10_BITS:
            resolution_value = 10;
            break;
        case ADC_RES_12_BITS:
            resolution_value = 12;
            break;
        default:
            return HAL_ERR_INVALID_ARG;
    }

    // Calculate the final voltage
    *voltage = (vdda * (float)raw_data) / (float)((1UL << resolution_value) - 1);

    return HAL_OK;
}


// For use of control of the analog watchdog
hal_err_t adc_analog_wdg_start(ADC_TypeDef* handle, uint16_t min, uint16_t max, bool regular, bool injected, adc_callback_t cb, void* arg) {
    if (handle == NULL || cb == NULL || min >= max) {
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
#if defined(ADC1)
    adcx_isr_helper(ADC1);
#endif

#if defined(ADC2)
    adcx_isr_helper(ADC2);
#endif

#if defined(ADC3)
    adcx_isr_helper(ADC3);
#endif
}

// DMA interrupt handler
void DMA2_Stream0_IRQHandler(void) {
    // TODO: Handle DMA completion events for the continuous mode with the regular group
}
