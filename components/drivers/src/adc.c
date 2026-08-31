#include "stm32f411xe.h"
#include "utils/common.h"
#include "drivers/dma.h"
#include "drivers/adc.h"
#include "utils/tick.h"
#include "utils/err.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>


// The internal channels
#define ADC_CHANNEL_TEMP (16U) // Temperature sensor
#define ADC_CHANNEL_VREF (17U) // V_refint
#define ADC_CHANNEL_VBAT (18U) // V_bat


// User context
typedef struct {
    adc_dma_callbacks_t continuous_mode_callbacks;

    adc_callback_t injected_done_cb;
    void*          injected_done_arg;

    adc_callback_t analog_wdg_cb;
    void*          analog_wdg_arg;
} adc_ctx_t;

static adc_ctx_t s_adc_ctx[NUM_OF_ADC_CONTROLLERS] = {};


// Mapping for the DMA channels for the ADC peripheral instances
static const dma_map_t s_adc_dma_map[] = {
#if defined(ADC1)
    // ADC1
    {.controller = DMA2, .stream = DMA2_Stream0, .stream_no = 0, .irq_type = DMA2_Stream0_IRQn, .channel = 0},
#endif
#if defined(ADC2)
    // ADC2
    {.controller = NULL, .stream = NULL, .stream_no = 0, .irq_type = 0, .channel = 0},
#endif
#if defined(ADC3)
    // ADC3
    {.controller = NULL, .stream = NULL, .stream_no = 0, .irq_type = 0, .channel = 0},
#endif
};


// Inline helpers
static inline uint8_t get_index(const ADC_TypeDef* handle) {
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

static inline void adcx_isr_helper(ADC_TypeDef* handle) {
    const uint8_t idx = get_index(handle);
    // Assert since we can't return the error anywhere
    ASSERT(idx != 0xFFU);

    // Check if end of conversion for the injected mode is reached and if interrupts for the injected group are enabled
    if ((handle->SR & ADC_SR_JEOC) && (handle->CR1 & ADC_CR1_JEOCIE)) {
        // Invoke the user callback since the sampling on the injected group is complete
        if (s_adc_ctx[idx].injected_done_cb) {
            // Save the user callback so we can clear it's global array position
            const adc_callback_t local_cb  = s_adc_ctx[idx].injected_done_cb;
            void* const          local_arg = s_adc_ctx[idx].injected_done_arg;

            // Clear the user passed callback since this is a one-off event
            s_adc_ctx[idx].injected_done_cb  = NULL;
            s_adc_ctx[idx].injected_done_arg = NULL;

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
        if (s_adc_ctx[idx].analog_wdg_cb) {
            // The callback isn't cleared since this is not a one-off
            // event. To clear, adc_analog_wdg_stop() should be used.
            s_adc_ctx[idx].analog_wdg_cb(s_adc_ctx[idx].analog_wdg_arg);
        }

        // Clear the AWD bit since the interrupt has already been serviced
        // The AWDIE bit is left since the watchdog can still trigger an interrupt
        handle->SR &= ~ADC_SR_AWD;
    }

    // Check if the OVR bit is set and interrupts for data overrun are enabled.
    if ((handle->SR & ADC_SR_OVR) && (handle->CR1 & ADC_CR1_OVRIE)) {
        // Invoke the user callback since there has been data overrun in the data register
        if (s_adc_ctx[idx].continuous_mode_callbacks.on_data_overrun) {
            // Save the user callback so we can clear it's global array position
            const adc_cont_err_cb_t local_cb  = s_adc_ctx[idx].continuous_mode_callbacks.on_data_overrun;
            void* const             local_arg = s_adc_ctx[idx].continuous_mode_callbacks.user;

            // Get the user paramters before clearing all state with adc_regular_group_cont_end_conv(...)
            DMA_Stream_TypeDef* stream = s_adc_dma_map[idx].stream;
            ASSERT(stream);

            // If the CT bit is 0, that means the DMA controller is in the first buffer
            const bool     is_buf_1          = (stream->CR & DMA_SxCR_CT) == 0;
            const uint16_t num_of_items_left = (uint16_t)stream->NDTR;

            // End the regular group conversion. The conversion can be restarted in the callback if necessary
            ASSERT(adc_regular_group_cont_end_conv(handle) == HAL_OK);

            // Finally, invoke the user callback
            local_cb(local_arg, is_buf_1, num_of_items_left);
        }

        // No need to clear ADC_SR_OVR and ADC_CR1_OVRIE since adc_regular_group_cont_end_conv(...) already does
    }
}

static inline void adcx_dma_isr_helper(ADC_TypeDef*       handle,
                                       volatile uint32_t* irq_clear_register,
                                       volatile uint32_t* irq_status_register,
                                       uint32_t           tc_mask,
                                       uint32_t           te_mask,
                                       uint32_t           dme_mask) {
    const uint8_t idx = get_index(handle);
    ASSERT(idx != 0xFFU);

    DMA_TypeDef*        controller = s_adc_dma_map[idx].controller;
    DMA_Stream_TypeDef* stream     = s_adc_dma_map[idx].stream;
    ASSERT(controller && stream);

    // If the CT bit is 0, that means the DMA controller is in the first buffer
    const bool     is_buf_1          = (stream->CR & DMA_SxCR_CT) == 0;
    const uint16_t num_of_items_left = (uint16_t)stream->NDTR;
    void* const    local_arg         = s_adc_ctx[idx].continuous_mode_callbacks.user;

    if (*irq_status_register & tc_mask) {
        // Transfer complete. If in circular or double buffering mode, the DMA has either
        // wrapped around in the same buffer or has switched to the second buffer respectively.
        if (s_adc_ctx[idx].continuous_mode_callbacks.on_buffer_full) {
            s_adc_ctx[idx].continuous_mode_callbacks.on_buffer_full(local_arg, is_buf_1);
            // If we are not in circular or double buffering mode, end the conversion
            if (!((stream->CR & DMA_SxCR_CIRC) || (stream->CR & DMA_SxCR_DBM))) {
                ASSERT(adc_regular_group_cont_end_conv(handle) == HAL_OK);
            }
        }
        // Clear the TC flag. The DMA_HIFCR and DMA_LIFCR are write 1 to clear
        *irq_clear_register |= tc_mask;
    }

    // Errors that end the conversion. Either a plain DMA transfer error or a direct mode error.
    // Conversions can be restarted by the user if necessary in the callback.
    if (*irq_status_register & te_mask) {
        // Transfer error. End the conversion and invoke the user callback.
        if (s_adc_ctx[idx].continuous_mode_callbacks.on_transfer_error) {
            adc_cont_err_cb_t local_cb = s_adc_ctx[idx].continuous_mode_callbacks.on_transfer_error;
            ASSERT(adc_regular_group_cont_end_conv(handle) == HAL_OK);
            local_cb(local_arg, is_buf_1, num_of_items_left);
        }
        // Clear the TE flag. The DMA_HIFCR and DMA_LIFCR are write 1 to clear
        *irq_clear_register |= te_mask;
    }

    if (*irq_status_register & dme_mask) {
        // Direct mode error. End the conversion and invoke the user callback.
        if (s_adc_ctx[idx].continuous_mode_callbacks.on_direct_mode_error) {
            adc_cont_err_cb_t local_cb = s_adc_ctx[idx].continuous_mode_callbacks.on_direct_mode_error;
            ASSERT(adc_regular_group_cont_end_conv(handle) == HAL_OK);
            local_cb(local_arg, is_buf_1, num_of_items_left);
        }
        // Clear the DME flag. The DMA_HIFCR and DMA_LIFCR are write 1 to clear
        *irq_clear_register |= dme_mask;
    }
}

static inline void clear_state(ADC_TypeDef* handle, bool regular, bool injected) {
    if (regular) {
        handle->SR &= ~(ADC_SR_EOC | ADC_SR_STRT | ADC_SR_OVR);
        handle->SR &= ~(ADC_SR_EOC | ADC_SR_STRT | ADC_SR_OVR);
        handle->CR1 &= ~(ADC_CR1_DISCEN | ADC_CR1_EOCIE | ADC_CR1_DISCNUM | ADC_CR1_OVRIE);
        handle->CR2 &= ~(ADC_CR2_CONT | ADC_CR2_EXTEN | ADC_CR2_DMA | ADC_CR2_SWSTART | ADC_CR2_EXTSEL | ADC_CR2_EOCS | ADC_CR2_DDS);
        handle->SQR1 &= ~(ADC_SQR1_L | ADC_SQR1_SQ13 | ADC_SQR1_SQ14 | ADC_SQR1_SQ15 | ADC_SQR1_SQ16);
        handle->SQR2 &= ~(ADC_SQR2_SQ7 | ADC_SQR2_SQ8 | ADC_SQR2_SQ9 | ADC_SQR2_SQ10 | ADC_SQR2_SQ11 | ADC_SQR2_SQ12);
        handle->SQR3 &= ~(ADC_SQR3_SQ1 | ADC_SQR3_SQ2 | ADC_SQR3_SQ3 | ADC_SQR3_SQ4 | ADC_SQR3_SQ5 | ADC_SQR3_SQ6);
    } else if (injected) {
        handle->SR &= ~(ADC_SR_JEOC | ADC_SR_JSTRT);
        handle->SR &= ~(ADC_SR_JEOC | ADC_SR_JSTRT);
        handle->CR1 &= ~(ADC_CR1_JDISCEN | ADC_CR1_JEOCIE | ADC_CR1_JAUTO);
        handle->CR2 &= ~(ADC_CR2_JEXTEN | ADC_CR2_JSWSTART | ADC_CR2_JEXTSEL);
        handle->JSQR &= ~(ADC_JSQR_JL | ADC_JSQR_JSQ1 | ADC_JSQR_JSQ2 | ADC_JSQR_JSQ3 | ADC_JSQR_JSQ4);
        handle->JOFR1 &= ~ADC_JOFR1_JOFFSET1;
        handle->JOFR2 &= ~ADC_JOFR2_JOFFSET2;
        handle->JOFR3 &= ~ADC_JOFR3_JOFFSET3;
        handle->JOFR4 &= ~ADC_JOFR4_JOFFSET4;
    }
}

static inline uint16_t oneshot_regular_group(ADC_TypeDef* handle, adc_channels_t channel) {
    // Clear all stale state before proceeding
    clear_state(handle, true, false);

    // Set the number of channels to be converted
    // An L[3:0] value of 0b0000 means 1 channel/conversion, a value of 0b0011 means 4
    // conversions etc. Which is why 0b0000 is used despite the sampling being for 1 channel
    handle->SQR1 |= 0b0000U << ADC_SQR1_L_Pos;

    // Set the channel
    handle->SQR3 |= (uint32_t)channel << ADC_SQR3_SQ1_Pos;

    // Start the conversion, and set the EOCS bit so that the EOC bit is set after any regular conversion
    handle->CR2 |= (ADC_CR2_SWSTART | ADC_CR2_EOCS);

    // Poll till the conversion is complete. That is, till the EOC bit is set
    uint32_t timeout_cycles = TIMEOUT_CYCLES;
    while (!(handle->SR & ADC_SR_EOC) && --timeout_cycles);
    if (timeout_cycles == 0) {
        // The max value of the ADC is 2^12 - 1 (4095). It is
        // safe to use UINT16_MAX as an error sentinel value.
        return UINT16_MAX;
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
        delay_us(ADC_STARUP_TIME_US);
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


// For use with the regular group and external channels in polling oneshot mode
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


// For use with the regular group and external channels in DMA continuous sampling mode
hal_err_t adc_regular_group_cont_start_conv(ADC_TypeDef* handle, const adc_continuous_config_t* config) {
    if (handle == NULL || config == NULL || config->channels.channels_sequence == NULL || config->channels.num_of_channels == 0 ||
        config->channels.num_of_channels > MAX_REGULAR_CHANNELS || config->buffer_1 == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU) {
        return HAL_ERR_INVALID_ARG;
    }

    // Clear all stale state before proceeding
    clear_state(handle, true, false);

    // Enable scan mode if we have more than one channel
    if (config->channels.num_of_channels > 1) {
        handle->CR1 |= ADC_CR1_SCAN;
    }

    // Set the number of channels/conversions in the L bit positions of the
    // SQR1 register. The L bit positions are zero indexed. That is, 2 channels
    // means an L value of 0b0001, 3 channels means an L value of 0b0010 etc.
    handle->SQR1 |= ((config->channels.num_of_channels - 1) << ADC_SQR1_L_Pos) & ADC_SQR1_L;

    for (size_t i = 0; i < config->channels.num_of_channels; i++) {
        // TODO: Set the channel sequence
        (void)config->channels.channels_sequence;
    }

    // ADC DMA stream mapping
    DMA_TypeDef*        controller = s_adc_dma_map[idx].controller;
    DMA_Stream_TypeDef* stream     = s_adc_dma_map[idx].stream;
    const uint8_t       channel    = s_adc_dma_map[idx].channel;
    const uint8_t       stream_no  = s_adc_dma_map[idx].stream_no;
    const IRQn_Type     irq_type   = s_adc_dma_map[idx].irq_type;

    if (controller == NULL || stream == NULL) {
        return HAL_ERR_NOT_SUPPORTED;
    }

    // Enable the DMA clock and disable the stream
    TRY(dmax_clk_enable(controller, true));
    TRY(dma_disable_stream(stream));

    // Clear the global DMA interrupt flags
    TRY(dma_clear_flags(controller, stream_no));

    // Configuration of the DMA stream
    dma_set_direction(stream, DMA_DIR_P_M);
    dma_set_direct_mode(stream, true);
    dma_set_channel(stream, channel);
    dma_set_increment(stream, false, true);
    dma_set_flow_controller(stream, true);
    dma_set_trans_length(stream, config->buffer_size);
    dma_set_stream_priority(stream, DMA_PRIORITY_HIGH);
    dma_set_per_mem_size(stream, DMA_SIZE_HWORD, DMA_SIZE_HWORD);

    if (config->use_double_buffers && config->buffer_2 == NULL) {
        return HAL_ERR_INVALID_ARG;
    }

    const bool  double_buffering = config->use_double_buffers;
    const bool  circular_mode    = config->use_double_buffers || config->dma_wraparound_when_done;
    const void* buffer_2         = config->use_double_buffers ? config->buffer_2 : NULL;

    dma_enable_circm_dbm(stream, circular_mode, double_buffering);
    dma_set_addresses(stream, &handle->DR, config->buffer_1, buffer_2);

    // Enable ADC continuous sampling and DMA mode
    // Set the DDS bit only if we are in circular mode so the ADC continues
    // to send DMA requests even after the first buffer is filled.
    if (circular_mode) {
        handle->CR2 |= (ADC_CR2_CONT | ADC_CR2_DMA | ADC_CR2_DDS);
    } else {
        handle->CR2 |= (ADC_CR2_CONT | ADC_CR2_DMA);
    }

    // Clear the CT bit to ensure the DMA controller starts at the first buffer
    // Enable ADC continuous sampling and DMA mode
    // Set the DDS bit only if we are in circular mode so the ADC continues
    // to send DMA requests even after the first buffer is filled.
    if (circular_mode) {
        handle->CR2 |= (ADC_CR2_CONT | ADC_CR2_DMA | ADC_CR2_DDS);
    } else {
        handle->CR2 |= (ADC_CR2_CONT | ADC_CR2_DMA);
    }

    // Clear the CT bit to ensure the DMA controller starts at the first buffer
    stream->CR &= ~DMA_SxCR_CT;

    // Callback registration
    s_adc_ctx[idx].continuous_mode_callbacks = config->callbacks;

    const bool transfer_error    = s_adc_ctx[idx].continuous_mode_callbacks.on_transfer_error != NULL;
    const bool direct_mode_error = s_adc_ctx[idx].continuous_mode_callbacks.on_direct_mode_error != NULL;
    const bool transfer_complete = s_adc_ctx[idx].continuous_mode_callbacks.on_buffer_full != NULL;
    const bool data_overrun      = s_adc_ctx[idx].continuous_mode_callbacks.on_data_overrun != NULL;

    // Enable the interrupts based on what callbacks were passed
    dma_enable_irqs(stream, transfer_complete, transfer_error, false, direct_mode_error);
    if (data_overrun) {
        handle->CR1 |= ADC_CR1_OVRIE;
    }

    // Enable the DMA stream interrupts
    NVIC_SetPriority(irq_type, ADC_DMA_NVIC_IRQ_PRIORITY);
    NVIC_EnableIRQ(irq_type);

    // Enable the DMA stream after all DMA configuration is complete
    TRY(dma_enable_stream(stream));

    if (config->trigger == RG_TRIGGER_SOFTWARE) {
        // If the trigger is from software, set the SWSTART bit and return as that's all that's needed
        handle->CR2 |= ADC_CR2_SWSTART;
    } else {
        // Set the polarity of the trigger and the would be trigger source
        handle->CR2 |= (uint32_t)(config->trigger_polarity << ADC_CR2_EXTEN_Pos);
        handle->CR2 |= (uint32_t)(config->trigger << ADC_CR2_EXTSEL_Pos);
    }

    return HAL_OK;
}

hal_err_t adc_regular_group_cont_end_conv(ADC_TypeDef* handle) {
    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU) {
        return HAL_ERR_INVALID_ARG;
    }

    // Stop the conversion by clearing the necessary bits
    clear_state(handle, true, false);

    // ADC DMA stream mapping
    DMA_TypeDef*        controller = s_adc_dma_map[idx].controller;
    DMA_Stream_TypeDef* stream     = s_adc_dma_map[idx].stream;
    const uint8_t       stream_no  = s_adc_dma_map[idx].stream_no;

    if (controller == NULL || stream == NULL) {
        return HAL_ERR_NOT_SUPPORTED;
    }

    // Then disable the stream, disable all interrupts and clear the interrupt flags
    TRY(dma_disable_stream(stream));
    TRY(dma_clear_flags(controller, stream_no));
    dma_enable_irqs(stream, false, false, false, false);

    // Clear all user passed callbacks
    memset(&s_adc_ctx[idx].continuous_mode_callbacks, 0, sizeof(s_adc_ctx[idx].continuous_mode_callbacks));

    return HAL_OK;
}


// For use with the injected group and external channels with interrupts
hal_err_t adc_injected_group_start_conv(ADC_TypeDef* handle, const adc_injected_group_config_t* config) {
    if (handle == NULL || config == NULL || config->channels.channels_sequence == NULL || config->channels.num_of_channels == 0 ||
        config->channels.num_of_channels > MAX_INJECTED_CHANNELS) {
        return HAL_ERR_INVALID_ARG;
    }

    const uint8_t idx = get_index(handle);
    if (idx == 0xFFU) {
        return HAL_ERR_INVALID_ARG;
    }

    // Clear all stale state before proceeding
    clear_state(handle, false, true);

    // Enable scan mode if we have more than one channel
    if (config->channels.num_of_channels > 1) {
        handle->CR1 |= ADC_CR1_SCAN;
    }

    // Set the number of channels/conversions in the JL bit positions of the
    // JSQR register. The JL bit positions are zero indexed. That is, 1 channel
    // means a JL value of 0b00, 3 channels means a JL value of 0b10 etc.
    handle->JSQR |= ((config->channels.num_of_channels - 1) << ADC_JSQR_JL_Pos) & ADC_JSQR_JL;

    // Set the channel sequence in the JSQ register and the the offsets
    // As per the TRM, there are only 4 injected channels, and they have to be filled from the last
    // slot, that is, JSQ4. This is because all conversions in the injected group must end at JSQ4
    switch (config->channels.num_of_channels) {
        case 1:
            handle->JSQR |= (uint32_t)(config->channels.channels_sequence[0] << ADC_JSQR_JSQ4_Pos);
            handle->JOFR1 |= (uint32_t)(config->offsets[0] << ADC_JOFR1_JOFFSET1_Pos) & ADC_JOFR1_JOFFSET1;
            break;
        case 2:
            handle->JSQR |= (uint32_t)((config->channels.channels_sequence[0] << ADC_JSQR_JSQ3_Pos) |
                                       (config->channels.channels_sequence[1] << ADC_JSQR_JSQ4_Pos));
            handle->JOFR1 |= (uint32_t)(config->offsets[0] << ADC_JOFR1_JOFFSET1_Pos) & ADC_JOFR1_JOFFSET1;
            handle->JOFR2 |= (uint32_t)(config->offsets[1] << ADC_JOFR2_JOFFSET2_Pos) & ADC_JOFR2_JOFFSET2;
            break;
        case 3:
            handle->JSQR |= (uint32_t)((config->channels.channels_sequence[0] << ADC_JSQR_JSQ2_Pos) |
                                       (config->channels.channels_sequence[1] << ADC_JSQR_JSQ3_Pos) |
                                       (config->channels.channels_sequence[2] << ADC_JSQR_JSQ4_Pos));
            handle->JOFR1 |= (uint32_t)(config->offsets[0] << ADC_JOFR1_JOFFSET1_Pos) & ADC_JOFR1_JOFFSET1;
            handle->JOFR2 |= (uint32_t)(config->offsets[1] << ADC_JOFR2_JOFFSET2_Pos) & ADC_JOFR2_JOFFSET2;
            handle->JOFR3 |= (uint32_t)(config->offsets[2] << ADC_JOFR3_JOFFSET3_Pos) & ADC_JOFR3_JOFFSET3;
            break;
        case 4:
            handle->JSQR |= (uint32_t)((config->channels.channels_sequence[0] << ADC_JSQR_JSQ1_Pos) |
                                       (config->channels.channels_sequence[1] << ADC_JSQR_JSQ2_Pos) |
                                       (config->channels.channels_sequence[2] << ADC_JSQR_JSQ3_Pos) |
                                       (config->channels.channels_sequence[3] << ADC_JSQR_JSQ4_Pos));
            handle->JOFR1 |= (uint32_t)(config->offsets[0] << ADC_JOFR1_JOFFSET1_Pos) & ADC_JOFR1_JOFFSET1;
            handle->JOFR2 |= (uint32_t)(config->offsets[1] << ADC_JOFR2_JOFFSET2_Pos) & ADC_JOFR2_JOFFSET2;
            handle->JOFR3 |= (uint32_t)(config->offsets[2] << ADC_JOFR3_JOFFSET3_Pos) & ADC_JOFR3_JOFFSET3;
            handle->JOFR4 |= (uint32_t)(config->offsets[3] << ADC_JOFR4_JOFFSET4_Pos) & ADC_JOFR4_JOFFSET4;
            break;
        default:
            return HAL_ERR_INVALID_ARG;
    }

    if (config->on_conv_complete) {
        // Save the user passed callback
        s_adc_ctx[idx].injected_done_cb  = config->on_conv_complete;
        s_adc_ctx[idx].injected_done_arg = config->arg;

        // Enable interrupts for the injected group on conversion
        // completion only if the user passed in a callback.
        handle->CR1 |= ADC_CR1_JEOCIE;
    }

    if (config->trigger == JG_TRIGGER_SOFTWARE) {
        // If the trigger is from software, set the JSWSTART bit and return as that's all that's needed
        handle->CR2 |= ADC_CR2_JSWSTART;
    } else {
        // Set the polarity of the trigger and the would be trigger source
        handle->CR2 |= (uint32_t)(config->trigger_polarity << ADC_CR2_JEXTEN_Pos);
        handle->CR2 |= (uint32_t)(config->trigger << ADC_CR2_JEXTSEL_Pos);
    }

    return HAL_OK;
}

hal_err_t adc_injected_group_get_result(ADC_TypeDef* handle, uint16_t* raw_data_buffer, size_t buffer_size) {
    if (handle == NULL || raw_data_buffer == NULL || buffer_size == 0 || buffer_size > MAX_INJECTED_CHANNELS) {
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
    switch (buffer_size) {
        case 1:
            raw_data_buffer[0] = (uint16_t)handle->JDR1;
            break;
        case 2:
            raw_data_buffer[0] = (uint16_t)handle->JDR1;
            raw_data_buffer[1] = (uint16_t)handle->JDR2;
            break;
        case 3:
            raw_data_buffer[0] = (uint16_t)handle->JDR1;
            raw_data_buffer[1] = (uint16_t)handle->JDR2;
            raw_data_buffer[2] = (uint16_t)handle->JDR3;
            break;
        case 4:
            raw_data_buffer[0] = (uint16_t)handle->JDR1;
            raw_data_buffer[1] = (uint16_t)handle->JDR2;
            raw_data_buffer[2] = (uint16_t)handle->JDR3;
            raw_data_buffer[3] = (uint16_t)handle->JDR4;
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
hal_err_t adc_analog_wdg_start(ADC_TypeDef* handle, const adc_analog_wdg_config_t* config) {
    if (handle == NULL || config == NULL || config->on_thresholds_violated == NULL || config->min_adc_value >= config->max_adc_value) {
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
    handle->CR1 &= ~(ADC_CR1_AWDIE | ADC_CR1_JAWDEN | ADC_CR1_AWDEN | ADC_CR1_AWDSGL);

    if (config->monitor_regular_channels && config->monitor_injected_channels) {
        // Enable monitoring on all channels
        handle->CR1 |= (ADC_CR1_AWDEN | ADC_CR1_JAWDEN);
    } else if (config->monitor_injected_channels) {
        // Enable monitoring on only the injected channels
        handle->CR1 |= ADC_CR1_JAWDEN;
    } else if (config->monitor_regular_channels) {
        // Enable monitoring on only the regular channels
        handle->CR1 |= ADC_CR1_AWDEN;
    } else {
        return HAL_ERR_INVALID_ARG;
    }

    // Set the voltage thresholds
    handle->HTR = config->max_adc_value & 0xFFFU; // Only the lower 12 bits are used
    handle->LTR = config->min_adc_value & 0xFFFU; // Only the lower 12 bits are used

    // Save the user passed callback
    s_adc_ctx[idx].analog_wdg_cb  = config->on_thresholds_violated;
    s_adc_ctx[idx].analog_wdg_arg = config->arg;

    // Enable the analog watchdog interrupt and clear any pending interrupts
    handle->SR &= ~ADC_SR_AWD;
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
    handle->SR &= ~ADC_SR_AWD;

    // Clear the voltage thresholds
    handle->HTR |= ADC_HTR_HT;  // Set to highest value possible
    handle->LTR &= ~ADC_LTR_LT; // Set to lowest value possible

    // Clear the user passed callback
    s_adc_ctx[idx].analog_wdg_cb  = NULL;
    s_adc_ctx[idx].analog_wdg_arg = NULL;

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

void DMA2_Stream0_IRQHandler(void) {
#if defined(ADC1)
    adcx_dma_isr_helper(ADC1, &DMA2->LIFCR, &DMA2->LISR, DMA_LISR_TCIF0, DMA_LISR_TEIF0, DMA_LISR_DMEIF0);
#endif
#if defined(ADC2)
    adcx_dma_isr_helper(ADC2, &DMA2->LIFCR, &DMA2->LISR, DMA_LISR_TCIF0, DMA_LISR_TEIF0, DMA_LISR_DMEIF0);
#endif
#if defined(ADC3)
    adcx_dma_isr_helper(ADC3, &DMA2->LIFCR, &DMA2->LISR, DMA_LISR_TCIF0, DMA_LISR_TEIF0, DMA_LISR_DMEIF0);
#endif
}
