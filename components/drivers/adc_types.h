#ifndef ADC_TYPES_H_
#define ADC_TYPES_H_


#include "drivers/dma.h"

#include <stdint.h>
#include <stddef.h>


// Number of regular and injected channels supported by the ADC peripheral
#define MAX_REGULAR_CHANNELS (16)
#define MAX_INJECTED_CHANNELS (4)


// Shape of all callbacks passed to the ADC driver
typedef void (*adc_callback_t)(void* arg);
typedef void (*adc_cont_done_cb_t)(void* arg, bool is_buf_1);
typedef void (*adc_cont_err_cb_t)(void* arg, bool is_buf_1, uint16_t num_of_items_left);
// arg is user passed in data. is_buf_1 represents whether the DMA controller is in the first buffer or not.
// num_of_items_left is the number of elements that were remaining to be transmitted when the error occurred


// External ADC channels
typedef enum : uint8_t {
    ADC_CHANNEL_0,  // PA0
    ADC_CHANNEL_1,  // PA1
    ADC_CHANNEL_2,  // PA2
    ADC_CHANNEL_3,  // PA3
    ADC_CHANNEL_4,  // PA4
    ADC_CHANNEL_5,  // PA5
    ADC_CHANNEL_6,  // PA6
    ADC_CHANNEL_7,  // PA7
    ADC_CHANNEL_8,  // PB0
    ADC_CHANNEL_9,  // PB1
    ADC_CHANNEL_10, // PC0
    ADC_CHANNEL_11, // PC1
    ADC_CHANNEL_12, // PC2
    ADC_CHANNEL_13, // PC3
    ADC_CHANNEL_14, // PC4
    ADC_CHANNEL_15, // PC5
} adc_channels_t;


// The ADC resolution. Each extra resolution adds an extra ADCCLK cycles when sampling
typedef enum : uint8_t {
    ADC_RES_6_BITS  = 0b11, // ADC resolution of 6 bits
    ADC_RES_8_BITS  = 0b10, // ADC resolution of 8 bits
    ADC_RES_10_BITS = 0b01, // ADC resolution of 10 bits
    ADC_RES_12_BITS = 0b00, // ADC resolution of 12 bits
} adc_resolution_t;


// Prescaler to divide the clock of the APB2 bus to feed the ADCCLK
typedef enum : uint8_t {
    ADC_CLK_PRESCALER_2 = 0b00, // ADCCLK = (APB2 / 2)
    ADC_CLK_PRESCALER_4 = 0b01, // ADCCLK = (APB2 / 4)
    ADC_CLK_PRESCALER_6 = 0b10, // ADCCLK = (APB2 / 6)
    ADC_CLK_PRESCALER_8 = 0b11, // ADCCLK = (APB2 / 8)
} adc_prescaler_t;


// The number of ADCCLK clock cycles that a conversion lasts. This only
// affects the external channels. The internal channels require a much
// longer sampling time than the internal channels. Those all use 480 cycles.
typedef enum : uint8_t {
    ADC_SAMPLE_3_CYCLES   = 0b000, // ADC sampling time is 3 cycles
    ADC_SAMPLE_15_CYCLES  = 0b001, // ADC sampling time is 15 cycles
    ADC_SAMPLE_28_CYCLES  = 0b010, // ADC sampling time is 28 cycles
    ADC_SAMPLE_56_CYCLES  = 0b011, // ADC sampling time is 56 cycles
    ADC_SAMPLE_84_CYCLES  = 0b100, // ADC sampling time is 84 cycles
    ADC_SAMPLE_112_CYCLES = 0b101, // ADC sampling time is 112 cycles
    ADC_SAMPLE_144_CYCLES = 0b110, // ADC sampling time is 144 cycles
    ADC_SAMPLE_480_CYCLES = 0b111, // ADC sampling time is 480 cycles
} adc_sample_cycles_t;


// How the data will be aligned in the data register after conversion
typedef enum : uint8_t {
    ADC_RIGHT_ALIGN = 0b0,
    ADC_LEFT_ALIGN  = 0b1,
} adc_align_t;


// The various trigger sources for channels in the regular group
typedef enum : uint8_t {
    RG_TRIGGER_TIM1_CH1     = 0b0000,
    RG_TRIGGER_TIM1_CH2     = 0b0001,
    RG_TRIGGER_TIM1_CH3     = 0b0010,
    RG_TRIGGER_TIM2_CH2     = 0b0011,
    RG_TRIGGER_TIM2_CH3     = 0b0100,
    RG_TRIGGER_TIM2_CH4     = 0b0101,
    RG_TRIGGER_TIM2_TRGO    = 0b0110,
    RG_TRIGGER_TIM3_CH1     = 0b0111,
    RG_TRIGGER_TIM3_TRGO    = 0b1000,
    RG_TRIGGER_TIM4_CH4     = 0b1001,
    RG_TRIGGER_TIM5_CH1     = 0b1010,
    RG_TRIGGER_TIM5_CH2     = 0b1011,
    RG_TRIGGER_TIM5_CH3     = 0b1100,
    RG_TRIGGER_SOFTWARE     = 0b1101,
    RG_TRIGGER_EXTI_LINE_11 = 0b1111,
} adc_regular_group_trigger_t;


// The various trigger sources for channels in the injected group
typedef enum : uint8_t {
    JG_TRIGGER_TIM1_CH4     = 0b0000,
    JG_TRIGGER_TIM1_TRGO    = 0b0001,
    JG_TRIGGER_TIM2_CH1     = 0b0010,
    JG_TRIGGER_TIM2_TRGO    = 0b0011,
    JG_TRIGGER_TIM3_CH2     = 0b0100,
    JG_TRIGGER_TIM3_CH4     = 0b0101,
    JG_TRIGGER_TIM4_CH1     = 0b0110,
    JG_TRIGGER_TIM4_CH2     = 0b0111,
    JG_TRIGGER_TIM4_CH3     = 0b1000,
    JG_TRIGGER_TIM4_TRGO    = 0b1001,
    JG_TRIGGER_TIM5_CH4     = 0b1010,
    JG_TRIGGER_TIM5_TRGO    = 0b1011,
    JG_TRIGGER_SOFTWARE     = 0b1101,
    JG_TRIGGER_EXTI_LINE_15 = 0b1111,
} adc_injected_group_trigger_t;


// The polarities of the would be trigger source
// It is ignored if the trigger is RG_TRIGGER_SOFTWARE or JG_TRIGGER_SOFTWARE
typedef enum : uint8_t {
    RISING_EDGE         = 0b01,
    FALLING_EDGE        = 0b10,
    RISING_FALLING_EDGE = 0b11,
} adc_trigger_polarity_t;


// Configure a given ADC peripheral instance
typedef struct {
    adc_align_t         alignment;
    adc_resolution_t    resolution;
    adc_sample_cycles_t sampling_cycles;
} adc_config_t;


// The sequence of channels and the number of channels
typedef struct {
    const adc_channels_t* channels_sequence;
    size_t                num_of_channels;
} adc_channels_config_t;


// The various callbacks that can be registered when using continuous DMA sampling
typedef struct {
    // Called when a buffer is filled
    adc_cont_done_cb_t on_buffer_full;

    // Error callbacks. The errors end the conversion.
    // Use adc_regular_group_cont_start_conv(...) to restart the conversion.
    adc_cont_err_cb_t on_transfer_error;    // Called on a DMA transfer error
    adc_cont_err_cb_t on_direct_mode_error; // Called when an error arose from the DMA direct mode.
    adc_cont_err_cb_t on_data_overrun;      // Called when the DMA is too slow to read and data is lost

    // User data to be passed to the callbacks
    void* user;
} adc_dma_callbacks_t;


// Configure a conversion in the regular group with continuous DMA sampling
typedef struct {
    // The channel sequence and the trigger source
    adc_channels_config_t       channels;
    adc_regular_group_trigger_t trigger;
    adc_trigger_polarity_t      trigger_polarity;

    // The buffer(s) to store the samples
    const uint16_t* buffer_1;
    const uint16_t* buffer_2;

    // DMA settings
    uint16_t        buffer_size;
    dma_priority_t  priority;
    dma_circ_mode_t circular_mode;

    // The different callbacks to be registered. They determine what interrupts will be enabled.
    adc_dma_callbacks_t callbacks;

} adc_continuous_config_t;


// Configure a conversion in the injected group
typedef struct {
    // The channel sequence and the trigger source
    adc_channels_config_t        channels;
    adc_injected_group_trigger_t trigger;
    adc_trigger_polarity_t       trigger_polarity;

    // The sample offsets to be fed to the offset registers
    uint16_t offsets[MAX_INJECTED_CHANNELS];

    // Called when conversion is finished. Pretty straightforward
    adc_callback_t on_conv_complete;
    void*          arg;
} adc_injected_group_config_t;


// Configure the analog watchdog to use for monitoring the voltages on the given channels
typedef struct {
    // The thresholds to use to monitor the regular and/or injected groups
    uint16_t min_adc_value;
    uint16_t max_adc_value;

    // Whether or not to monitor the regular and/or injected groups
    bool monitor_regular_channels;
    bool monitor_injected_channels;

    // Called when any of the values in min_adc_value and max_adc_value are violated
    adc_callback_t on_thresholds_violated;
    void*          arg;
} adc_analog_wdg_config_t;


#endif // ADC_TYPES_H_