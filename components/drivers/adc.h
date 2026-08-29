#ifndef ADC_H_
#define ADC_H_


#ifdef __cplusplus
extern "C" {
#endif


#include "stm32f411xe.h"
#include "utils/err.h"

#include <stdint.h>
#include <stddef.h>


// Shape of all callbacks passed to the ADC driver
typedef void (*adc_callback_t)(void* arg);
typedef void (*adc_cont_done_cb_t)(void* arg, bool is_buf_1);

typedef enum : uint8_t {
    // External ADC channels
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

typedef enum : uint8_t {
    ADC_RES_6_BITS  = 0b11, // ADC resolution of 6 bits
    ADC_RES_8_BITS  = 0b10, // ADC resolution of 8 bits
    ADC_RES_10_BITS = 0b01, // ADC resolution of 10 bits
    ADC_RES_12_BITS = 0b00, // ADC resolution of 12 bits
} adc_resolution_t;

typedef enum : uint8_t {
    ADC_CLK_PRESCALER_2 = 0b00, // ADCCLK = (APB2 / 2)
    ADC_CLK_PRESCALER_4 = 0b01, // ADCCLK = (APB2 / 4)
    ADC_CLK_PRESCALER_6 = 0b10, // ADCCLK = (APB2 / 6)
    ADC_CLK_PRESCALER_8 = 0b11, // ADCCLK = (APB2 / 8)
} adc_prescaler_t;

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

typedef enum : uint8_t {
    ADC_RIGHT_ALIGN = 0b0,
    ADC_LEFT_ALIGN  = 0b1,
} adc_align_t;

typedef enum : uint8_t {
    REG_TRIGGER_TIM1_CH1     = 0b0000,
    REG_TRIGGER_TIM1_CH2     = 0b0001,
    REG_TRIGGER_TIM1_CH3     = 0b0010,
    REG_TRIGGER_TIM2_CH2     = 0b0011,
    REG_TRIGGER_TIM2_CH3     = 0b0100,
    REG_TRIGGER_TIM2_CH4     = 0b0101,
    REG_TRIGGER_TIM2_TRGO    = 0b0110,
    REG_TRIGGER_TIM3_CH1     = 0b0111,
    REG_TRIGGER_TIM3_TRGO    = 0b1000,
    REG_TRIGGER_TIM4_CH4     = 0b1001,
    REG_TRIGGER_TIM5_CH1     = 0b1010,
    REG_TRIGGER_TIM5_CH2     = 0b1011,
    REG_TRIGGER_TIM5_CH3     = 0b1100,
    REG_TRIGGER_SOFTWARE     = 0b1101,
    REG_TRIGGER_EXTI_LINE_11 = 0b1111,
} adc_regular_group_trigger_t;

typedef enum : uint8_t {
    INJ_TRIGGER_TIM1_CH4     = 0b0000,
    INJ_TRIGGER_TIM1_TRGO    = 0b0001,
    INJ_TRIGGER_TIM2_CH1     = 0b0010,
    INJ_TRIGGER_TIM2_TRGO    = 0b0011,
    INJ_TRIGGER_TIM3_CH2     = 0b0100,
    INJ_TRIGGER_TIM3_CH4     = 0b0101,
    INJ_TRIGGER_TIM4_CH1     = 0b0110,
    INJ_TRIGGER_TIM4_CH2     = 0b0111,
    INJ_TRIGGER_TIM4_CH3     = 0b1000,
    INJ_TRIGGER_TIM4_TRGO    = 0b1001,
    INJ_TRIGGER_TIM5_CH4     = 0b1010,
    INJ_TRIGGER_TIM5_TRGO    = 0b1011,
    INJ_TRIGGER_SOFTWARE     = 0b1101,
    INJ_TRIGGER_EXTI_LINE_15 = 0b1111,
} adc_injected_group_trigger_t;

typedef struct {
    adc_align_t         alignment;
    adc_resolution_t    resolution;
    adc_sample_cycles_t sampling_cycles;
} adc_config_t;

typedef struct {
    // The sequence of channels and the number of channels
    const adc_channels_t* channels_sequence;
    size_t                num_of_channels;
} adc_channels_config_t;

typedef struct {
    adc_cont_done_cb_t on_buffer_full;       // Called when a buffer is filled
    adc_cont_done_cb_t on_transfer_error;    // Called on a DMA transfer error
    adc_cont_done_cb_t on_direct_mode_error; // Called when an error arose from the DMA direct mode
    adc_cont_done_cb_t on_data_overrun;      // Called when the DMA is too slow to read and data is lost
    // User data to be passed to the callbacks
    void* arg;
} adc_dma_callbacks_t;

// Configuration data for ADC continuous mode, injected group and the analog watchdog
typedef struct {
    // The channel sequence and the trigger source
    adc_channels_config_t       channels;
    adc_regular_group_trigger_t trigger;

    // The buffer(s) to store the samples
    const uint16_t* buffer_1;
    const uint16_t* buffer_2;
    uint16_t        buffer_size;

    // The different callbacks to be registered. They determine what interrupts will be enabled.
    adc_dma_callbacks_t callbacks;

    // DMA settings
    bool use_double_buffers;       // Double buffering mode. Pretty straightforward
    bool dma_wraparound_when_done; // This is only used when use_double_buffers is false. This tells the DMA controller
                                   // to wrap around to the start of the buffer if it gets to the end of the buffer.
} adc_continuous_config_t;

typedef struct {
    // The channel sequence and the trigger source
    adc_channels_config_t        channels;
    adc_injected_group_trigger_t trigger;

    // The sample offsets to be fed to the offset registers
    uint16_t offset_1;
    uint16_t offset_2;
    uint16_t offset_3;
    uint16_t offset_4;

    // Called when conversion is finished. Pretty straightforward
    adc_callback_t on_conv_complete;
    void*          arg;
} adc_injected_group_config_t;

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


// Configure a specific ADC peripheral
hal_err_t adcx_clk_enable(ADC_TypeDef* handle, bool enable);
hal_err_t adc_configure(ADC_TypeDef* handle, const adc_config_t* config);
hal_err_t adc_power_on(ADC_TypeDef* handle, bool on);


// General control of all the ADCs. These functions affect all the ADC peripheral instances
void adc_clk_configure(adc_prescaler_t clk_prescaler);
void adc_enable_nvic_irq(bool enable);
void adc_power_on_temp_sensor(bool on);


// Number of regular and injected channels supported by the ADC peripheral
#define MAX_REGULAR_CHANNELS (16)
#define MAX_INJECTED_CHANNELS (4)


// For use with the external channels
// The regular group with oneshot polling mode
hal_err_t adc_regular_group_get_oneshot(ADC_TypeDef* handle, adc_channels_t channel, uint16_t* raw_data);

// The regular group with continuous DMA mode
hal_err_t adc_regular_group_cont_start_conv(ADC_TypeDef* handle, const adc_continuous_config_t* config);
hal_err_t adc_regular_group_cont_end_conv(ADC_TypeDef* handle);

// The injected group with interrupts
hal_err_t adc_injected_group_start_conv(ADC_TypeDef* handle, const adc_injected_group_config_t* config);
hal_err_t adc_injected_group_get_result(ADC_TypeDef* handle, uint16_t* raw_data_buffer, size_t buffer_size);


// For operation of the internal channels.
// Only oneshot and conversion with the regular group are supported.
// These only return the raw ADC values. Pair with adc_get_value_right_aligned(...)
// (if you use ADC_RIGHT_ALIGN) to get the actual voltages read by thr ADC.
hal_err_t adc_get_v_bat(ADC_TypeDef* handle, uint16_t* raw_data);
hal_err_t adc_get_temperature(ADC_TypeDef* handle, uint16_t* raw_data);
hal_err_t adc_get_v_ref_internal(ADC_TypeDef* handle, uint16_t* raw_data);


// Get the actual voltage readings from the ADC
hal_err_t adc_get_vdda(ADC_TypeDef* handle, float* vdda);
hal_err_t adc_get_temp_celsius(ADC_TypeDef* handle, float* temp_celsius);
hal_err_t adc_get_value_right_aligned(ADC_TypeDef* handle, uint16_t raw_data, adc_resolution_t resolution, float* voltage);


// For control of the analog watchdog control
hal_err_t adc_analog_wdg_start(ADC_TypeDef* handle, const adc_analog_wdg_config_t* config);
hal_err_t adc_analog_wdg_stop(ADC_TypeDef* handle);


#ifdef __cplusplus
}
#endif


#endif // ADC_H_