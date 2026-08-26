#ifndef ADC_H_
#define ADC_H_


#ifdef __cplusplus
extern "C" {
#endif


#include "stm32f411xe.h"
#include "drivers/dma.h"
#include "utils/err.h"

#include <stdint.h>
#include <stddef.h>


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

// Shape of all the non-DMA callbacks to be registered with this driver
typedef void (*adc_callback_t)(void* arg);

// For use with the external channels
// The regular group with oneshot polling mode
hal_err_t adc_regular_group_get_oneshot(ADC_TypeDef* handle, adc_channels_t channel, uint16_t* raw_data);

// The regular group with continuous DMA mode
typedef struct {
    // The channels to be sampled
    adc_channels_config_t channels;

    // The DMA buffer to store the samples
    const uint16_t* buffer_1;
    const uint16_t* buffer_2;
    uint16_t        buffer_size;

    // DMA settings
    bool use_double_buffers;       // Double buffering mode. Pretty straightforward
    bool dma_wraparound_when_done; // This is only used when use_double_buffers is false. This tells the DMA controller
                                   // to wrap around to the start of the buffer if it gets to the buffer end.

    // The different callbacks to be registered. They determine what interrupts will be enabled.
    struct {
        void (*on_buffer_full)(void* arg, bool is_buf_1);       // Called when a buffer is filled
        void (*on_transfer_error)(void* arg, bool is_buf_1);    // Called on a DMA transfer error
        void (*on_direct_mode_error)(void* arg, bool is_buf_1); // Called when an error arose from the DMA direct mode
        void (*on_data_overrun)(void* arg, bool is_buf_1);      // Called when the DMA is too slow to read and data is lost
    } callbacks;

} adc_continuous_config_t;

hal_err_t adc_regular_group_cont_start_conv(ADC_TypeDef* handle, const adc_continuous_config_t* config, void* arg);
hal_err_t adc_regular_group_cont_end_conv(ADC_TypeDef* handle);

// The injected group with interrupts
hal_err_t adc_injected_group_start_conv(ADC_TypeDef* handle, const adc_channels_config_t* channels, adc_callback_t cb, void* arg);
hal_err_t adc_injected_group_get_result(ADC_TypeDef* handle, uint16_t* raw_data, size_t size);


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
hal_err_t adc_analog_wdg_start(ADC_TypeDef* handle, uint16_t min, uint16_t max, bool regular, bool injected, adc_callback_t cb, void* arg);
hal_err_t adc_analog_wdg_stop(ADC_TypeDef* handle);


#ifdef __cplusplus
}
#endif


#endif // ADC_H_