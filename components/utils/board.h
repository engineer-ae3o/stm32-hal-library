#ifndef BOARD_H_
#define BOARD_H_


#ifdef __cplusplus
extern "C" {
#endif


// Board specific configuration details
// These are gotten from the datasheet and/or the TRM


#if defined(STM32F411xE)
// HSI and HSE clocks
#define HSE_VALUE_MHz (25U)
#define HSE_VALUE_Hz ((HSE_VALUE_MHz) * 1'000'000U)

#define HSI_VALUE_MHz (16U)
#define HSI_VALUE_Hz ((HSI_VALUE_MHz) * 1'000'000U)

#define MAX_SYSTEM_CLOCK_Hz (100'000'000U)
#define MAX_APB1_CLOCK_Hz (50'000'000U)
#define MAX_APB2_CLOCK_Hz (100'000'000U)
#define MAX_AUDIO_PLL_CLOCK_Hz (192'000'000U)

// ADC values to use the internal temperature sensor. Gotten from the datasheet
#define ADC_STARUP_TIME_US (4U)
#define VBAT_DIVIDER_RATIO (4U)

// Temperature sensor constants
#define TEMP_SENSOR_AVERAGE_SLOPE (2.5F)
#define TEMP_SENSOR_VSENSE_AT_25C (0.76F)
#define TEMP_SENSOR_MIN_SAMPLING_TIME_US (10U)
#define TEMP_SENSOR_STARUP_TIME_US (10U)

// Calibration data for internal ADC reference voltage and the temperature sensor
#define VREFINT_CALIBRATION_VALUE (uint16_t)(*(volatile uint16_t*)0x1FFF7A2AU)
#define TEMP_SENSOR_CALIB_30C_VALUE (uint16_t)(*(volatile uint16_t*)0x1FFF7A2CU)
#define TEMP_SENSOR_CALIB_110C_VALUE (uint16_t)(*(volatile uint16_t*)0x1FFF7A2EU)

// Bus rise times for I2C
#define I2C_TRISE_TIME_400kHz_ns (300U)
#define I2C_TRISE_TIME_100kHz_ns (1000U)
#else
#error "No board configuration given for current target."
#endif


#ifdef __cplusplus
}
#endif


#endif // BOARD_H_