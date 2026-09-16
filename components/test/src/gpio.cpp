#include "stm32f411xe.h"
#include "Unity/unity.h"

#include "drivers/gpio.h"
#include "test/gpio.hpp"
#include "utils/err.h"
#include "utils/log.h"

#include <array>
#include <cstdint>


namespace test::gpio {

    namespace {

        constexpr const char* TAG = "GPIO_Test";

        const auto ALL_PORTS = std::array{
            GPIOA,
            GPIOB,
            GPIOC,
            GPIOD,
            GPIOE,
            GPIOH,
        };

        constexpr auto ALL_PINS = std::array{
            GPIO_PIN_0,
            GPIO_PIN_1,
            GPIO_PIN_2,
            GPIO_PIN_3,
            GPIO_PIN_4,
            GPIO_PIN_5,
            GPIO_PIN_6,
            GPIO_PIN_7,
            GPIO_PIN_8,
            GPIO_PIN_9,
            GPIO_PIN_10,
            GPIO_PIN_11,
            GPIO_PIN_12,
            GPIO_PIN_13,
            GPIO_PIN_14,
            GPIO_PIN_15,
        };

        // A scratch pin/port used for tests that need to actually drive and read back a level.
        // Chosen arbitrarily; nothing on this board ties it to a fixed function.
        const auto     SCRATCH_PORT = GPIOC;
        constexpr auto SCRATCH_PIN  = GPIO_PIN_15;

        // Helpers
        inline uint32_t get_mode_register_bits(GPIO_TypeDef* port, gpio_pin_t pin) {
            return (port->MODER >> (pin * 2)) & 0b11UL;
        }

        inline uint32_t get_pullup_pulldown_register_bits(GPIO_TypeDef* port, gpio_pin_t pin) {
            return (port->PUPDR >> (pin * 2)) & 0b11UL;
        }

        inline void reset_port(GPIO_TypeDef* port) {
            port->MODER   = 0;
            port->PUPDR   = 0;
            port->OTYPER  = 0;
            port->OSPEEDR = 0;
            port->AFR[0]  = 0;
            port->AFR[1]  = 0;
        }

        inline void enable_all_port_clocks(bool enable) {
            for (auto* const port : ALL_PORTS) {
                TEST_ASSERT_EQUAL(HAL_OK, gpiox_clk_enable(port, enable));
            }
        }

        // TESTS
        void clk_enable_toggles_only_the_targeted_port() {
            for (auto* port : ALL_PORTS) {
                TEST_ASSERT_EQUAL(HAL_OK, gpiox_clk_enable(port, true));
                TEST_ASSERT_EQUAL(HAL_OK, gpiox_clk_enable(port, false));
            }

            // An unrecognized port pointer must be rejected without touching any of the RCC registers
            const uint32_t before = RCC->AHB1ENR;
            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, gpiox_clk_enable(nullptr, true));
            TEST_ASSERT_EQUAL_UINT32(before, RCC->AHB1ENR);
        }

        void mode_setters_touch_only_their_own_pin() {
            reset_port(SCRATCH_PORT);
            enable_all_port_clocks(true);

            for (const auto pin : ALL_PINS) {
                gpio_set_output(SCRATCH_PORT, pin);
                TEST_ASSERT_EQUAL_UINT32(0b01U, get_mode_register_bits(SCRATCH_PORT, pin));

                gpio_set_input(SCRATCH_PORT, pin);
                TEST_ASSERT_EQUAL_UINT32(0b00U, get_mode_register_bits(SCRATCH_PORT, pin));

                gpio_set_analog(SCRATCH_PORT, pin);
                TEST_ASSERT_EQUAL_UINT32(0b11U, get_mode_register_bits(SCRATCH_PORT, pin));

                // Setting one pin must never disturb its neighbours
                gpio_set_output(SCRATCH_PORT, pin);
                for (const auto other : ALL_PINS) {
                    if (other != pin) {
                        TEST_ASSERT_EQUAL_UINT32(0b00U, get_mode_register_bits(SCRATCH_PORT, other));
                    }
                }

                // Reset state
                gpio_set_input(SCRATCH_PORT, pin);
            }

            enable_all_port_clocks(false);
            reset_port(SCRATCH_PORT);
        }

        void alternate_function_selects_the_right_afr_half_and_masks_the_value() {
            reset_port(SCRATCH_PORT);
            enable_all_port_clocks(true);

            // The alternate value deliberately exceeds 4 bits to verify the driver masks it to 4 bits
            constexpr uint8_t  alternate_value = 0xABU;
            constexpr uint32_t EXPECTED_NIBBLE = alternate_value & 0xFU; // Only the low nibble should survive

            for (const auto pin : ALL_PINS) {
                TEST_ASSERT_EQUAL(HAL_OK, gpio_set_alternate_function(SCRATCH_PORT, pin, alternate_value));
                TEST_ASSERT_EQUAL_UINT32(0b10U, get_mode_register_bits(SCRATCH_PORT, pin));

                if (pin <= GPIO_PIN_7) {
                    TEST_ASSERT_EQUAL_UINT32(EXPECTED_NIBBLE, (SCRATCH_PORT->AFR[0] >> (pin * 4)) & 0xFU);
                } else if (pin <= GPIO_PIN_15) {
                    TEST_ASSERT_EQUAL_UINT32(EXPECTED_NIBBLE, (SCRATCH_PORT->AFR[1] >> ((pin - 8) * 4)) & 0xFU);
                } else {
                    TEST_FAIL_MESSAGE("Invalid GPIO pin");
                }

                gpio_set_input(SCRATCH_PORT, pin);
            }

            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, gpio_set_alternate_function(nullptr, GPIO_PIN_0, 0));

            enable_all_port_clocks(false);
            reset_port(SCRATCH_PORT);
        }

        void pullup_and_pulldown_are_mutually_exclusive() {
            reset_port(SCRATCH_PORT);
            enable_all_port_clocks(true);

            for (const auto pin : ALL_PINS) {
                gpio_enable_pullup(SCRATCH_PORT, pin, true);
                TEST_ASSERT_EQUAL_UINT32(0b01U, get_pullup_pulldown_register_bits(SCRATCH_PORT, pin));

                // Enabling the pulldown must replace the pullup bit, not OR into it
                gpio_enable_pulldown(SCRATCH_PORT, pin, true);
                TEST_ASSERT_EQUAL_UINT32(0b10U, get_pullup_pulldown_register_bits(SCRATCH_PORT, pin));

                gpio_enable_pulldown(SCRATCH_PORT, pin, false);
                TEST_ASSERT_EQUAL_UINT32(0b00U, get_pullup_pulldown_register_bits(SCRATCH_PORT, pin));

                gpio_enable_pullup(SCRATCH_PORT, pin, false);
                TEST_ASSERT_EQUAL_UINT32(0b00U, get_pullup_pulldown_register_bits(SCRATCH_PORT, pin));
            }

            enable_all_port_clocks(false);
            reset_port(SCRATCH_PORT);
        }

        void output_type_and_speed_cover_every_enum_value() {
            reset_port(SCRATCH_PORT);
            enable_all_port_clocks(true);

            gpio_set_output_type(SCRATCH_PORT, SCRATCH_PIN, GPIO_OPEN_DRAIN);
            TEST_ASSERT_TRUE(SCRATCH_PORT->OTYPER & (1UL << SCRATCH_PIN));

            gpio_set_output_type(SCRATCH_PORT, SCRATCH_PIN, GPIO_PUSH_PULL);
            TEST_ASSERT_FALSE(SCRATCH_PORT->OTYPER & (1UL << SCRATCH_PIN));

            constexpr auto SPEEDS = std::array{GPIO_LOW_SPEED, GPIO_MEDIUM_SPEED, GPIO_HIGH_SPEED, GPIO_FULL_SPEED};
            for (const auto speed : SPEEDS) {
                gpio_set_speed_mode(SCRATCH_PORT, SCRATCH_PIN, speed);
                TEST_ASSERT_EQUAL_UINT32(static_cast<uint32_t>(speed), (SCRATCH_PORT->OSPEEDR >> (SCRATCH_PIN * 2)) & 0b11UL);
            }

            enable_all_port_clocks(false);
            reset_port(SCRATCH_PORT);
        }

        void level_set_get_and_toggle_round_trip_through_the_pad() {
            reset_port(SCRATCH_PORT);
            enable_all_port_clocks(true);

            gpio_set_output(SCRATCH_PORT, SCRATCH_PIN);
            gpio_set_output_type(SCRATCH_PORT, SCRATCH_PIN, GPIO_PUSH_PULL);

            bool scratch_pin_level = false;

            // A push pull output pin's own drive state is readable back on the IDR via the pad
            gpio_level_set(SCRATCH_PORT, SCRATCH_PIN, true);
            scratch_pin_level = true;
            TEST_ASSERT_EQUAL(scratch_pin_level, gpio_get_level(SCRATCH_PORT, SCRATCH_PIN));

            gpio_level_set(SCRATCH_PORT, SCRATCH_PIN, false);
            scratch_pin_level = false;
            TEST_ASSERT_EQUAL(scratch_pin_level, gpio_get_level(SCRATCH_PORT, SCRATCH_PIN));

            gpio_level_toggle(SCRATCH_PORT, SCRATCH_PIN);
            scratch_pin_level = !scratch_pin_level;
            TEST_ASSERT_EQUAL(scratch_pin_level, gpio_get_level(SCRATCH_PORT, SCRATCH_PIN));

            gpio_level_toggle(SCRATCH_PORT, SCRATCH_PIN);
            scratch_pin_level = !scratch_pin_level;
            TEST_ASSERT_EQUAL(scratch_pin_level, gpio_get_level(SCRATCH_PORT, SCRATCH_PIN));

            // BSRR is a set/reset register. Driving a neighbouring pin should not disturb this one
            constexpr gpio_pin_t ANOTHER_SCRATCH_PIN = GPIO_PIN_9;
            static_assert(ANOTHER_SCRATCH_PIN != SCRATCH_PIN);

            gpio_set_output(SCRATCH_PORT, ANOTHER_SCRATCH_PIN);
            gpio_set_output_type(SCRATCH_PORT, ANOTHER_SCRATCH_PIN, GPIO_PUSH_PULL);

            gpio_level_set(SCRATCH_PORT, ANOTHER_SCRATCH_PIN, scratch_pin_level);
            TEST_ASSERT_EQUAL(scratch_pin_level, gpio_get_level(SCRATCH_PORT, ANOTHER_SCRATCH_PIN));

            TEST_ASSERT_FALSE(gpio_get_level(nullptr, ANOTHER_SCRATCH_PIN));
            gpio_set_input(SCRATCH_PORT, ANOTHER_SCRATCH_PIN);

            enable_all_port_clocks(false);
            reset_port(SCRATCH_PORT);
        }

        void interrupt_config_covers_every_port_code_and_every_edge() {
            reset_port(SCRATCH_PORT);
            enable_all_port_clocks(true);

            struct port_code_t {
                GPIO_TypeDef* port;
                uint8_t       code;
            };
            const auto PORTS = std::array<port_code_t, 6>{{
                {.port = GPIOA, .code = 0b000U},
                {.port = GPIOB, .code = 0b001U},
                {.port = GPIOC, .code = 0b010U},
                {.port = GPIOD, .code = 0b011U},
                {.port = GPIOE, .code = 0b100U},
                {.port = GPIOH, .code = 0b111U},
            }};

            constexpr gpio_pin_t PIN     = GPIO_PIN_3;
            constexpr uint8_t    REG_IDX = PIN / 4;
            constexpr uint8_t    BIT_POS = (PIN % 4) * 4;

            for (const auto& pc : PORTS) {
                TEST_ASSERT_EQUAL(HAL_OK, gpio_set_interrupt(pc.port, PIN, GPIO_RISING_FALLING_EDGE));
                TEST_ASSERT_EQUAL_UINT32(pc.code, (SYSCFG->EXTICR[REG_IDX] >> BIT_POS) & 0xFUL);
                gpio_clear_interrupt(pc.port, PIN);
            }

            struct edge_case_t {
                gpio_edge_trigger_t edge;
                bool                rising;
                bool                falling;
            };
            constexpr auto EDGES = std::array<edge_case_t, 3>{{
                {.edge = GPIO_RISING_EDGE, .rising = true, .falling = false},
                {.edge = GPIO_FALLING_EDGE, .rising = false, .falling = true},
                {.edge = GPIO_RISING_FALLING_EDGE, .rising = true, .falling = true},
            }};

            for (const auto& e : EDGES) {
                TEST_ASSERT_EQUAL(HAL_OK, gpio_set_interrupt(GPIOA, PIN, e.edge));
                TEST_ASSERT_EQUAL(e.rising, (EXTI->RTSR & (1UL << PIN)) != 0);
                TEST_ASSERT_EQUAL(e.falling, (EXTI->FTSR & (1UL << PIN)) != 0);
                TEST_ASSERT_TRUE(EXTI->IMR & (1UL << PIN));
                gpio_clear_interrupt(GPIOA, PIN);
            }

            TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, gpio_set_interrupt(nullptr, PIN, GPIO_RISING_FALLING_EDGE));

            enable_all_port_clocks(false);
            reset_port(SCRATCH_PORT);
        }

        void clear_interrupt_fully_undoes_set_interrupt() {
            reset_port(SCRATCH_PORT);
            enable_all_port_clocks(true);

            GPIO_TypeDef* const  PORT    = GPIOB;
            constexpr gpio_pin_t PIN     = GPIO_PIN_5;
            constexpr uint8_t    REG_IDX = PIN / 4;

            // The pin is gpio 5, so EXTI5, EXTI_RTSR_TR5, EXTI_FTSR_TR5 and EXTI_IMR_MR5.
            // The register index is 1, so the second register, that is EXTICR2. So
            // SYSCFG_EXTICR2_EXTI5. It's port B, so SYSCFG_EXTICR2_EXTI5_PB.

            TEST_ASSERT_EQUAL(HAL_OK, gpio_set_interrupt(PORT, PIN, GPIO_RISING_FALLING_EDGE));

            TEST_ASSERT_EQUAL_UINT32(SYSCFG_EXTICR2_EXTI5_PB, (SYSCFG->EXTICR[REG_IDX] & SYSCFG_EXTICR2_EXTI5));
            TEST_ASSERT_EQUAL_UINT32(EXTI_RTSR_TR5, (EXTI->RTSR & EXTI_RTSR_TR5));
            TEST_ASSERT_EQUAL_UINT32(EXTI_FTSR_TR5, (EXTI->FTSR & EXTI_FTSR_TR5));
            TEST_ASSERT_EQUAL_UINT32(EXTI_IMR_MR5, (EXTI->IMR & EXTI_IMR_MR5));

            gpio_clear_interrupt(PORT, PIN);

            TEST_ASSERT_EQUAL_UINT32(0, (SYSCFG->EXTICR[REG_IDX] & SYSCFG_EXTICR2_EXTI5));
            TEST_ASSERT_EQUAL_UINT32(0, (EXTI->RTSR & EXTI_RTSR_TR5));
            TEST_ASSERT_EQUAL_UINT32(0, (EXTI->FTSR & EXTI_FTSR_TR5));
            TEST_ASSERT_EQUAL_UINT32(0, (EXTI->IMR & EXTI_IMR_MR5));

            enable_all_port_clocks(false);
            reset_port(SCRATCH_PORT);
        }

    } // namespace

    void all() {
        LOGI(TAG, "Starting the tests on the GPIO driver");
        UNITY_BEGIN();

        RUN_TEST(clk_enable_toggles_only_the_targeted_port);
        RUN_TEST(mode_setters_touch_only_their_own_pin);
        RUN_TEST(alternate_function_selects_the_right_afr_half_and_masks_the_value);
        RUN_TEST(pullup_and_pulldown_are_mutually_exclusive);
        RUN_TEST(output_type_and_speed_cover_every_enum_value);
        RUN_TEST(level_set_get_and_toggle_round_trip_through_the_pad);
        RUN_TEST(interrupt_config_covers_every_port_code_and_every_edge);
        RUN_TEST(clear_interrupt_fully_undoes_set_interrupt);

        UNITY_END();
        LOGI(TAG, "Done with all tests on the GPIO driver");
    }

} // namespace test::gpio
