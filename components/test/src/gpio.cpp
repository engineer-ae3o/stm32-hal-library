#include "stm32f411xe.h"
#include "Unity/unity.h"

#include "drivers/gpio.h"
#include "utils/common.h"
#include "utils/log.h"

#include <array>
#include <cstdint>


namespace test::gpio {

    namespace {

        constexpr const char* TAG = "GPIO_Test";

        // A scratch pin/port used for tests that need to actually drive and read back a level.
        // Chosen arbitrarily; nothing on this board ties it to a fixed function.
        GPIO_TypeDef* const  SCRATCH_PORT = GPIOA;
        constexpr gpio_pin_t SCRATCH_PIN  = GPIO_PIN_8;

        const std::array<GPIO_TypeDef*, 6> ALL_PORTS{GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOH};

        constexpr std::array<gpio_pin_t, 16> ALL_PINS{
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

        uint32_t moder_bits(GPIO_TypeDef* port, gpio_pin_t pin) {
            return (port->MODER >> (pin * 2)) & 0b11UL;
        }

        uint32_t pupdr_bits(GPIO_TypeDef* port, gpio_pin_t pin) {
            return (port->PUPDR >> (pin * 2)) & 0b11UL;
        }

        void reset_port(GPIO_TypeDef* port) {
            port->MODER   = 0;
            port->PUPDR   = 0;
            port->OTYPER  = 0;
            port->OSPEEDR = 0;
            port->AFR[0]  = 0;
            port->AFR[1]  = 0;
        }

    } // namespace

    void clk_enable_toggles_only_the_targeted_port() {
        for (auto* port : ALL_PORTS) {
            TEST_ASSERT_EQUAL(HAL_OK, gpiox_clk_enable(port, true));
            TEST_ASSERT_EQUAL(HAL_OK, gpiox_clk_enable(port, false));
        }

        // An unrecognized port pointer must be rejected without touching RCC at all
        const uint32_t before = RCC->AHB1ENR;
        TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, gpiox_clk_enable(nullptr, true));
        TEST_ASSERT_EQUAL_UINT32(before, RCC->AHB1ENR);
    }

    void mode_setters_touch_only_their_own_pin() {
        reset_port(SCRATCH_PORT);

        for (const auto pin : ALL_PINS) {
            gpio_set_output(SCRATCH_PORT, pin);
            TEST_ASSERT_EQUAL_UINT32(0b01U, moder_bits(SCRATCH_PORT, pin));

            gpio_set_input(SCRATCH_PORT, pin);
            TEST_ASSERT_EQUAL_UINT32(0b00U, moder_bits(SCRATCH_PORT, pin));

            gpio_set_analog(SCRATCH_PORT, pin);
            TEST_ASSERT_EQUAL_UINT32(0b11U, moder_bits(SCRATCH_PORT, pin));
            gpio_set_input(SCRATCH_PORT, pin);

            // Setting one pin must never disturb its neighbours
            gpio_set_output(SCRATCH_PORT, pin);
            for (const auto other : ALL_PINS) {
                if (other == pin) {
                    continue;
                }
                TEST_ASSERT_EQUAL_UINT32(0b00U, moder_bits(SCRATCH_PORT, other));
            }
            gpio_set_input(SCRATCH_PORT, pin);
        }

        reset_port(SCRATCH_PORT);
    }

    void alternate_function_selects_the_right_afr_half_and_masks_the_value() {
        reset_port(SCRATCH_PORT);

        for (const auto pin : ALL_PINS) {
            // alt_val deliberately exceeds 4 bits to verify the driver masks it to & 0xF
            TEST_ASSERT_EQUAL(HAL_OK, gpio_set_alternate_function(SCRATCH_PORT, pin, 0xABU));
            TEST_ASSERT_EQUAL_UINT32(0b10U, moder_bits(SCRATCH_PORT, pin));

            constexpr uint32_t EXPECTED_NIBBLE = 0xBU; // only the low nibble should survive
            if (pin <= GPIO_PIN_7) {
                TEST_ASSERT_EQUAL_UINT32(EXPECTED_NIBBLE, (SCRATCH_PORT->AFR[0] >> (pin * 4)) & 0xFUL);
            } else {
                TEST_ASSERT_EQUAL_UINT32(EXPECTED_NIBBLE, (SCRATCH_PORT->AFR[1] >> ((pin - 8) * 4)) & 0xFUL);
            }

            gpio_set_input(SCRATCH_PORT, pin);
        }

        TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, gpio_set_alternate_function(nullptr, GPIO_PIN_0, 0));

        reset_port(SCRATCH_PORT);
    }

    void pullup_and_pulldown_are_mutually_exclusive() {
        reset_port(SCRATCH_PORT);

        for (const auto pin : ALL_PINS) {
            gpio_enable_pullup(SCRATCH_PORT, pin, true);
            TEST_ASSERT_EQUAL_UINT32(0b01U, pupdr_bits(SCRATCH_PORT, pin));

            // Enabling the pulldown must replace the pullup bit, not OR into it
            gpio_enable_pulldown(SCRATCH_PORT, pin, true);
            TEST_ASSERT_EQUAL_UINT32(0b10U, pupdr_bits(SCRATCH_PORT, pin));

            gpio_enable_pulldown(SCRATCH_PORT, pin, false);
            TEST_ASSERT_EQUAL_UINT32(0b00U, pupdr_bits(SCRATCH_PORT, pin));

            gpio_enable_pullup(SCRATCH_PORT, pin, false);
            TEST_ASSERT_EQUAL_UINT32(0b00U, pupdr_bits(SCRATCH_PORT, pin));
        }

        reset_port(SCRATCH_PORT);
    }

    void output_type_and_speed_cover_every_enum_value() {
        reset_port(SCRATCH_PORT);

        gpio_set_output_type(SCRATCH_PORT, SCRATCH_PIN, GPIO_OPEN_DRAIN);
        TEST_ASSERT_TRUE(SCRATCH_PORT->OTYPER & (1UL << SCRATCH_PIN));

        gpio_set_output_type(SCRATCH_PORT, SCRATCH_PIN, GPIO_PUSH_PULL);
        TEST_ASSERT_FALSE(SCRATCH_PORT->OTYPER & (1UL << SCRATCH_PIN));

        constexpr std::array<gpio_speed_mode_t, 4> SPEEDS{GPIO_LOW_SPEED, GPIO_MEDIUM_SPEED, GPIO_FAST_SPEED, GPIO_HIGH_SPEED};
        for (const auto speed : SPEEDS) {
            gpio_set_speed_mode(SCRATCH_PORT, SCRATCH_PIN, speed);
            TEST_ASSERT_EQUAL_UINT32(static_cast<uint32_t>(speed), (SCRATCH_PORT->OSPEEDR >> (SCRATCH_PIN * 2)) & 0b11UL);
        }

        reset_port(SCRATCH_PORT);
    }

    void level_set_get_and_toggle_round_trip_through_the_pad() {
        reset_port(SCRATCH_PORT);
        gpio_set_output(SCRATCH_PORT, SCRATCH_PIN);
        gpio_set_output_type(SCRATCH_PORT, SCRATCH_PIN, GPIO_PUSH_PULL);

        // A push-pull output pin's own drive state is readable back on IDR via the pad
        gpio_level_set(SCRATCH_PORT, SCRATCH_PIN, true);
        TEST_ASSERT_TRUE(gpio_get_level(SCRATCH_PORT, SCRATCH_PIN));

        gpio_level_set(SCRATCH_PORT, SCRATCH_PIN, false);
        TEST_ASSERT_FALSE(gpio_get_level(SCRATCH_PORT, SCRATCH_PIN));

        gpio_level_toggle(SCRATCH_PORT, SCRATCH_PIN);
        TEST_ASSERT_TRUE(gpio_get_level(SCRATCH_PORT, SCRATCH_PIN));

        gpio_level_toggle(SCRATCH_PORT, SCRATCH_PIN);
        TEST_ASSERT_FALSE(gpio_get_level(SCRATCH_PORT, SCRATCH_PIN));

        // BSRR is a set/reset register - driving a neighbouring pin must not disturb this one
        gpio_set_output(SCRATCH_PORT, GPIO_PIN_9);
        gpio_level_set(SCRATCH_PORT, GPIO_PIN_9, true);
        TEST_ASSERT_FALSE(gpio_get_level(SCRATCH_PORT, SCRATCH_PIN));
        gpio_set_input(SCRATCH_PORT, GPIO_PIN_9);

        TEST_ASSERT_FALSE(gpio_get_level(nullptr, SCRATCH_PIN));

        reset_port(SCRATCH_PORT);
    }

    void interrupt_config_covers_every_port_code_and_every_edge() {
        struct port_code_t {
            GPIO_TypeDef* port;
            uint8_t       code;
        };
        const std::array<port_code_t, 6> PORTS{{
            {GPIOA, 0b000U},
            {GPIOB, 0b001U},
            {GPIOC, 0b010U},
            {GPIOD, 0b011U},
            {GPIOE, 0b100U},
            {GPIOH, 0b111U},
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
        constexpr std::array<edge_case_t, 4> EDGES{{
            {GPIO_NO_EDGE, false, false},
            {GPIO_RISING_EDGE_ONLY, true, false},
            {GPIO_FALLING_EDGE_ONLY, false, true},
            {GPIO_RISING_FALLING_EDGE, true, true},
        }};

        for (const auto& e : EDGES) {
            TEST_ASSERT_EQUAL(HAL_OK, gpio_set_interrupt(GPIOA, PIN, e.edge));
            TEST_ASSERT_EQUAL(e.rising, (EXTI->RTSR & (1UL << PIN)) != 0);
            TEST_ASSERT_EQUAL(e.falling, (EXTI->FTSR & (1UL << PIN)) != 0);
            TEST_ASSERT_TRUE(EXTI->IMR & (1UL << PIN));
            gpio_clear_interrupt(GPIOA, PIN);
        }

        TEST_ASSERT_EQUAL(HAL_ERR_INVALID_ARG, gpio_set_interrupt(nullptr, PIN, GPIO_NO_EDGE));
    }

    void clear_interrupt_fully_undoes_set_interrupt() {
        constexpr gpio_pin_t PIN     = GPIO_PIN_5;
        constexpr uint8_t    REG_IDX = PIN / 4;
        constexpr uint8_t    BIT_POS = (PIN % 4) * 4;

        TEST_ASSERT_EQUAL(HAL_OK, gpio_set_interrupt(GPIOB, PIN, GPIO_RISING_FALLING_EDGE));
        TEST_ASSERT_NOT_EQUAL(0, (SYSCFG->EXTICR[REG_IDX] >> BIT_POS) & 0xFUL);

        gpio_clear_interrupt(GPIOB, PIN);

        TEST_ASSERT_EQUAL_UINT32(0, (SYSCFG->EXTICR[REG_IDX] >> BIT_POS) & 0xFUL);
        TEST_ASSERT_FALSE(EXTI->RTSR & (1UL << PIN));
        TEST_ASSERT_FALSE(EXTI->FTSR & (1UL << PIN));
        TEST_ASSERT_FALSE(EXTI->IMR & (1UL << PIN));

        // A NULL port is documented (via the `if (port)` guard) as a safe no-op, not an error
        gpio_clear_interrupt(nullptr, PIN);
    }

    void all() {
        LOGI(TAG, "Starting the tests on the GPIO driver");

        RUN_TEST(clk_enable_toggles_only_the_targeted_port);
        RUN_TEST(mode_setters_touch_only_their_own_pin);
        RUN_TEST(alternate_function_selects_the_right_afr_half_and_masks_the_value);
        RUN_TEST(pullup_and_pulldown_are_mutually_exclusive);
        RUN_TEST(output_type_and_speed_cover_every_enum_value);
        RUN_TEST(level_set_get_and_toggle_round_trip_through_the_pad);
        RUN_TEST(interrupt_config_covers_every_port_code_and_every_edge);
        RUN_TEST(clear_interrupt_fully_undoes_set_interrupt);

        LOGI(TAG, "Done with all tests on the GPIO driver");
    }

} // namespace test::gpio
