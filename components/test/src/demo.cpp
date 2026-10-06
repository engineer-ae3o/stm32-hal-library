#include "drivers/timer.h"
#include "utils/common.h"
#include "test/demo.hpp"
#include "drivers/pwm.h"
#include "utils/board.h"
#include "utils/tick.h"
#include "utils/err.h"
#include "utils/log.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>


namespace demo {

    namespace {

        constexpr const char* TAG = "PWM_Demo";

        enum class wave_form_t : uint8_t {
            TRIANGLE,  // 0 -> max -> 0, cycle = 2 * steps_per_ramp steps
            RAMP_UP,   // 0 -> max, jump back to 0, cycle = steps_per_ramp + 1 steps
            RAMP_DOWN, // max -> 0, jump back to max, cycle = steps_per_ramp + 1 steps
        };

        struct complementary_channel_t {
            bool        invert_output     = false;
            bool        output_idle_state = false;
            board_pin_t gpio_pin{};
        };

        // Every parameter of the demo. Aggregate: override with designated initializers, in declaration order:
        //   pwm_sweep({.frequency_hz = 5'000, .dead_time_ns = 2'000, .clk_div = TIM_CLK_DIV_1, .lag_steps = -10});
        struct swwep_config_t {
            // Timer
            TIM_TypeDef*     timer          = TIM1; // Must be an advanced timer
            uint32_t         frequency_hz   = 20'000U;
            uint32_t         dead_time_ns   = 1000U;
            pwm_clk_div_t    clk_div        = TIM_CLK_DIV_4; // DIV_1 tops out near 10 us of dead time at 100MHz
            pwm_mode_t       pwm_mode       = PWM_MODE_1;
            pwm_count_mode_t count_mode     = PWM_CENTER_ALIGNED_MODE_3;
            uint8_t          repetition_cnt = 10;
            bool             ossr           = true;
            bool             ossi           = true;

            pwm_write_protection_t wp_level = WP_OFF;

            // Channels
            // channels[0..num_channels) are driven, in this order. Lag is applied by position in this list.
            std::array<pwm_channel_config_t, MAX_TIM1_CHANNELS> channels     = {{
                {.invert_output = false, .output_idle_state = false, .channel = TIMER_CHANNEL_1, .gpio_pin = BOARD_TIM1_CH1_PA8},
                {.invert_output = false, .output_idle_state = false, .channel = TIMER_CHANNEL_2, .gpio_pin = BOARD_TIM1_CH2_PA9},
                {.invert_output = false, .output_idle_state = false, .channel = TIMER_CHANNEL_3, .gpio_pin = BOARD_TIM1_CH3_PA10},
                {.invert_output = false, .output_idle_state = false, .channel = TIMER_CHANNEL_4, .gpio_pin = BOARD_TIM1_CH4_PA11},
            }};
            size_t                                              num_channels = MAX_TIM1_CHANNELS;

            bool use_complementary_channels = true;
            // Indexed by channel number (CH1N..CH3N), as the driver does. CH4 has none.
            std::array<complementary_channel_t, MAX_TIM1_COMPLEMENTARY_CHANNELS> complementary_channels = {{
                {.invert_output = false, .output_idle_state = false, .gpio_pin = BOARD_TIM1_CH1N_PB13},
                {.invert_output = false, .output_idle_state = false, .gpio_pin = BOARD_TIM1_CH2N_PB14},
                {.invert_output = false, .output_idle_state = false, .gpio_pin = BOARD_TIM1_CH3N_PB15},
            }};

            // Break input
            bool        use_break_input  = true;
            bool        break_active_low = true;
            bool        break_auto_rearm = true;
            board_pin_t break_pin        = BOARD_TIM1_BKIN_PA6;
            timer_cb_t  break_callback   = {
                .cb =
                    [](void*, timer_channel_t) {
                        LOGW(TAG, "Break event detected");
                    },
                .arg = nullptr,
            };

            // Sweep
            wave_form_t waveform       = wave_form_t::TRIANGLE;
            uint32_t    steps_per_ramp = 100U; // Resolution of the sweep
            uint32_t    step_ms        = 20U;
            uint32_t    min_permille   = 0U;    // Sweep floor, 0..1000
            uint32_t    max_permille   = 1000U; // Sweep ceiling, 0..1000
            uint32_t    cycles         = 0U;    // 0 = run forever

            // Steps each channel trails the previous one. Negative = leads. nullopt = spread evenly over one cycle.
            std::optional<int32_t> lag_steps = std::nullopt;
        };

        uint32_t cycle_steps(const swwep_config_t& c) {
            return (c.waveform == wave_form_t::TRIANGLE) ? (2U * c.steps_per_ramp) : (c.steps_per_ramp + 1U);
        }

        // Shape of one cycle: phase in [0, cycle_steps) -> [0, steps_per_ramp]
        uint32_t shape(const swwep_config_t& c, uint32_t phase) {
            const uint32_t n = c.steps_per_ramp;
            switch (c.waveform) {
                case wave_form_t::TRIANGLE:
                    return (phase < n) ? phase : (2U * n - phase);
                case wave_form_t::RAMP_UP:
                    return phase;
                case wave_form_t::RAMP_DOWN:
                    return n - phase;
            }
            return 0;
        }

        int64_t lag_for(const swwep_config_t& c) {
            return c.lag_steps.has_value() ? *c.lag_steps : static_cast<int64_t>(cycle_steps(c) / c.num_channels);
        }

        uint32_t duty_counts(const swwep_config_t& c, uint32_t max_duty, uint32_t step, size_t index) {
            const int64_t  cycle  = cycle_steps(c);
            const int64_t  offset = ((static_cast<int64_t>(index) * lag_for(c)) % cycle + cycle) % cycle; // Normalized to [0, cycle)
            const uint32_t phase  = static_cast<uint32_t>((static_cast<int64_t>(step) + cycle - offset) % cycle);

            const uint64_t n    = c.steps_per_ramp;
            const uint64_t span = c.max_permille - c.min_permille;
            // Duty in permille * n units, so the whole thing stays in integers
            const uint64_t scaled = static_cast<uint64_t>(c.min_permille) * n + span * shape(c, phase);
            return static_cast<uint32_t>((static_cast<uint64_t>(max_duty) * scaled) / (1000ULL * n));
        }

        hal_err_t validate(const swwep_config_t& c) {
            if (c.timer == nullptr || c.frequency_hz == 0 || c.steps_per_ramp == 0 || c.step_ms == 0 || c.min_permille > c.max_permille ||
                c.max_permille > 1000U || c.num_channels == 0 || c.num_channels > c.channels.size()) {
                return HAL_ERR_INVALID_ARG;
            }

            // Channels must be unique, and the complementary table is indexed by channel number
            uint32_t seen = 0;
            for (size_t i = 0; i < c.num_channels; i++) {
                const uint32_t bit = 1U << static_cast<uint32_t>(c.channels[i].channel);
                if (c.channels[i].channel > TIMER_CHANNEL_4 || (seen & bit)) {
                    return HAL_ERR_INVALID_ARG;
                }
                seen |= bit;
            }

            // CCR is preloaded and latched only on update events: every (RCR + 1) overflows/underflows.
            // Center aligned has two per period, edge aligned has one.
            const bool     center = (static_cast<uint32_t>(c.count_mode) & TIM_CR1_CMS) != 0;
            const uint64_t update_us =
                ((c.repetition_cnt + 1ULL) * 1'000'000ULL + (center ? 2ULL : 1ULL) * c.frequency_hz - 1) / ((center ? 2ULL : 1ULL) * c.frequency_hz);
            if (static_cast<uint64_t>(c.step_ms) * 1000ULL <= update_us) {
                LOGE(TAG, "Step of %lums is not slower than the %luus PWM update interval", c.step_ms, static_cast<uint32_t>(update_us));
                return HAL_ERR_INVALID_ARG;
            }

            // Dead time delays only the rising edge, so the main output needs duty > DT/T and the complementary one duty < 1 - DT/T
            const uint64_t period_ns = 1'000'000'000ULL / c.frequency_hz;
            const uint64_t dead_pm   = (static_cast<uint64_t>(c.dead_time_ns) * 1000ULL) / period_ns;
            if (c.use_complementary_channels && (c.min_permille < dead_pm || c.max_permille > 1000U - dead_pm)) {
                LOGW(TAG,
                     "Dead time is %lu permille of the period: one output of each pair vanishes outside %lu..%lu permille",
                     static_cast<uint32_t>(dead_pm),
                     static_cast<uint32_t>(dead_pm),
                     static_cast<uint32_t>(1000U - dead_pm));
            }

            return HAL_OK;
        }

        hal_err_t apply_duties(const swwep_config_t& c, uint32_t max_duty, uint32_t step) {
            for (size_t i = 0; i < c.num_channels; i++) {
                TRY(pwm_set_duty_cycle(c.timer, c.channels[i].channel, duty_counts(c, max_duty, step, i)));
            }
            return HAL_OK;
        }

        hal_err_t pwm_sweep(const swwep_config_t& c) {
            TRY(validate(c));

            // Translate into the driver's config
            pwm_advanced_timer_config_t drv{};
            drv.pwm_mode                   = c.pwm_mode;
            drv.repetition_cnt             = c.repetition_cnt;
            drv.pwm_count_mode             = c.count_mode;
            drv.ossr                       = c.ossr;
            drv.ossi                       = c.ossi;
            drv.clk_div                    = c.clk_div;
            drv.wp_level                   = c.wp_level;
            drv.use_complementary_channels = c.use_complementary_channels;
            drv.dead_time_ns               = c.dead_time_ns;
            drv.num_channels               = c.num_channels;

            for (size_t i = 0; i < c.num_channels; i++) {
                drv.channels[i] = c.channels[i];

                // The driver indexes the complementary table by channel number, not by list position
                const size_t ch = static_cast<size_t>(c.channels[i].channel);
                if (c.use_complementary_channels && ch < c.complementary_channels.size()) {
                    drv.complementary_channels[ch] = {
                        .invert_output     = c.complementary_channels[ch].invert_output,
                        .output_idle_state = c.complementary_channels[ch].output_idle_state,
                        .gpio_pin          = c.complementary_channels[ch].gpio_pin,
                    };
                }
            }

            drv.break_input = {
                .use_break_input = c.use_break_input,
                .active_low      = c.break_active_low,
                .auto_rearm      = c.break_auto_rearm,
                .gpio_pin        = c.break_pin,
                .callback        = c.break_callback,
            };

            TRY(timer_clock_enable(c.timer, true));
            TRY(pwm_advanced_timer_init(c.timer, &drv));

            // pwm_start() zeroes every CCR, so call it once and only touch the duties afterwards
            uint32_t max_duty = 0;
            TRY(pwm_start(c.timer, c.frequency_hz, &max_duty));
            LOGI(TAG, "PWM running at %luHz, max duty cycle of %lu", c.frequency_hz, max_duty);

            const uint32_t steps = cycle_steps(c);
            for (uint32_t cycle = 0; c.cycles == 0 || cycle < c.cycles; cycle++) {
                for (uint32_t step = 0; step < steps; step++) {
                    TRY_THEN_LOG(apply_duties(c, max_duty, step), "Applying duties");
                    delay_ms(c.step_ms);
                }
            }

            // Finite runs return here with the outputs still running at the last applied duties
            return HAL_OK;
        }

    } // namespace

    void all() {
        // Defaults: 20kHz, 15us dead time, 4 channels, triangle, evenly spread lag, forever
        ASSERT(pwm_sweep({}) == HAL_OK);
    }

} // namespace demo
