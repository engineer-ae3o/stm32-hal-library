#include "test/runner.hpp"
#include "o1heap/o1heap.h"
#include "utils/common.h"
#include "drivers/gpio.h"
#include "drivers/crc.h"
#include "drivers/adc.h"
#include "utils/tick.h"
#include "utils/log.h"

#include <array>
#include <cstdint>
#include <numeric>
#include <algorithm>


namespace profile {

    void segger_rtt() {
        constexpr const char* TAG = "Profile_Logs";

        constexpr uint32_t LOG_FREQ_HZ = 40;
        constexpr uint32_t SAMPLE_SIZE = 1'000;

        uint32_t shortest_time_cycles = UINT32_MAX, longest_time_cycles = 0;

        std::array<uint32_t, SAMPLE_SIZE> log_time_samples{};
        for (uint32_t i = 0; auto& sample : log_time_samples) {
            prof_start();
            LOGI(TAG,
                 "Typical log. Lets also do this: %.3f,%d,%d. Even more stuff. Add a format string as well. %s long enough. Iteration: %lu",
                 3.0,
                 4,
                 5,
                 "This should be",
                 i);
            sample = prof_end();

            shortest_time_cycles = std::min(shortest_time_cycles, sample);
            longest_time_cycles  = std::max(longest_time_cycles, sample);
            i++;

            // Adding a small delay. Normal operation won't log anywhere near this frequency.
            delay_us(1'000'000U / LOG_FREQ_HZ);
        }

        const uint64_t total_cycles   = std::accumulate(log_time_samples.begin(), log_time_samples.end(), 0ULL);
        const uint32_t average_cycles = static_cast<uint32_t>(total_cycles / log_time_samples.size());

        // Total time for all rounds
        LOGI(TAG, "Total time taken: %llu cycles", total_cycles);
        LOGI(TAG, "Total time taken: %llu.%03llums", cycles_to_ms(total_cycles), cycles_to_ms_frac(total_cycles));
        LOGI(TAG, "Total time taken: %llu.%03llus", cycles_to_ss(total_cycles), cycles_to_ss_frac(total_cycles));

        // Average logging time
        LOGI(TAG, "Average logging time: %lu cycles", average_cycles);
        LOGI(TAG, "Average logging time: %llu.%03lluus", cycles_to_us(average_cycles), cycles_to_us_frac(average_cycles));
        LOGI(TAG, "Average logging time: %llu.%03llums", cycles_to_ms(average_cycles), cycles_to_ms_frac(average_cycles));

        // Shortest log time
        LOGI(TAG, "Shortest time for logging: %lu cycles", shortest_time_cycles);
        LOGI(TAG, "Shortest time for logging: %llu.%03lluus", cycles_to_us(shortest_time_cycles), cycles_to_us_frac(shortest_time_cycles));
        LOGI(TAG, "Shortest time for logging: %llu.%03llums", cycles_to_ms(shortest_time_cycles), cycles_to_ms_frac(shortest_time_cycles));

        // Longest log time
        LOGI(TAG, "Longest time for logging: %lu cycles", longest_time_cycles);
        LOGI(TAG, "Longest time for logging: %llu.%03lluus", cycles_to_us(longest_time_cycles), cycles_to_us_frac(longest_time_cycles));
        LOGI(TAG, "Longest time for logging: %llu.%03llums", cycles_to_ms(longest_time_cycles), cycles_to_ms_frac(longest_time_cycles));
    }

    void o1heap_malloc() {
        constexpr const char* TAG = "O1Heap_Profile";

        // Requires O1HEAP_ALIGNMENT aligned arena. Uses a PRNG
        // instead of rand() to avoid the overhead of libc.
        constexpr uint32_t ARENA_SIZE = 32 * 1024;

        alignas(O1HEAP_ALIGNMENT) std::array<uint8_t, ARENA_SIZE> arena{};

        constexpr uint32_t SAMPLE_SIZE = 1'000;
        constexpr uint32_t POOL_SIZE   = 64; // Outstanding allocations kept live during the test

        constexpr size_t MIN_ALLOC = 8;
        constexpr size_t MAX_ALLOC = 512;

        O1HeapInstance* heap = o1heapInit(arena.data(), ARENA_SIZE);
        if (heap == nullptr) {
            LOGI(TAG, "Heap initialization failed, arena not aligned or too small");
            ASSERT(false);
        }

        // Helpers for pseudo random state generation
        uint32_t rng_state = 0xC0FFEE01;

        auto xorshift32 = [&] {
            rng_state ^= rng_state << 13;
            rng_state ^= rng_state >> 17;
            rng_state ^= rng_state << 5;
            return rng_state;
        };

        // Produce a pseudo random number in the range of [MIN_ALLOC - MAX_ALLOC]
        auto rand_size = [&] {
            return MIN_ALLOC + (xorshift32() % (MAX_ALLOC - MIN_ALLOC));
        };

        std::array<void*, POOL_SIZE> pool{};
        // Prime the pool so when we start the profiling, we operating on a fragmented, non empty heap.
        for (auto& ptr : pool) {
            // cppcheck-suppress useStlAlgorithm
            ptr = o1heapAllocate(heap, rand_size());
            ASSERT(ptr != nullptr);
        }

        std::array<uint32_t, SAMPLE_SIZE> alloc_samples{};
        std::array<uint32_t, SAMPLE_SIZE> free_samples{};

        uint32_t alloc_min = UINT32_MAX, alloc_max = 0;
        uint32_t free_min = UINT32_MAX, free_max = 0;

        for (uint32_t i = 0; i < SAMPLE_SIZE; i++) {
            // Free a pseudo random slot (not the one we're about to fill). This keeps
            // fragmentation churning instead of settling into a stable pattern.
            uint32_t victim = xorshift32() % POOL_SIZE;

            __disable_irq();
            prof_start();
            o1heapFree(heap, pool[victim]);
            free_samples[i] = prof_end();
            __enable_irq();

            free_min = std::min(free_min, free_samples[i]);
            free_max = std::max(free_max, free_samples[i]);

            // Reallocate the memory back into the slot that was just freed, but with a different size.
            const size_t request_size = rand_size();

            __disable_irq();
            prof_start();
            pool[victim]     = o1heapAllocate(heap, request_size);
            alloc_samples[i] = prof_end();
            __enable_irq();

            ASSERT(pool[victim] != nullptr);

            alloc_min = std::min(alloc_min, alloc_samples[i]);
            alloc_max = std::max(alloc_max, alloc_samples[i]);
        }

        const uint64_t alloc_total = std::accumulate(alloc_samples.begin(), alloc_samples.end(), 0ULL);
        const uint64_t free_total  = std::accumulate(free_samples.begin(), free_samples.end(), 0ULL);
        const uint32_t alloc_avg   = static_cast<uint32_t>(alloc_total / alloc_samples.size());
        const uint32_t free_avg    = static_cast<uint32_t>(free_total / free_samples.size());

        // Allocations
        LOGI(TAG, "[Alloc] Total time taken: %llu cycles", alloc_total);
        LOGI(TAG, "[Alloc] Total time taken: %llu.%03llums", cycles_to_ms(alloc_total), cycles_to_ms_frac(alloc_total));
        LOGI(TAG, "[Alloc] Total time taken: %llu.%03llus", cycles_to_ss(alloc_total), cycles_to_ss_frac(alloc_total));

        LOGI(TAG, "[Alloc] Average time: %lu cycles", alloc_avg);
        LOGI(TAG, "[Alloc] Average time: %llu.%03lluus", cycles_to_us(alloc_avg), cycles_to_us_frac(alloc_avg));
        LOGI(TAG, "[Alloc] Average time: %llu.%03llums", cycles_to_ms(alloc_avg), cycles_to_ms_frac(alloc_avg));

        LOGI(TAG, "[Alloc] Shortest time: %lu cycles", alloc_min);
        LOGI(TAG, "[Alloc] Shortest time: %llu.%03lluus", cycles_to_us(alloc_min), cycles_to_us_frac(alloc_min));
        LOGI(TAG, "[Alloc] Shortest time: %llu.%03llums", cycles_to_ms(alloc_min), cycles_to_ms_frac(alloc_min));

        LOGI(TAG, "[Alloc] Longest (WCET) time: %lu cycles", alloc_max);
        LOGI(TAG, "[Alloc] Longest (WCET) time: %llu.%03lluus", cycles_to_us(alloc_max), cycles_to_us_frac(alloc_max));
        LOGI(TAG, "[Alloc] Longest (WCET) time: %llu.%03llums", cycles_to_ms(alloc_max), cycles_to_ms_frac(alloc_max));

        // Freeing memory
        LOGI(TAG, "[Free] Total time taken: %llu cycles", free_total);
        LOGI(TAG, "[Free] Total time taken: %llu.%03llums", cycles_to_ms(free_total), cycles_to_ms_frac(free_total));
        LOGI(TAG, "[Free] Total time taken: %llu.%03llus", cycles_to_ss(free_total), cycles_to_ss_frac(free_total));

        LOGI(TAG, "[Free] Average time: %lu cycles", free_avg);
        LOGI(TAG, "[Free] Average time: %llu.%03lluus", cycles_to_us(free_avg), cycles_to_us_frac(free_avg));
        LOGI(TAG, "[Free] Average time: %llu.%03llums", cycles_to_ms(free_avg), cycles_to_ms_frac(free_avg));

        LOGI(TAG, "[Free] Shortest time: %lu cycles", free_min);
        LOGI(TAG, "[Free] Shortest time: %llu.%03lluus", cycles_to_us(free_min), cycles_to_us_frac(free_min));
        LOGI(TAG, "[Free] Shortest time: %llu.%03llums", cycles_to_ms(free_min), cycles_to_ms_frac(free_min));

        LOGI(TAG, "[Free] Longest (WCET) time: %lu cycles", free_max);
        LOGI(TAG, "[Free] Longest (WCET) time: %llu.%03lluus", cycles_to_us(free_max), cycles_to_us_frac(free_max));
        LOGI(TAG, "[Free] Longest (WCET) time: %llu.%03llums", cycles_to_ms(free_max), cycles_to_ms_frac(free_max));

        // Clean up the remaining live allocations from the start
        for (auto* ptr : pool) {
            o1heapFree(heap, ptr);
        }
    }

    void all() {
        // Profile logging from SEGGER RTT
        segger_rtt();
        // Profile malloc from o1heap
        o1heap_malloc();
    }

} // namespace profile

extern "C" {
int main() {
    /**
    // Run all the hardware driver tests
    test::runner();

    // Run the profile tests
    profile::all();

    // Halt once tests are finished since nothing else to do.
    LOGI("Main", "Done with all tests. Halting...");
    */


    // ADC peripheral
    adc_clk_configure(ADC_CLK_PRESCALER_4);
    adc_enable_nvic_irq(true);
    adc_power_on_temp_sensor(true);

    constexpr adc_config_t adc_cfg = {
        .alignment       = ADC_RIGHT_ALIGN,
        .resolution      = ADC_RES_12_BITS,
        .sampling_cycles = ADC_SAMPLE_15_CYCLES,
    };
    ASSERT(adcx_clk_enable(ADC1, true) == HAL_OK);
    ASSERT(adc_configure(ADC1, &adc_cfg) == HAL_OK);
    ASSERT(adc_power_on(ADC1, true) == HAL_OK);

    // Configure the analog watchdog
    constexpr adc_analog_wdg_config_t awdg_config = {
        .min_adc_value             = 0,
        .max_adc_value             = 4095,
        .monitor_regular_channels  = true,
        .monitor_injected_channels = true,
        .on_thresholds_violated =
            [](void* arg) {
                (void)arg;
                LOGE("ADC_WDG", "Voltage on an ADC channel past the valid range");
                PANIC();
            },
        .arg = nullptr,
    };
    ASSERT(adc_analog_wdg_start(ADC1, &awdg_config) == HAL_OK);

    // Configure PA3 as analog since it is ADC channel 3
    ASSERT(gpiox_clk_enable(GPIOA, true) == HAL_OK);
    gpio_set_analog(GPIOA, 3);

    // Get the raw ADC data
    uint16_t raw_channel_3 = 0, raw_vbat = 0, raw_vref_int = 0;
    ASSERT(adc_get_v_bat(ADC1, &raw_vbat) == HAL_OK);
    ASSERT(adc_get_v_ref_internal(ADC1, &raw_vref_int) == HAL_OK);
    ASSERT(adc_regular_group_get_oneshot(ADC1, ADC_CHANNEL_3, &raw_channel_3) == HAL_OK);

    // Get the converted voltages
    float vdda = 0, temp_celsius = 0, channel_3 = 0, vbat = 0, vref_int = 0;
    ASSERT(adc_get_vdda(ADC1, &vdda) == HAL_OK);
    ASSERT(adc_get_temp_celsius(ADC1, &temp_celsius) == HAL_OK);
    ASSERT(adc_get_value_right_aligned(ADC1, raw_vbat, ADC_RES_12_BITS, &vbat) == HAL_OK);
    ASSERT(adc_get_value_right_aligned(ADC1, raw_vref_int, ADC_RES_12_BITS, &vref_int) == HAL_OK);
    ASSERT(adc_get_value_right_aligned(ADC1, raw_channel_3, ADC_RES_12_BITS, &channel_3) == HAL_OK);

    // Log the data
    LOGI("ADC", "VDDA: %.3fV", (double)vdda);
    LOGI("ADC", "V_bat: %.3fV", (double)vbat);
    LOGI("ADC", "PA3: %.3fV", (double)channel_3);
    LOGI("ADC", "V_ref_int: %.3fV", (double)vref_int);
    LOGI("ADC", "Temperature: %.3fC", (double)temp_celsius);


    // CRC32 peripheral
    crc_clk_enable(true);

    constexpr auto buffer = std::array{1UL, 2UL, 3UL, 4UL, 5UL, 6UL, 7UL, 8UL, 9UL};
    uint32_t       crc32  = 0;

    ASSERT(crc_get(buffer.data(), buffer.size(), &crc32) == HAL_OK);
    ASSERT(crc_get_dma(
               buffer.data(),
               buffer.size(),
               [](void* arg, hal_err_t err, uint32_t crc_32) {
                   UNUSED(arg);
                   ASSERT(err == HAL_OK);
                   LOGI("CRC32", "CRC32 checksum of data gotten with DMA = %lu", crc_32);
                   LOGI("CRC32", "CRC32 checksum of data gotten with DMA = 0x%X", (size_t)crc_32);
               },
               nullptr) == HAL_OK);

    LOGI("CRC32", "CRC32 checksum of data = %lu", crc32);
    LOGI("CRC32", "CRC32 checksum of data = 0x%X", (size_t)crc32);

    HALT();
}
}
