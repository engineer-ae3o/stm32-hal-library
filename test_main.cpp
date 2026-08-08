#include "test/runner.hpp"
#include "utils/log.h"
#include "utils/tick.h"

#include <array>
#include <cstdint>
#include <numeric>
#include <algorithm>

namespace {

    void profile_logging() {
        uint32_t shortest_time_cycles = UINT32_MAX, longest_time_cycles = 0;

        std::array<uint32_t, 100> log_time_samples{};
        for (uint32_t i = 0; auto& sample : log_time_samples) {
            prof_start();
            LOGI("Profile_Logs", "Typical log message. Lets do this as well: %d, %d, %u. Even more stuff. Long enough. Iteration: %lu", 3, 4, 5, i);
            sample = prof_end();

            shortest_time_cycles = std::min(shortest_time_cycles, sample);
            longest_time_cycles  = std::max(longest_time_cycles, sample);
            i++;
        }

        uint64_t total_time     = std::accumulate(log_time_samples.begin(), log_time_samples.end(), 0ULL);
        uint32_t average_cycles = static_cast<uint32_t>(total_time / log_time_samples.size());

        // Average logging time
        LOGI("Profile_Logs", "Average logging time: %lu cycles", average_cycles);
        LOGI("Profile_Logs", "Average logging time: %lluus", cycles_to_us(average_cycles));
        LOGI("Profile_Logs", "Average logging time: %llums", cycles_to_ms(average_cycles));

        // Shortest log time
        LOGI("Profile_Logs", "Shortest time for logging: %lu cycles", shortest_time_cycles);
        LOGI("Profile_Logs", "Shortest time for logging: %lluus", cycles_to_us(shortest_time_cycles));
        LOGI("Profile_Logs", "Shortest time for logging: %llums", cycles_to_ms(shortest_time_cycles));

        // Longest log time
        LOGI("Profile_Logs", "Longest time for logging: %lu cycles", longest_time_cycles);
        LOGI("Profile_Logs", "Longest time for logging: %lluus", cycles_to_us(longest_time_cycles));
        LOGI("Profile_Logs", "Longest time for logging: %llums", cycles_to_ms(longest_time_cycles));
    }

} // namespace

extern "C" {
[[noreturn]] int main() {
    // Run all the tests and halt.
    // test::runner();

    // Profile logging
    profile_logging();

    // Halt once tests are finished since nothing else to do.
    LOGI("Main", "Done with all tests. Halting...");
    while (true);
}
}
