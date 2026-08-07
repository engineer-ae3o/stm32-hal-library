#include "o1heap.h"
#include "unity.h"

#include "extra/common.h"
#include "extra/heap.h"
#include "extra/tick.h"
#include "extra/log.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <numeric>
#include <algorithm>


namespace test::heap {

    namespace {

        constexpr const char* TAG = "HeapTest";

        // Helper to verify zero leaks and valid invariant state
        void assert_heap_clean() {
            TEST_ASSERT_TRUE(check_heap_state());

            heap_info_t info{};
            get_heap_stats(&info);

            TEST_ASSERT_EQUAL(0, info.allocated_size);
            TEST_ASSERT_EQUAL(0, info.num_of_allocations);
        }

    } // namespace

    void invalid_arg_guards() {
        assert_heap_clean();

        // Malloc 0 or > HEAP_SIZE
        TEST_ASSERT_NULL(lib_malloc(0));
        TEST_ASSERT_NULL(lib_malloc(HEAP_SIZE_BYTES + 1));

        // Calloc zero args or > HEAP_SIZE
        TEST_ASSERT_NULL(lib_calloc(0, 100));
        TEST_ASSERT_NULL(lib_calloc(100, 0));
        TEST_ASSERT_NULL(lib_calloc(HEAP_SIZE_BYTES, 2));

        // Freeing NULL must be safe
        lib_free(nullptr);

        assert_heap_clean();
    }

    void calloc_zero_initialization() {
        assert_heap_clean();

        constexpr size_t ELEM_COUNT = 32;
        constexpr size_t ELEM_SIZE  = 16;
        constexpr size_t TOTAL      = ELEM_COUNT * ELEM_SIZE;

        auto* ptr = static_cast<uint8_t*>(lib_calloc(ELEM_COUNT, ELEM_SIZE));
        TEST_ASSERT_NOT_NULL(ptr);

        // Verify memory is completely zeroed
        const bool is_zeroed = std::all_of(ptr, ptr + TOTAL, [](uint8_t b) {
            return b == 0;
        });
        TEST_ASSERT_TRUE(is_zeroed);

        lib_free(ptr);
        assert_heap_clean();
    }

    void realloc_semantics_and_data_preservation() {
        assert_heap_clean();

        // 1. realloc(nullptr, size) acts as malloc
        constexpr size_t INITIAL_SIZE = 128 - 8;
        auto*            ptr          = static_cast<uint8_t*>(lib_realloc(nullptr, INITIAL_SIZE));
        TEST_ASSERT_NOT_NULL(ptr);

        // Fill payload
        std::memset(ptr, 0xEE, INITIAL_SIZE);

        // 2. Expand allocation
        constexpr size_t EXPANDED_SIZE = 512 - 8;
        auto*            expanded_ptr  = static_cast<uint8_t*>(lib_realloc(ptr, EXPANDED_SIZE));
        TEST_ASSERT_NOT_NULL(expanded_ptr);

        // Verify payload contents preserved
        const bool preserved = std::all_of(expanded_ptr, expanded_ptr + INITIAL_SIZE, [](uint8_t b) {
            return b == 0xEE;
        });
        TEST_ASSERT_TRUE(preserved);

        // 3. realloc to oversized request should fail and return NULL without freeing original block
        auto* failed_ptr = lib_realloc(expanded_ptr, HEAP_SIZE_BYTES + 100);
        TEST_ASSERT_NULL(failed_ptr);

        // Original block must still be valid
        TEST_ASSERT_TRUE(check_heap_state());

        // 4. realloc(ptr, 0) acts as free
        auto* null_ptr = lib_realloc(expanded_ptr, 0);
        TEST_ASSERT_NULL(null_ptr);

        assert_heap_clean();
    }

    void max_capacity_packing_efficiency() {
        assert_heap_clean();

        // Optimal packing pattern verified earlier: 51 x 504B + 6 x 1016B = 32,256 B
        // A header is O1HEAP_ALIGNMENT bytes
        std::array<void*, 51> small_ptrs{};
        std::array<void*, 6>  large_ptrs{};

        for (auto& p : small_ptrs) {
            p = lib_malloc(512 - O1HEAP_ALIGNMENT);
            TEST_ASSERT_NOT_NULL(p);
        }

        for (auto& p : large_ptrs) {
            p = lib_malloc(1024 - O1HEAP_ALIGNMENT);
            TEST_ASSERT_NOT_NULL(p);
        }

        heap_info_t info{};
        get_heap_stats(&info);

        TEST_ASSERT_EQUAL(32256, info.allocated_size);
        TEST_ASSERT_EQUAL(57, info.num_of_allocations);
        TEST_ASSERT_EQUAL(1016, info.peak_request_size);
        TEST_ASSERT_EQUAL(0, info.out_of_mem_count);

        // Cleanup
        for (auto& p : small_ptrs) {
            lib_free(p);
        }
        for (auto& p : large_ptrs) {
            lib_free(p);
        }

        assert_heap_clean();
    }

    void oom_counter_and_power_of_two_penalty() {
        assert_heap_clean();

        // 512-byte allocations without subtracting header forces 1024-byte bins
        std::array<void*, 35> ptrs{};
        size_t                allocated_count = 0;

        for (auto& p : ptrs) {
            p = lib_malloc(512);
            if (p != nullptr) {
                allocated_count++;
            }
        }

        heap_info_t info{};
        get_heap_stats(&info);

        // 32KiB heap fits exactly 31 x 1024-byte bins (after instance overhead)
        TEST_ASSERT_EQUAL(31, allocated_count);
        TEST_ASSERT_EQUAL(4, info.out_of_mem_count);

        for (auto& p : ptrs) {
            lib_free(p);
        }

        assert_heap_clean();
    }

    void profile_raw_allocation_cycles() {
        assert_heap_clean();

        constexpr size_t             ROUNDS = 50;
        std::array<uint32_t, ROUNDS> alloc_cycles{};
        std::array<uint32_t, ROUNDS> free_cycles{};
        std::array<void*, ROUNDS>    ptrs{};

        // Profile sequential allocations across changing heap states
        for (size_t i = 0; i < ROUNDS; i++) {
            prof_start();
            ptrs[i]         = lib_malloc(256 - 8);
            alloc_cycles[i] = prof_end();

            TEST_ASSERT_NOT_NULL(ptrs[i]);
        }

        // Profile sequential frees
        for (size_t i = 0; i < ROUNDS; i++) {
            prof_start();
            lib_free(ptrs[i]);
            free_cycles[i] = prof_end();
        }

        // Cycle statistics calculation
        const auto [min_alloc, max_alloc] = std::minmax_element(alloc_cycles.begin(), alloc_cycles.end());
        const uint64_t total_alloc_cycles = std::accumulate(alloc_cycles.begin(), alloc_cycles.end(), 0ULL);
        const uint32_t avg_alloc          = static_cast<uint32_t>(total_alloc_cycles / ROUNDS);

        const auto [min_free, max_free]  = std::minmax_element(free_cycles.begin(), free_cycles.end());
        const uint64_t total_free_cycles = std::accumulate(free_cycles.begin(), free_cycles.end(), 0ULL);
        const uint32_t avg_free          = static_cast<uint32_t>(total_free_cycles / ROUNDS);

        LOGI(TAG, "--- O1HEAP BENCHMARK RESULTS (%zu rounds) ---", ROUNDS);
        LOGI(TAG, "lib_malloc: Min = %lu cyc | Avg = %lu cyc | Max = %lu cyc", *min_alloc, avg_alloc, *max_alloc);
        LOGI(TAG, "lib_free  : Min = %lu cyc | Avg = %lu cyc | Max = %lu cyc", *min_free, avg_free, *max_free);

        // Verify O(1) determinism: Max cycle time must not diverge wildly from Min cycle time
        TEST_ASSERT_INT_WITHIN(100, *min_alloc, *max_alloc);

        assert_heap_clean();
    }

    void all() {
        RUN_TEST(invalid_arg_guards);
        RUN_TEST(calloc_zero_initialization);
        RUN_TEST(realloc_semantics_and_data_preservation);
        RUN_TEST(max_capacity_packing_efficiency);
        RUN_TEST(oom_counter_and_power_of_two_penalty);
        RUN_TEST(profile_raw_allocation_cycles);
    }

} // namespace test::heap
