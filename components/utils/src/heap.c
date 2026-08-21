#include "o1heap/o1heap.h"
#include "utils/common.h"
#include "utils/heap.h"
#include "utils/log.h"

#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdalign.h>


// The heap
static O1HeapInstance* s_heap_handle        = NULL;
static size_t          s_num_of_allocations = 0;

static const char* TAG = "O1heap";

// The heap should be initialized and created before main runs. Only one heap is needed
__attribute__((constructor)) static void init() {
    LOGI(TAG, "Initializing the heap.");

    static alignas(O1HEAP_ALIGNMENT) uint8_t s_heap_buffer[HEAP_SIZE_BYTES] = {};

    s_heap_handle = o1heapInit(s_heap_buffer, sizeof(s_heap_buffer));
    if (s_heap_handle == NULL) {
        LOGE(TAG, "Failed to initialize the heap. Halting.");
        PANIC();
    }
}

// Public API
void* lib_malloc(size_t size) {
    if (size == 0 || size >= HEAP_SIZE_BYTES) {
        return NULL;
    }
    void* ptr = o1heapAllocate(s_heap_handle, size);
    if (ptr != NULL) {
        s_num_of_allocations++;
    }
    return ptr;
}

void* lib_calloc(size_t nmemb, size_t size) {
    if (size == 0 || size >= HEAP_SIZE_BYTES || nmemb == 0 || nmemb >= HEAP_SIZE_BYTES) {
        return NULL;
    }
    void* ptr = o1heapAllocate(s_heap_handle, nmemb * size);
    if (ptr != NULL) {
        s_num_of_allocations++;
        memset(ptr, 0, nmemb * size);
    }
    return ptr;
}

void* lib_realloc(void* old, size_t new_size) {
    if (old == NULL) {
        return lib_malloc(new_size);
    }
    if (new_size == 0) {
        lib_free(old);
        return NULL;
    }
    if (new_size >= HEAP_SIZE_BYTES) {
        return NULL;
    }
    return o1heapReallocate(s_heap_handle, old, new_size);
}

void lib_free(void* ptr) {
    if (ptr != NULL) {
        o1heapFree(s_heap_handle, ptr);
        s_num_of_allocations--;
    }
}

bool check_heap_state(void) {
    return o1heapDoInvariantsHold(s_heap_handle);
}

void get_heap_stats(heap_info_t* info) {
    ASSERT(info);

    O1HeapDiagnostics diagnostics = o1heapGetDiagnostics(s_heap_handle);
    info->max_capacity            = diagnostics.capacity;
    info->allocated_size          = diagnostics.allocated;
    info->peak_allocated_size     = diagnostics.peak_allocated;
    info->peak_request_size       = diagnostics.peak_request_size;
    info->out_of_mem_count        = (size_t)diagnostics.oom_count;
    info->num_of_allocations      = s_num_of_allocations;
}

void print_heap_stats(void) {
    heap_info_t info;
    get_heap_stats(&info);

    LOGI(TAG, "Max heap capacity: %zubytes", info.max_capacity);
    LOGI(TAG, "Size of heap used: %zubytes", info.allocated_size);
    LOGI(TAG, "Highest size allocated since boot: %zubytes", info.peak_allocated_size);
    LOGI(TAG, "Largest requested size from heap: %zubytes", info.peak_request_size);
    LOGI(TAG, "Number of failed allocation requests: %zu", info.out_of_mem_count);
    LOGI(TAG, "Number of allocations: %zu", info.num_of_allocations);
}
