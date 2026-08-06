#ifndef HEAP_H_
#define HEAP_H_


#ifdef __cplusplus
extern "C" {
#endif


#include <stddef.h>


typedef struct {
    size_t max_capacity;
    size_t allocated_size;
    size_t peak_allocated_size;
    size_t peak_request_size;
    size_t out_of_mem_count;
    size_t num_of_allocations;
} heap_info_t;


void* lib_malloc(size_t size);
void* lib_calloc(size_t nmemb, size_t size);
void* lib_realloc(void* old, size_t new_size);
void  lib_free(void* ptr);

bool check_heap_state(void);
void get_heap_stats(heap_info_t* info);
void print_heap_stats(void);


#ifdef __cplusplus
}
#endif


#endif // HEAP_H_