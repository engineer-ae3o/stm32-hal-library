#ifndef HEAP_H_
#define HEAP_H_


#include <stddef.h>


// The heap should be initialized and created before main runs
__attribute__((constructor)) void heap_init();

void* heap_alloc(size_t size);
void* heap_realloc(void* old, size_t new_size);

void heap_free(void* ptr);


#endif // HEAP_H_