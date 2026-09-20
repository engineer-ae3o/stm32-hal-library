#ifndef MEMCPY_
#define MEMCPY_


#ifdef __cplusplus
extern "C" {
#endif


#include "drivers/dma_types.h"
#include "utils/err.h"

#include <stdint.h>


typedef enum : uint8_t {
    DMA_MEMCPY_NOT_DONE,
    DMA_MEMCPY_DONE,
    DMA_MEMCPY_ERROR,
    DMA_MEMCPY_TIMEOUT,
} memcpy_state_t;

hal_err_t      dma_memcpy(void* dest, const void* src, uint16_t len, volatile memcpy_state_t* dma_done_flag);
hal_err_t      dma_memcpy_cb(void* dest, const void* src, uint16_t len, dma_done_cb_t cb, void* arg);
memcpy_state_t dma_memcpy_wait_for_flag(volatile memcpy_state_t* dma_done_flag, uint32_t timeout);
dma_map_t      dma_memcpy_get_stream_info(void);


#ifdef __cplusplus
}
#endif


#endif // MEMCPY_