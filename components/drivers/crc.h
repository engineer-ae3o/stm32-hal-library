#ifndef CRC_H_
#define CRC_H_


#ifdef __cplusplus
extern "C" {
#endif


#include "drivers/dma.h"
#include "utils/err.h"

#include <stddef.h>
#include <stdint.h>


typedef void (*crc_dma_done_cb_t)(void* arg, hal_err_t err, uint32_t crc32);

void      crc_clk_enable(bool enable);
hal_err_t crc_get(const uint32_t* data, size_t size, uint32_t* crc32);
hal_err_t crc_get_dma(const uint32_t* data, uint16_t size, crc_dma_done_cb_t cb, void* arg);
dma_map_t crc_get_dma_stream_info();


#ifdef __cplusplus
}
#endif


#endif // CRC_H_