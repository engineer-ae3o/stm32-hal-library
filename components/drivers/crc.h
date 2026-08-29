#ifndef CRC_H_
#define CRC_H_


#ifdef __cplusplus
extern "C" {
#endif


#include "utils/err.h"

#include <stddef.h>
#include <stdint.h>


typedef void (*crc_dma_done_cb_t)(void* arg, uint32_t crc32);

void      crc_clk_enable(bool enable);
hal_err_t crc_get(const uint32_t* data, size_t size, uint32_t* crc32);


#ifdef __cplusplus
}
#endif


#endif // CRC_H_