#ifndef DMA_TYPES_H_
#define DMA_TYPES_H_


#include "stm32f411xe.h"
#include "utils/err.h"

#include <stdint.h>


typedef enum : uint8_t {
    DMA_DIR_P_M = 0b00, // Peripheral to memory
    DMA_DIR_M_P = 0b01, // Memory to peripheral
    DMA_DIR_M_M = 0b10, // Memory to memory
} dma_direction_t;

typedef enum : uint8_t {
    DMA_PRIORITY_LOW       = 0b00,
    DMA_PRIORITY_MEDIUM    = 0b01,
    DMA_PRIORITY_HIGH      = 0b10,
    DMA_PRIORITY_VERY_HIGH = 0b11,
} dma_priority_t;

typedef enum : uint8_t {
    DMA_SIZE_BYTE  = 0b00, // Data size is a byte
    DMA_SIZE_HWORD = 0b01, // Data size is a half word (2 bytes)
    DMA_SIZE_WORD  = 0b10, // Data size is a word (4 bytes)
} dma_data_size_t;

typedef enum : uint8_t {
    DMA_MODE_FIFO,
    DMA_MODE_DIRECT,
} dma_direct_mode_t;

typedef enum : uint8_t {
    DMA_FLOW_CONTROLLER_DMA,        // The DMA controller is the flow controller
    DMA_FLOW_CONTROLLER_PERIPHERAL, // The peripheral is the flow controller
} dma_flow_control_t;

typedef enum : uint8_t {
    DMA_MODE_NO_CIRCULAR,
    DMA_MODE_CIRCULAR,
    DMA_MODE_DOUBLE_BUFFER,
} dma_circ_mode_t;

typedef struct {
    // irq_status_register is the DMA interrupt status register.
    // When a bit gets set, the corresponding DMA interrupt is fired.
    uint32_t* irq_status_register;

    // irq_clear_register is the DMA interrupt clear register. It is used to clear the
    // interrupt bit set in its corresponding status register. It is write 1 to clear a bit.
    uint32_t* irq_clear_register;

    uint32_t tc_mask;  // Transfer complete flag bitmask
    uint32_t ht_mask;  // Half transfer complete flag bitmask
    uint32_t te_mask;  // Transfer error flag bitmask
    uint32_t dme_mask; // Direct mode error flag bitmask
    uint32_t fe_mask;  // FIFO mode error flag bitmask
} dma_stream_flags_t;

typedef struct {
    DMA_Stream_TypeDef* stream;
    uint8_t             channel;
} dma_map_t;

typedef struct {
    dma_map_t tx;
    dma_map_t rx;
} dma_stream_map_t;

// Callback for DMA transmission and reception completion
typedef void (*dma_done_cb_t)(void* arg, hal_err_t error);

typedef struct {
    dma_done_cb_t callback;
    void*         arg;
} dma_ctx_t;

typedef struct {
    dma_ctx_t tx;
    dma_ctx_t rx;
} dma_stream_ctx_t;

typedef struct {
    DMA_Stream_TypeDef* stream;
    DMA_TypeDef*        controller;
    uint32_t            stream_number;
    IRQn_Type           nvic_irq_type;
} dma_stream_info_t;

typedef struct {
    bool deconfigure;   // Set to deinitialize the given stream. All other fields are ignored if this is true
    bool enable_stream; // Enable the DMA stream immediately after configuring it

    bool per_inc; // Increment the peripheral address (or source address in memory to memory transfers)
    bool mem_inc; // Increment the memory address(es) (or destination address(es) in memory to memory transfers)

    bool tc_irq_enable;  // Transfer complete interrupt enable
    bool ht_irq_enable;  // Half transfer interrupt enable
    bool te_irq_enable;  // Transfer error interrupt enable
    bool dme_irq_enable; // Direct mode error interrupt enable
    bool fe_irq_enable;  // FIFO mode error interrupt enable

    dma_direct_mode_t  mode;
    dma_priority_t     priority;
    dma_direction_t    direction;
    dma_data_size_t    per_data_size;
    dma_data_size_t    mem_data_size;
    dma_circ_mode_t    circular_mode;
    dma_flow_control_t flow_controller;

    uint16_t buffer_size;
    uint32_t channel;
    uint32_t nvic_irq_priority;

    const volatile void* per_addr;
    const volatile void* mem_buf_0;
    const volatile void* mem_buf_1;
} dma_stream_config_t;


#endif // DMA_TYPES_H_