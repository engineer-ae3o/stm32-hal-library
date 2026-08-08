#include "utils/common.h"
#include "utils/tick.h"
#include "SEGGER_RTT.h"
#include "utils/log.h"

#include <stdio.h>
#include <string.h>
#include <stdarg.h>


void log_fmt(const char* esc_code, const char* tag, const char* fmt, ...) {
    char full_string[FMT_STR_BUF_SIZE];

    int prefix_len = snprintf(full_string, sizeof(full_string), "%s(%lums) [%s]: ", esc_code, ticks_since_boot_ms(), tag);
    ASSERT((prefix_len > 0) && ((size_t)prefix_len < sizeof(full_string)));

    va_list args;
    va_start(args, fmt);
    int msg_len = vsnprintf(full_string + prefix_len, sizeof(full_string) - (size_t)prefix_len, fmt, args);
    va_end(args);

    size_t final_string_size = (size_t)prefix_len + (msg_len < 0 ? 0 : (size_t)msg_len);
    if (final_string_size >= sizeof(full_string)) {
        final_string_size = sizeof(full_string) - 1; // truncated
    }

    // append reset, clamped to remaining space
    size_t remaining = sizeof(full_string) - final_string_size;
    size_t reset_len = sizeof(ESC_TEXT_RESET "\r\n") - 1;
    if (reset_len >= remaining) {
        reset_len = remaining > 0 ? remaining - 1 : 0;
    }
    memcpy(full_string + final_string_size, ESC_TEXT_RESET "\r\n", reset_len);
    final_string_size += reset_len;

    LOCK_ACQUIRE();
    SEGGER_RTT_Write(RTT_BUFFER_INDEX, full_string, final_string_size);
    LOCK_RELEASE();
}

void log_str(const char* esc_code, const char* msg) {
    LOCK_ACQUIRE();
    SEGGER_RTT_WriteString(RTT_BUFFER_INDEX, esc_code);
    SEGGER_RTT_WriteString(RTT_BUFFER_INDEX, msg);
    SEGGER_RTT_WriteString(RTT_BUFFER_INDEX,
                           ESC_TEXT_RESET "\r\n"
                                          "\r\n");
    LOCK_RELEASE();
}
