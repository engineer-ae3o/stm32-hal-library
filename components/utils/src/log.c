#include "utils/common.h"
#include "utils/tick.h"
#include "SEGGER_RTT.h"
#include "utils/log.h"
#include "printf.h"

#include <string.h>
#include <stdarg.h>


void log_fmt(const char* esc_code, const char* tag, const char* fmt, ...) {
    char full_string[FMT_STR_BUF_SIZE];

    // The message to be logged must always contain the line end, so we reserve space for it at the end of the buffer
    size_t final_string_length = 0;

    const int header_length = snprintf_(full_string, sizeof(full_string), "%s(%lums) [%s]: ", esc_code, ticks_since_boot_ms(), tag);
    ASSERT((header_length > 0) && ((size_t)header_length < sizeof(full_string)));

    va_list args;
    va_start(args, fmt);
    const int log_length = vsnprintf_(full_string + header_length, sizeof(full_string) - (size_t)header_length, fmt, args);
    ASSERT(log_length > 0); // Format string cannot be empty
    va_end(args);

    SEGGER_RTT_WriteNoLock(RTT_BUFFER_INDEX, full_string, final_string_length);
}
