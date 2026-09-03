#include "RTT/SEGGER_RTT.h"
#include "printf/printf.h"
#include "utils/common.h"
#include "utils/tick.h"
#include "utils/log.h"

#include <string.h>
#include <stdarg.h>


void log_fmt(const char* esc_code, const char* tag, const char* fmt, ...) {
    char full_string[FMT_STR_BUF_SIZE];

    // The message to be logged must always contain the line end, so we reserve space for it at the end of the buffer.
    const int line_end_length = sizeof(LINE_END) - 1;
    const int buffer_length   = (int)sizeof(full_string) - line_end_length;

    // Write the header of the log which consists of the escape code to set the output color, the timestamp, and the tag.
    const int header_length = snprintf_(full_string, buffer_length, "%s(%lums) [%s]: ", esc_code, ticks_since_boot_ms(), tag);
    ASSERT(header_length > 0 && header_length < (int)buffer_length);
    const int size_remaining = buffer_length - header_length;

    // Write the formatted string
    va_list args;
    va_start(args, fmt);
    const int total_log_length = vsnprintf_(full_string + header_length, (size_t)(size_remaining), fmt, args);
    ASSERT(total_log_length > 0); // Format string can't/shouldn't be empty
    va_end(args);

    // Get the actual length of the format string that was written and ignore the null terminator added
    const int actual_log_length = total_log_length >= size_remaining ? (size_remaining - 1) : total_log_length;

    // Write the line end at the back of the buffer
    size_t final_string_length = (size_t)(header_length + actual_log_length);
    memcpy(full_string + final_string_length, LINE_END, line_end_length);
    final_string_length += line_end_length;

    // Write the final string with no locks. If thread safety is needed, it can be done with
    // the addition of a mutex, rather than disabling all interrupts for the entire write period.
    ASSERT(SEGGER_RTT_WriteNoLock(RTT_BUFFER_INDEX, full_string, final_string_length) == final_string_length);
}
