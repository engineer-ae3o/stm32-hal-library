#include "extra/common.h"
#include "extra/tick.h"
#include "SEGGER_RTT.h"
#include "extra/log.h"

#include <stdarg.h>


void log_fmt(const char* esc_code, const char* tag, const char* fmt, ...) {
    // Set the output color and write the timestamp and tag
    SEGGER_RTT_printf(RTT_BUFFER_INDEX, "%s(%lums) [%s]: ", esc_code, ticks_since_boot_ms(), tag);

    // Print the log message
    va_list args = {};
    va_start(args, fmt);
    SEGGER_RTT_vprintf(RTT_BUFFER_INDEX, fmt, &args);
    va_end(args);

    // Reset the color back to the terminal's default and go to the next line
    SEGGER_RTT_WriteString(RTT_BUFFER_INDEX, ESC_TEXT_RESET);
}

void log_str(const char* esc_code, const char* str) {
    SEGGER_RTT_WriteString(RTT_BUFFER_INDEX, esc_code);
    SEGGER_RTT_WriteString(RTT_BUFFER_INDEX, str);
    SEGGER_RTT_WriteString(RTT_BUFFER_INDEX, ESC_TEXT_RESET);
}
