#include "extra/log.h"
#include "extra/tick.h"
#include "SEGGER_RTT.h"

#include <stdarg.h>


void log_fmt(const char* esc_code, const char* tag, const char* fmt, ...) {
    // Set the output color
    SEGGER_RTT_WriteString(0, esc_code);

    // Write the timestamp and tag
    SEGGER_RTT_printf(0, "(%ums) [%s]: ", ticks_since_boot_ms(), tag);

    // Print the log message
    va_list args;
    va_start(args, fmt);
    SEGGER_RTT_vprintf(0, fmt, &args);
    va_end(args);

    // Reset the color back to the terminal's default
    SEGGER_RTT_WriteString(0, ESC_TEXT_RESET);
}

void log_str(const char* esc_code, const char* str) {
    SEGGER_RTT_WriteString(0, esc_code);
    SEGGER_RTT_WriteString(0, str);
    SEGGER_RTT_WriteString(0, ESC_TEXT_RESET);
}
