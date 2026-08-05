#ifndef _LOG_H_
#define _LOG_H_


#include "SEGGER_RTT.h"
#include "common.h"

#include <stdarg.h>
#include <stdint.h>


// Set the log level to any appropriate log level
#define LOG_LEVEL LOG_LEVEL_INFO


#if LOG_LEVEL == LOG_LEVEL_INFO
#define LOGE(tag, ...) log_fmt(ESC_TEXT_RED, tag, __VA_ARGS__)
#define LOGW(tag, ...) log_fmt(ESC_TEXT_YELLOW, tag, __VA_ARGS__)
#define LOGI(tag, ...) log_fmt(ESC_TEXT_GREEN, tag, __VA_ARGS__)

#define LOGE_ISR(str) log_isr(ESC_TEXT_RED, str)
#define LOGW_ISR(str) log_isr(ESC_TEXT_YELLOW, str)
#define LOGI_ISR(str) log_isr(ESC_TEXT_GREEN, str)

#elif LOG_LEVEL == LOG_LEVEL_WARN
#define LOGE(tag, ...) log_fmt(ESC_TEXT_RED, tag, __VA_ARGS__)
#define LOGW(tag, ...) log_fmt(ESC_TEXT_YELLOW, tag, __VA_ARGS__)
#define LOGI(tag, ...)

#define LOGE_ISR(str) log_isr(ESC_TEXT_RED, str)
#define LOGW_ISR(str) log_isr(ESC_TEXT_YELLOW, str)
#define LOGI_ISR(str)

#elif LOG_LEVEL == LOG_LEVEL_ERROR
#define LOGE(tag, ...) log_fmt(ESC_TEXT_RED, tag, __VA_ARGS__)
#define LOGW(tag, ...)
#define LOGI(tag, ...)

#define LOGE_ISR(str) log_isr(ESC_TEXT_RED, str)
#define LOGW_ISR(str)
#define LOGI_ISR(str)

#elif LOG_LEVEL == LOG_LEVEL_NONE
#define LOGE(tag, ...)
#define LOGW(tag, ...)
#define LOGI(tag, ...)

#define LOGE_ISR(str)
#define LOGW_ISR(str)
#define LOGI_ISR(str)

#endif


// Debug logging levels
#define LOG_LEVEL_INFO 3
#define LOG_LEVEL_WARN 2
#define LOG_LEVEL_ERROR 1
#define LOG_LEVEL_NONE 0


#define ESC_TEXT_GREEN "\x1B[92m I "
#define ESC_TEXT_YELLOW "\x1B[93m W "
#define ESC_TEXT_RED "\x1B[91m R "
#define ESC_TEXT_RESET "\x1B[0m"

inline void log_fmt(const char* esc_code, const char* tag, const char* fmt, ...) {
    // Set the output color
    SEGGER_RTT_WriteString(0, esc_code);

    // Write the timestamp and tag
    SEGGER_RTT_printf(0, "(%ums) [%s]: ", get_tick_ms(), tag);

    // Print the actual log message
    va_list args;
    va_start(args, fmt);
    SEGGER_RTT_vprintf(0, fmt, &args);
    va_end(args);

    // Reset the color back to the terminal's default
    SEGGER_RTT_WriteString(0, "\r\n" ESC_TEXT_RESET);
}

inline void log_isr(const char* esc_code, const char* str) {
    SEGGER_RTT_WriteString(0, esc_code);
    SEGGER_RTT_WriteString(0, str);
    SEGGER_RTT_WriteString(0, "\r\n" ESC_TEXT_RESET);
}


#endif // _LOG_H_