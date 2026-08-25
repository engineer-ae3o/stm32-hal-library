#ifndef LOG_H_
#define LOG_H_


#ifdef __cplusplus
extern "C" {
#endif


// Debug logging levels
#define LOG_LEVEL_INFO (3)
#define LOG_LEVEL_WARN (2)
#define LOG_LEVEL_ERROR (1)
#define LOG_LEVEL_NONE (0)


// Set the log level to any appropriate log level
#ifndef LOG_LEVEL
#define LOG_LEVEL (LOG_LEVEL_INFO)
#endif


#if LOG_LEVEL == LOG_LEVEL_INFO
#define LOGE(tag, ...) log_fmt(ESC_TEXT_RED, tag, __VA_ARGS__)
#define LOGW(tag, ...) log_fmt(ESC_TEXT_YELLOW, tag, __VA_ARGS__)
#define LOGI(tag, ...) log_fmt(ESC_TEXT_GREEN, tag, __VA_ARGS__)

#elif LOG_LEVEL == LOG_LEVEL_WARN
#define LOGE(tag, ...) log_fmt(ESC_TEXT_RED, tag, __VA_ARGS__)
#define LOGW(tag, ...) log_fmt(ESC_TEXT_YELLOW, tag, __VA_ARGS__)
#define LOGI(tag, ...)

#elif LOG_LEVEL == LOG_LEVEL_ERROR
#define LOGE(tag, ...) log_fmt(ESC_TEXT_RED, tag, __VA_ARGS__)
#define LOGW(tag, ...)
#define LOGI(tag, ...)

#elif LOG_LEVEL == LOG_LEVEL_NONE
#define LOGE(tag, ...)
#define LOGW(tag, ...)
#define LOGI(tag, ...)

#else
#error "Invalid debug log level"

#endif


// Not to be used directly
[[__gnu__::__format__(printf, 3, 4)]] void log_fmt(const char* esc_code, const char* tag, const char* fmt, ...);


#define ESC_TEXT_GREEN "\x1B[92m I "
#define ESC_TEXT_YELLOW "\x1B[93m W "
#define ESC_TEXT_RED "\x1B[91m E "
#define ESC_TEXT_RESET "\x1B[0m"

#define LINE_END (ESC_TEXT_RESET "\r\n")


#ifdef __cplusplus
}
#endif


#endif // LOG_H_