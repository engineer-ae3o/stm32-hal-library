#ifndef LOG_H_
#define LOG_H_


#ifdef __cplusplus
extern "C" {
#endif


// Debug logging levels
#define LOG_LEVEL_INFO 3
#define LOG_LEVEL_WARN 2
#define LOG_LEVEL_ERROR 1
#define LOG_LEVEL_NONE 0


// Set the log level to any appropriate log level
#define LOG_LEVEL LOG_LEVEL_INFO


// The LOGx macros are the standard logging functions, take in variadic arguments
// and go through the formatting path. The LOGx_ISR macros on the other hand don't
// have any formatting capability, as its just a plain string write. This simplicity
// allows it to be called from ISR contexts since the actual transport mechanism is a
// literal memcpy(...) into the RTT buffer, thud, making it light enough to be called
// from an ISR. It does lack the TAGs and timestamps that the regular LOGx macros have.
// This is an acceptable trade-off to have logging inside ISRs.


#if LOG_LEVEL == LOG_LEVEL_INFO
#define LOGE(tag, ...) log_fmt(ESC_TEXT_RED, tag, __VA_ARGS__)
#define LOGW(tag, ...) log_fmt(ESC_TEXT_YELLOW, tag, __VA_ARGS__)
#define LOGI(tag, ...) log_fmt(ESC_TEXT_GREEN, tag, __VA_ARGS__)

#define LOGE_ISR(str) log_str(ESC_TEXT_RED, str)
#define LOGW_ISR(str) log_str(ESC_TEXT_YELLOW, str)
#define LOGI_ISR(str) log_str(ESC_TEXT_GREEN, str)

#elif LOG_LEVEL == LOG_LEVEL_WARN
#define LOGE(tag, ...) log_fmt(ESC_TEXT_RED, tag, __VA_ARGS__)
#define LOGW(tag, ...) log_fmt(ESC_TEXT_YELLOW, tag, __VA_ARGS__)
#define LOGI(tag, ...)

#define LOGE_ISR(str) log_str(ESC_TEXT_RED, str)
#define LOGW_ISR(str) log_str(ESC_TEXT_YELLOW, str)
#define LOGI_ISR(str)

#elif LOG_LEVEL == LOG_LEVEL_ERROR
#define LOGE(tag, ...) log_fmt(ESC_TEXT_RED, tag, __VA_ARGS__)
#define LOGW(tag, ...)
#define LOGI(tag, ...)

#define LOGE_ISR(str) log_str(ESC_TEXT_RED, str)
#define LOGW_ISR(str)
#define LOGI_ISR(str)

#elif LOG_LEVEL == LOG_LEVEL_NONE
#define LOGE(tag, ...)
#define LOGW(tag, ...)
#define LOGI(tag, ...)

#define LOGE_ISR(str)
#define LOGW_ISR(str)
#define LOGI_ISR(str)

#else
#error "Invalid debug log level"

#endif


// To not be used directly
__attribute__((format(printf, 3, 4))) void log_fmt(const char* esc_code, const char* tag, const char* fmt, ...);

void log_str(const char* esc_code, const char* str);


#define ESC_TEXT_GREEN "\x1B[92m I "
#define ESC_TEXT_YELLOW "\x1B[93m W "
#define ESC_TEXT_RED "\x1B[91m R "
#define ESC_TEXT_RESET "\r\n\x1B[0m"


#ifdef __cplusplus
}
#endif


#endif // LOG_H_