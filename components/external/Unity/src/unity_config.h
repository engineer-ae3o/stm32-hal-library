#ifndef _UNITY_CONFIG_H_
#define _UNITY_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

#if !defined(UNITY_INCLUDE_CONFIG_H)
#define UNITY_INCLUDE_CONFIG_H
#endif

#if !defined(UNITY_OUTPUT_COLOR)
#define UNITY_OUTPUT_COLOR
#endif

#if !defined(UNITY_EXCLUDE_DOUBLE)
#define UNITY_EXCLUDE_DOUBLE
#endif

#if defined(UNITY_EXCLUDE_FLOAT)
#undef UNITY_EXCLUDE_FLOAT
#endif

#ifdef __cplusplus
}
#endif

#endif // _UNITY_CONFIG_H_
