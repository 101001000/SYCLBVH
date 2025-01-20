#pragma once
#define RTC_INVALID_GEOMETRY_ID ((unsigned int)-1)

#if defined(_WIN32) && !defined(__MINGW32__)
#  define RTC_ALIGN(...) __declspec(align(__VA_ARGS__))
#else
#  define RTC_ALIGN(...) __attribute__((aligned(__VA_ARGS__)))
#endif