#pragma once

#include <cstddef>

#if defined(_WIN32) || defined(__CYGWIN__)
#if defined(PATH_GENERATE_BUILD_DLL)
#define PATH_GENERATE_API __declspec(dllexport)
#else
#define PATH_GENERATE_API __declspec(dllimport)
#endif
#else
#define PATH_GENERATE_API __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

// Returns a newly allocated UTF-8 JSON string. Free it with pg_free_string.
PATH_GENERATE_API const char* pg_plan_from_json(const char* input_json);

// Frees a string returned by pg_plan_from_json.
PATH_GENERATE_API void pg_free_string(const char* str);

#ifdef __cplusplus
}  // extern "C"
#endif
