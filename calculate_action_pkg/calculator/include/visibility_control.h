#ifndef CALCULATE_ACTION_PKG__VISIBILITY_CONTROL_H_
#define CALCULATE_ACTION_PKG__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C" {
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define CALCULATE_ACTION_PKG_EXPORT __attribute__((dllexport))
#define CALCULATE_ACTION_PKG_IMPORT __attribute__((dllimport))
#else
#define CALCULATE_ACTION_PKG_EXPORT __declspec(dllexport)
#define CALCULATE_ACTION_PKG_IMPORT __declspec(dllimport)
#endif
#ifdef CALCULATE_ACTION_PKG_BUILDING_DLL
#define CALCULATE_ACTION_PKG_PUBLIC CALCULATE_ACTION_PKG_EXPORT
#else
#define CALCULATE_ACTION_PKG_PUBLIC CALCULATE_ACTION_PKG_IMPORT
#endif
#define CALCULATE_ACTION_PKG_PUBLIC_TYPE CALCULATE_ACTION_PKG_PUBLIC
#define CALCULATE_ACTION_PKG_LOCAL
#else
#define CALCULATE_ACTION_PKG_EXPORT __attribute__((visibility("default")))
#define CALCULATE_ACTION_PKG_IMPORT
#if __GNUC__ >= 4
#define CALCULATE_ACTION_PKG_PUBLIC __attribute__((visibility("default")))
#define CALCULATE_ACTION_PKG_LOCAL __attribute__((visibility("hidden")))
#else
#define CALCULATE_ACTION_PKG_PUBLIC
#define CALCULATE_ACTION_PKG_LOCAL
#endif
#define CALCULATE_ACTION_PKG_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif // CALCULATE_ACTION_PKG__VISIBILITY_CONTROL_H_