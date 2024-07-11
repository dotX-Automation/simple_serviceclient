#ifndef SIMPLE_SERVICECLIENT__VISIBILITY_CONTROL_H_
#define SIMPLE_SERVICECLIENT__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define SIMPLE_SERVICECLIENT_EXPORT __attribute__ ((dllexport))
    #define SIMPLE_SERVICECLIENT_IMPORT __attribute__ ((dllimport))
  #else
    #define SIMPLE_SERVICECLIENT_EXPORT __declspec(dllexport)
    #define SIMPLE_SERVICECLIENT_IMPORT __declspec(dllimport)
  #endif
  #ifdef SIMPLE_SERVICECLIENT_BUILDING_LIBRARY
    #define SIMPLE_SERVICECLIENT_PUBLIC SIMPLE_SERVICECLIENT_EXPORT
  #else
    #define SIMPLE_SERVICECLIENT_PUBLIC SIMPLE_SERVICECLIENT_IMPORT
  #endif
  #define SIMPLE_SERVICECLIENT_PUBLIC_TYPE SIMPLE_SERVICECLIENT_PUBLIC
  #define SIMPLE_SERVICECLIENT_LOCAL
#else
  #define SIMPLE_SERVICECLIENT_EXPORT __attribute__ ((visibility("default")))
  #define SIMPLE_SERVICECLIENT_IMPORT
  #if __GNUC__ >= 4
    #define SIMPLE_SERVICECLIENT_PUBLIC __attribute__ ((visibility("default")))
    #define SIMPLE_SERVICECLIENT_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define SIMPLE_SERVICECLIENT_PUBLIC
    #define SIMPLE_SERVICECLIENT_LOCAL
  #endif
  #define SIMPLE_SERVICECLIENT_PUBLIC_TYPE
#endif

#endif  // SIMPLE_SERVICECLIENT__VISIBILITY_CONTROL_H_
