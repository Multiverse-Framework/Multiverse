#ifndef VR_TELEOP_ACTION__VISIBILITY_CONTROL_H_
#define VR_TELEOP_ACTION__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define VR_TELEOP_ACTION_EXPORT __attribute__ ((dllexport))
    #define VR_TELEOP_ACTION_IMPORT __attribute__ ((dllimport))
  #else
    #define VR_TELEOP_ACTION_EXPORT __declspec(dllexport)
    #define VR_TELEOP_ACTION_IMPORT __declspec(dllimport)
  #endif
  #ifdef VR_TELEOP_ACTION_BUILDING_DLL
    #define VR_TELEOP_ACTION_PUBLIC VR_TELEOP_ACTION_EXPORT
  #else
    #define VR_TELEOP_ACTION_PUBLIC VR_TELEOP_ACTION_IMPORT
  #endif
  #define VR_TELEOP_ACTION_PUBLIC_TYPE VR_TELEOP_ACTION_PUBLIC
  #define VR_TELEOP_ACTION_LOCAL
#else
  #define VR_TELEOP_ACTION_EXPORT __attribute__ ((visibility("default")))
  #define VR_TELEOP_ACTION_IMPORT
  #if __GNUC__ >= 4
    #define VR_TELEOP_ACTION_PUBLIC __attribute__ ((visibility("default")))
    #define VR_TELEOP_ACTION_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define VR_TELEOP_ACTION_PUBLIC
    #define VR_TELEOP_ACTION_LOCAL
  #endif
  #define VR_TELEOP_ACTION_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // VR_TELEOP_ACTION__VISIBILITY_CONTROL_H_