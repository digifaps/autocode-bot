#ifndef STEREO_VISION__VISIBILITY_H_
#define STEREO_VISION__VISIBILITY_H_

// Visibility control macros for shared library exports

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define STEREO_VISION_EXPORT __attribute__ ((dllexport))
    #define STEREO_VISION_IMPORT __attribute__ ((dllimport))
  #else
    #define STEREO_VISION_EXPORT __declspec(dllexport)
    #define STEREO_VISION_IMPORT __declspec(dllimport)
  #endif
  #ifdef STEREO_VISION_BUILDING_DLL
    #define STEREO_VISION_PUBLIC STEREO_VISION_EXPORT
  #else
    #define STEREO_VISION_PUBLIC STEREO_VISION_IMPORT
  #endif
  #define STEREO_VISION_LOCAL
#else
  #define STEREO_VISION_EXPORT __attribute__ ((visibility("default")))
  #define STEREO_VISION_IMPORT
  #if __GNUC__ >= 4
    #define STEREO_VISION_PUBLIC __attribute__ ((visibility("default")))
    #define STEREO_VISION_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define STEREO_VISION_PUBLIC
    #define STEREO_VISION_LOCAL
  #endif
#endif

#endif  // STEREO_VISION__VISIBILITY_H_
