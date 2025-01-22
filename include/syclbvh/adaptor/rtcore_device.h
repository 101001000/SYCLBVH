#pragma once
#include <sycl/sycl.hpp>

enum RTCError{
    RTC_ERROR_NONE = 0
};



typedef void (*RTCErrorFunction)(
      void* userPtr,
      RTCError code,
      const char* str
    );

enum RTCDeviceProperty
{
  RTC_DEVICE_PROPERTY_VERSION       = 0,
  RTC_DEVICE_PROPERTY_VERSION_MAJOR = 1,
  RTC_DEVICE_PROPERTY_VERSION_MINOR = 2,
  RTC_DEVICE_PROPERTY_VERSION_PATCH = 3,

  RTC_DEVICE_PROPERTY_NATIVE_RAY4_SUPPORTED  = 32,
  RTC_DEVICE_PROPERTY_NATIVE_RAY8_SUPPORTED  = 33,
  RTC_DEVICE_PROPERTY_NATIVE_RAY16_SUPPORTED = 34
};

struct RTCDevice{
public:
    RTCDevice(){
        q = sycl::queue{sycl::default_selector_v};
    };
    RTCDevice(const char*) : RTCDevice(){}
    operator int(){return 1;};
    sycl::queue q;
};


inline ssize_t rtcGetDeviceProperty(RTCDevice device, enum RTCDeviceProperty prop){return 0;}
inline void rtcSetDeviceProperty(RTCDevice device, const enum RTCDeviceProperty prop, ssize_t value){}

inline RTCDevice rtcNewDevice(const char* config){ return RTCDevice(config);}
inline RTCError rtcGetDeviceError(RTCDevice device){ return RTC_ERROR_NONE;}
inline void rtcSetDeviceErrorFunction(RTCDevice device, RTCErrorFunction error, void* userPtr){}
inline void rtcReleaseDevice(RTCDevice device){};
inline const char* rtcGetErrorString(RTCError error){return "";}