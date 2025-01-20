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



struct RTCDevice{
public:
    RTCDevice(){
        q = sycl::queue{sycl::default_selector_v};
    };
    RTCDevice(const char*) : RTCDevice(){}
    operator int(){return 1;};
    sycl::queue q;
};

RTCDevice rtcNewDevice(const char* config){ return RTCDevice(config);}
RTCError rtcGetDeviceError(RTCDevice device){ return RTC_ERROR_NONE;}
void rtcSetDeviceErrorFunction(RTCDevice device, RTCErrorFunction error, void* userPtr){}
void rtcReleaseDevice(RTCDevice device){};