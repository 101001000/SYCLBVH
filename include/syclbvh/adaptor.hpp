
struct RTCDevice{
    RTCDevice(const char*){};
    operator int(){return 1;};
};
enum RTCError{};
typedef void (*RTCErrorFunction)(
      void* userPtr,
      RTCError code,
      const char* str
    );
inline RTCDevice rtcNewDevice(const char* config){ return RTCDevice(config);}
inline int rtcGetDeviceError(RTCDevice device){ return 0;}
inline void rtcSetDeviceErrorFunction(RTCDevice device, RTCErrorFunction error, void* userPtr){}