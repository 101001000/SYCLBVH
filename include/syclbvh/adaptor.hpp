
struct RTCDevice{
    RTCDevice(void*){};
    operator int(){return 0;};
};
enum RTCError{};
typedef void (*RTCErrorFunction)(
      void* userPtr,
      RTCError code,
      const char* str
    );
inline RTCDevice rtcNewDevice(const char* config){ return RTCDevice(nullptr);}
inline int rtcGetDeviceError(RTCDevice device){ return 0;}
inline void rtcSetDeviceErrorFunction(RTCDevice device, RTCErrorFunction error, void* userPtr){}