#include <cstddef> 

struct RTCDevice{
    RTCDevice(const char*){};
    operator int(){return 1;};
};
struct RTCScene{};
struct RTCGeometry{};

enum RTCError{};
enum RTCFormat{
    RTC_FORMAT_FLOAT3,
    RTC_FORMAT_UINT3
};
enum RTCBufferType{
    RTC_BUFFER_TYPE_VERTEX,
    RTC_BUFFER_TYPE_INDEX
};
enum RTCGeometryType{
    RTC_GEOMETRY_TYPE_TRIANGLE
};

typedef void (*RTCErrorFunction)(
      void* userPtr,
      RTCError code,
      const char* str
    );

inline RTCDevice rtcNewDevice(const char* config){ return RTCDevice(config);}
inline int rtcGetDeviceError(RTCDevice device){ return 0;}
inline void rtcSetDeviceErrorFunction(RTCDevice device, RTCErrorFunction error, void* userPtr){}
inline RTCScene rtcNewScene(RTCDevice device){ return RTCScene();};
inline RTCGeometry rtcNewGeometry(RTCDevice device, int type){ return RTCGeometry();};
inline void* rtcSetNewGeometryBuffer(RTCGeometry geometry, RTCBufferType type, unsigned int slot, RTCFormat format, std::size_t byteStride, std::size_t itemCount){return nullptr;}
inline void rtcCommitGeometry(RTCGeometry geometry){};
inline void rtcAttachGeometry(RTCScene scene, RTCGeometry geometry){};
inline void rtcReleaseGeometry(RTCGeometry geometry){};
inline void rtcCommitScene(RTCScene scene){};