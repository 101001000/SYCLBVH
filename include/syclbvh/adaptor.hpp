#include <cstddef> 
#include <sycl/sycl.hpp>

#if defined(_WIN32) && !defined(__MINGW32__)
#  define RTC_ALIGN(...) __declspec(align(__VA_ARGS__))
#else
#  define RTC_ALIGN(...) __attribute__((aligned(__VA_ARGS__)))
#endif

#define RTC_INVALID_GEOMETRY_ID ((unsigned int)-1)

struct RTCDevice{
public:
    RTCDevice(){
        q = sycl::queue{sycl::default_selector_v};
    };
    RTCDevice(const char*) : RTCDevice(){}
    operator int(){return 1;};
    sycl::queue q;
};


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

struct RTCGeometryImpl{
    RTCDevice device;
    RTCGeometryType type;
    void print(){
        std::cout << "RTCGeometry: " << type << std::endl;
    }
};

struct RTCGeometry{
    RTCGeometry(){}
    RTCGeometryImpl* impl;
};

struct RTCSceneImpl{
    void print(){
        std::cout << "RTCScene: " << std::endl;
        for (auto g : geometries){
            g.impl->print();
        }
    }
    std::vector<RTCGeometry> geometries;
};

struct RTCScene{
    RTCSceneImpl* impl;
};

typedef void (*RTCErrorFunction)(
      void* userPtr,
      RTCError code,
      const char* str
    );

#define RTC_MAX_INSTANCE_LEVEL_COUNT 1
struct RTC_ALIGN(16) RTCHit
{
  float Ng_x;       
  float Ng_y;  
  float Ng_z; 
  float u;  
  float v; 

  unsigned int primID; 
  unsigned int geomID;
  unsigned int instID[RTC_MAX_INSTANCE_LEVEL_COUNT]; 
};

struct RTC_ALIGN(16) RTCRay
{
  float org_x;       
  float org_y; 
  float org_z;   
  float tnear;

  float dir_x;       
  float dir_y; 
  float dir_z;       
  float time;

  float tfar; 
  unsigned int mask;
  unsigned int id;
  unsigned int flags;
};

struct RTCRayHit
{
  struct RTCRay ray;
  struct RTCHit hit;
};


inline RTCDevice rtcNewDevice(const char* config){ return RTCDevice(config);}
inline int rtcGetDeviceError(RTCDevice device){ return 0;}
inline void rtcSetDeviceErrorFunction(RTCDevice device, RTCErrorFunction error, void* userPtr){}
inline RTCScene rtcNewScene(RTCDevice device){
    RTCScene scene = RTCScene();
    scene.impl = new RTCSceneImpl();
    return scene;
};  
inline RTCGeometry rtcNewGeometry(RTCDevice device, int type){
    RTCGeometry geometry{};
    geometry.impl = new RTCGeometryImpl();
    geometry.impl->device = device;
    geometry.impl->type = (RTCGeometryType)type;
    return geometry;
};
inline void* rtcSetNewGeometryBuffer(RTCGeometry geometry, RTCBufferType type, unsigned int slot, RTCFormat format, std::size_t byteStride, std::size_t itemCount){
   return (void*) sycl::malloc_shared<char>(itemCount, geometry.impl->device.q);
}
inline void rtcCommitGeometry(RTCGeometry geometry){};
inline void rtcAttachGeometry(RTCScene scene, RTCGeometry geometry){
    scene.impl->geometries.push_back(geometry);
};
inline void rtcReleaseGeometry(RTCGeometry geometry){};
inline void rtcReleaseScene(RTCScene scene){};
inline void rtcReleaseDevice(RTCDevice device){};
inline void rtcCommitScene(RTCScene scene){};

void rtcIntersect1(RTCScene scene, RTCRayHit* rayhit){

    sycl::queue& q = scene.impl->geometries[0].impl->device.q;

    RTCRayHit* dev_hit = sycl::malloc_shared<RTCRayHit>(1, q);

    std::memcpy(dev_hit, rayhit, sizeof(RTCRayHit));

    q.wait();

    q.submit([&](sycl::handler& h){
        h.single_task([=](){
            dev_hit->hit.geomID = 1;
        });
    }).wait();

    std::memcpy(rayhit, dev_hit, sizeof(RTCRayHit));
}