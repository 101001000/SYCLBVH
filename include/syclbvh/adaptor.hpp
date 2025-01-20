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

    void* buffers[2];
    int buffer_sizes[2];

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
    scene.impl = sycl::malloc_shared<RTCSceneImpl>(1, device.q);
    return scene;
};  
inline RTCGeometry rtcNewGeometry(RTCDevice device, int type){
    RTCGeometry geometry{};
    geometry.impl = sycl::malloc_shared<RTCGeometryImpl>(1, device.q);
    geometry.impl->device = device;
    geometry.impl->type = (RTCGeometryType)type;
    return geometry;
};
inline void* rtcSetNewGeometryBuffer(RTCGeometry geometry, RTCBufferType type, unsigned int slot, RTCFormat format, std::size_t byteStride, std::size_t itemCount){

    void* buffer = nullptr;

    if(format == RTC_FORMAT_FLOAT3){
        buffer =(void*) sycl::malloc_shared<sycl::float3>(itemCount, geometry.impl->device.q);
    }

    if(format == RTC_FORMAT_UINT3){
        buffer =(void*) sycl::malloc_shared<sycl::uint3>(itemCount, geometry.impl->device.q);
    }
   
    geometry.impl->buffers[type] = buffer;
    geometry.impl->buffer_sizes[type] = itemCount;
    return buffer;
}
inline void rtcCommitGeometry(RTCGeometry geometry){};
inline void rtcAttachGeometry(RTCScene scene, RTCGeometry geometry){
    scene.impl->geometries.push_back(geometry);
};
inline void rtcReleaseGeometry(RTCGeometry geometry){};
inline void rtcReleaseScene(RTCScene scene){};
inline void rtcReleaseDevice(RTCDevice device){};
inline void rtcCommitScene(RTCScene scene){};

struct Vec3 {
    float x, y, z;
    Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_){};
    Vec3 operator-(const Vec3& b) const {return {x-b.x, y-b.y, z-b.z};}
    Vec3 cross(const Vec3& b) const {
        return { y*b.z - z*b.y,  z*b.x - x*b.z,  x*b.y - y*b.x };
    }
    float dot(const Vec3& b) const {return x*b.x + y*b.y + z*b.z;}
};

bool intersectRayTriangle(const Vec3& orig, const Vec3& dir,
                          const Vec3& v0, const Vec3& v1, const Vec3& v2,
                          float& tOut, float& uOut, float& vOut){
    const float EPS = 1e-8f;
    Vec3 edge1 = v1 - v0, edge2 = v2 - v0;
    Vec3 pvec  = dir.cross(edge2);
    float det  = edge1.dot(pvec);

    if (std::fabs(det) < EPS) return false;
    float invDet = 1.f / det;

    Vec3 tvec = orig - v0;
    float u   = tvec.dot(pvec) * invDet;
    if (u < 0.f || u > 1.f) return false;

    Vec3 qvec = tvec.cross(edge1);
    float v   = dir.dot(qvec) * invDet;
    if (v < 0.f || (u + v) > 1.f) return false;

    float t = edge2.dot(qvec) * invDet;
    if (t < EPS) return false;

    tOut = t;
    uOut = u;
    vOut = v;
    return true;
}

void rtcIntersect1(RTCScene scene, RTCRayHit* rayhit){

    RTCGeometry geometry = scene.impl->geometries[0];

    sycl::queue& q = geometry.impl->device.q;

    RTCRayHit* dev_hit = sycl::malloc_shared<RTCRayHit>(1, q);

    std::memcpy(dev_hit, rayhit, sizeof(RTCRayHit));

    q.wait();

    std::cout << geometry.impl->buffer_sizes[RTC_BUFFER_TYPE_INDEX];

    q.submit([&](sycl::handler& h){
        h.single_task([=](){

            for(int i = 0; i < geometry.impl->buffer_sizes[RTC_BUFFER_TYPE_INDEX]; i+=3){
       
                unsigned* i_buffer = static_cast<unsigned*>(geometry.impl->buffers[RTC_BUFFER_TYPE_INDEX]);
                float* v_buffer = static_cast<float*>(geometry.impl->buffers[RTC_BUFFER_TYPE_VERTEX]);

                unsigned i0 = i_buffer[i+0];
                unsigned i1 = i_buffer[i+1];
                unsigned i2 = i_buffer[i+2];

                float v0x = v_buffer[i0*3+0];
                float v0y = v_buffer[i0*3+1];
                float v0z = v_buffer[i0*3+2];

                float v1x = v_buffer[i1*3+0];
                float v1y = v_buffer[i1*3+1];
                float v1z = v_buffer[i1*3+2];

                float v2x = v_buffer[i2*+0];
                float v2y = v_buffer[i2*3+1];
                float v2z = v_buffer[i2*3+2];


                Vec3 orig = Vec3(dev_hit->ray.org_x, dev_hit->ray.org_y, dev_hit->ray.org_z);
                Vec3 dir = Vec3(dev_hit->ray.dir_x, dev_hit->ray.dir_y, dev_hit->ray.dir_z);
                Vec3 v0 = Vec3(v0x, v0y, v0z);
                Vec3 v1 = Vec3(v1x, v1y, v1z);
                Vec3 v2 = Vec3(v2x, v2y, v2z);

                float t, u ,v;

                intersectRayTriangle(orig, dir, v0, v1, v2, t, u, v);
                if(t <  dev_hit->ray.tfar){
                    dev_hit->ray.tfar = t;
                    dev_hit->hit.geomID = 1;
                    dev_hit->hit.primID = static_cast<unsigned int>(i/3);
                    dev_hit->hit.u = u;
                    dev_hit->hit.v = v;
                }
            }
        });
    }).wait();

    std::memcpy(rayhit, dev_hit, sizeof(RTCRayHit));
}