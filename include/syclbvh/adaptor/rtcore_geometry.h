#pragma once
#include "rtcore_buffer.h"
#include "rtcore_quaternion.h"



struct RTCScene;

/* Types of geometries */
enum RTCGeometryType
{
  RTC_GEOMETRY_TYPE_TRIANGLE = 0, // triangle mesh
  RTC_GEOMETRY_TYPE_QUAD     = 1, // quad (triangle pair) mesh
  RTC_GEOMETRY_TYPE_GRID     = 2, // grid mesh

  RTC_GEOMETRY_TYPE_SUBDIVISION = 8, // Catmull-Clark subdivision surface

  RTC_GEOMETRY_TYPE_CONE_LINEAR_CURVE   = 15, // Cone linear curves - discontinuous at edge boundaries 
  RTC_GEOMETRY_TYPE_ROUND_LINEAR_CURVE  = 16, // Round (rounded cone like) linear curves 
  RTC_GEOMETRY_TYPE_FLAT_LINEAR_CURVE   = 17, // flat (ribbon-like) linear curves

  RTC_GEOMETRY_TYPE_ROUND_BEZIER_CURVE  = 24, // round (tube-like) Bezier curves
  RTC_GEOMETRY_TYPE_FLAT_BEZIER_CURVE   = 25, // flat (ribbon-like) Bezier curves
  RTC_GEOMETRY_TYPE_NORMAL_ORIENTED_BEZIER_CURVE  = 26, // flat normal-oriented Bezier curves
  
  RTC_GEOMETRY_TYPE_ROUND_BSPLINE_CURVE = 32, // round (tube-like) B-spline curves
  RTC_GEOMETRY_TYPE_FLAT_BSPLINE_CURVE  = 33, // flat (ribbon-like) B-spline curves
  RTC_GEOMETRY_TYPE_NORMAL_ORIENTED_BSPLINE_CURVE  = 34, // flat normal-oriented B-spline curves

  RTC_GEOMETRY_TYPE_ROUND_HERMITE_CURVE = 40, // round (tube-like) Hermite curves
  RTC_GEOMETRY_TYPE_FLAT_HERMITE_CURVE  = 41, // flat (ribbon-like) Hermite curves
  RTC_GEOMETRY_TYPE_NORMAL_ORIENTED_HERMITE_CURVE  = 42, // flat normal-oriented Hermite curves

  RTC_GEOMETRY_TYPE_SPHERE_POINT = 50,
  RTC_GEOMETRY_TYPE_DISC_POINT = 51,
  RTC_GEOMETRY_TYPE_ORIENTED_DISC_POINT = 52,

  RTC_GEOMETRY_TYPE_ROUND_CATMULL_ROM_CURVE = 58, // round (tube-like) Catmull-Rom curves
  RTC_GEOMETRY_TYPE_FLAT_CATMULL_ROM_CURVE  = 59, // flat (ribbon-like) Catmull-Rom curves
  RTC_GEOMETRY_TYPE_NORMAL_ORIENTED_CATMULL_ROM_CURVE  = 60, // flat normal-oriented Catmull-Rom curves

  RTC_GEOMETRY_TYPE_USER     = 120, // user-defined geometry
  RTC_GEOMETRY_TYPE_INSTANCE = 121,  // scene instance
  RTC_GEOMETRY_TYPE_INSTANCE_ARRAY = 122,  // scene instance array
};

enum RTCFormat
{
  RTC_FORMAT_UNDEFINED = 0,

  /* 8-bit unsigned integer */
  RTC_FORMAT_UCHAR = 0x1001,
  RTC_FORMAT_UCHAR2,
  RTC_FORMAT_UCHAR3,
  RTC_FORMAT_UCHAR4,

  /* 8-bit signed integer */
  RTC_FORMAT_CHAR = 0x2001,
  RTC_FORMAT_CHAR2,
  RTC_FORMAT_CHAR3,
  RTC_FORMAT_CHAR4,

  /* 16-bit unsigned integer */
  RTC_FORMAT_USHORT = 0x3001,
  RTC_FORMAT_USHORT2,
  RTC_FORMAT_USHORT3,
  RTC_FORMAT_USHORT4,

  /* 16-bit signed integer */
  RTC_FORMAT_SHORT = 0x4001,
  RTC_FORMAT_SHORT2,
  RTC_FORMAT_SHORT3,
  RTC_FORMAT_SHORT4,

  /* 32-bit unsigned integer */
  RTC_FORMAT_UINT = 0x5001,
  RTC_FORMAT_UINT2,
  RTC_FORMAT_UINT3,
  RTC_FORMAT_UINT4,

  /* 32-bit signed integer */
  RTC_FORMAT_INT = 0x6001,
  RTC_FORMAT_INT2,
  RTC_FORMAT_INT3,
  RTC_FORMAT_INT4,

  /* 64-bit unsigned integer */
  RTC_FORMAT_ULLONG = 0x7001,
  RTC_FORMAT_ULLONG2,
  RTC_FORMAT_ULLONG3,
  RTC_FORMAT_ULLONG4,

  /* 64-bit signed integer */
  RTC_FORMAT_LLONG = 0x8001,
  RTC_FORMAT_LLONG2,
  RTC_FORMAT_LLONG3,
  RTC_FORMAT_LLONG4,

  /* 32-bit float */
  RTC_FORMAT_FLOAT = 0x9001,
  RTC_FORMAT_FLOAT2,
  RTC_FORMAT_FLOAT3,
  RTC_FORMAT_FLOAT4,
  RTC_FORMAT_FLOAT5,
  RTC_FORMAT_FLOAT6,
  RTC_FORMAT_FLOAT7,
  RTC_FORMAT_FLOAT8,
  RTC_FORMAT_FLOAT9,
  RTC_FORMAT_FLOAT10,
  RTC_FORMAT_FLOAT11,
  RTC_FORMAT_FLOAT12,
  RTC_FORMAT_FLOAT13,
  RTC_FORMAT_FLOAT14,
  RTC_FORMAT_FLOAT15,
  RTC_FORMAT_FLOAT16,

  /* 32-bit float matrix (row-major order) */
  RTC_FORMAT_FLOAT2X2_ROW_MAJOR = 0x9122,
  RTC_FORMAT_FLOAT2X3_ROW_MAJOR = 0x9123,
  RTC_FORMAT_FLOAT2X4_ROW_MAJOR = 0x9124,
  RTC_FORMAT_FLOAT3X2_ROW_MAJOR = 0x9132,
  RTC_FORMAT_FLOAT3X3_ROW_MAJOR = 0x9133,
  RTC_FORMAT_FLOAT3X4_ROW_MAJOR = 0x9134,
  RTC_FORMAT_FLOAT4X2_ROW_MAJOR = 0x9142,
  RTC_FORMAT_FLOAT4X3_ROW_MAJOR = 0x9143,
  RTC_FORMAT_FLOAT4X4_ROW_MAJOR = 0x9144,

  /* 32-bit float matrix (column-major order) */
  RTC_FORMAT_FLOAT2X2_COLUMN_MAJOR = 0x9222,
  RTC_FORMAT_FLOAT2X3_COLUMN_MAJOR = 0x9223,
  RTC_FORMAT_FLOAT2X4_COLUMN_MAJOR = 0x9224,
  RTC_FORMAT_FLOAT3X2_COLUMN_MAJOR = 0x9232,
  RTC_FORMAT_FLOAT3X3_COLUMN_MAJOR = 0x9233,
  RTC_FORMAT_FLOAT3X4_COLUMN_MAJOR = 0x9234,
  RTC_FORMAT_FLOAT4X2_COLUMN_MAJOR = 0x9242,
  RTC_FORMAT_FLOAT4X3_COLUMN_MAJOR = 0x9243,
  RTC_FORMAT_FLOAT4X4_COLUMN_MAJOR = 0x9244,

  /* special 12-byte format for grids */
  RTC_FORMAT_GRID = 0xA001,

  RTC_FORMAT_QUATERNION_DECOMPOSITION = 0xB001,
};

enum RTCSubdivisionMode
{
  RTC_SUBDIVISION_MODE_NO_BOUNDARY     = 0,
  RTC_SUBDIVISION_MODE_SMOOTH_BOUNDARY = 1,
  RTC_SUBDIVISION_MODE_PIN_CORNERS     = 2,
  RTC_SUBDIVISION_MODE_PIN_BOUNDARY    = 3,
  RTC_SUBDIVISION_MODE_PIN_ALL         = 4,
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
    RTCGeometry(std::nullptr_t){}
    RTCGeometryImpl* impl = nullptr;
    operator bool() const { return impl; }
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
inline void rtcReleaseGeometry(RTCGeometry geometry){};

inline unsigned int rtcGetGeometryFace(RTCGeometry geometry, unsigned int edgeID);

inline unsigned int rtcGetGeometryNextHalfEdge(RTCGeometry geometry, unsigned int edgeID) {return 0;}
inline unsigned int rtcGetGeometryOppositeHalfEdge(RTCGeometry geometry, unsigned int topologyID, unsigned int edgeID){return 0;}
inline unsigned int rtcGetGeometryFirstHalfEdge(RTCGeometry geometry, unsigned int faceID) {return 0;}
inline unsigned int rtcGetGeometryPreviousHalfEdge(RTCGeometry geometry, unsigned int edgeID){return 0;}
inline void rtcSetSharedGeometryBuffer(RTCGeometry geometry, enum RTCBufferType type, unsigned int slot, enum RTCFormat format, const void* ptr, size_t byteOffset, size_t byteStride, size_t itemCount){}
inline void rtcSetGeometryTimeStepCount(RTCGeometry geometry, unsigned int timeStepCount){}
inline void rtcSetGeometryTimeRange(RTCGeometry geometry, float startTime, float endTime){}
inline void rtcSetGeometryUserData(RTCGeometry geometry, void* ptr) {}
inline void rtcSetGeometrySubdivisionMode(RTCGeometry geometry, unsigned int topologyID, enum RTCSubdivisionMode mode){}
inline void rtcSetGeometryTopologyCount(RTCGeometry geometry, unsigned int topologyCount){}
inline void rtcSetGeometryVertexAttributeTopology(RTCGeometry geometry, unsigned int vertexAttributeID, unsigned int topologyID){}
inline void rtcSetGeometryVertexAttributeCount(RTCGeometry geometry, unsigned int vertexAttributeCount) {}
inline void rtcSetGeometryTessellationRate(RTCGeometry geometry, float tessellationRate) {}
void rtcSetGeometryInstancedScene(RTCGeometry geometry, RTCScene scene);