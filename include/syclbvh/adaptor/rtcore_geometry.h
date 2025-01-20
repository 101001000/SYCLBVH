#pragma once
#include "rtcore_buffer.h"

enum RTCGeometryType{
    RTC_GEOMETRY_TYPE_TRIANGLE
};

enum RTCFormat{
    RTC_FORMAT_FLOAT3,
    RTC_FORMAT_UINT3
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

inline RTCGeometry rtcNewGeometry(RTCDevice device, int type){
    RTCGeometry geometry{};
    geometry.impl = sycl::malloc_shared<RTCGeometryImpl>(1, device.q);
    geometry.impl->device = device;
    geometry.impl->type = (RTCGeometryType)type;
    return geometry;
};

void* rtcSetNewGeometryBuffer(RTCGeometry geometry, RTCBufferType type, unsigned int slot, RTCFormat format, std::size_t byteStride, std::size_t itemCount){

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

void rtcCommitGeometry(RTCGeometry geometry){};
void rtcReleaseGeometry(RTCGeometry geometry){};