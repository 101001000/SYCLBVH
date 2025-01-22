#pragma once

#include "rtcore_common.h"


struct RTC_ALIGN(16) RTCQuaternionDecomposition
{
  float scale_x;
  float scale_y;
  float scale_z;
  float skew_xy;
  float skew_xz;
  float skew_yz;
  float shift_x;
  float shift_y;
  float shift_z;
  float quaternion_r;
  float quaternion_i;
  float quaternion_j;
  float quaternion_k;
  float translation_x;
  float translation_y;
  float translation_z;
};

inline void rtcSetGeometryTransformQuaternion(RTCGeometry geometry, unsigned int timeStep, const struct RTCQuaternionDecomposition* qd) {}