#pragma once


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