// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include <syclbvh/adaptor/rtcore_config.h>

/* #undef EMBREE_RAY_MASK */
/* #undef EMBREE_STAT_COUNTERS */
/* #undef EMBREE_BACKFACE_CULLING */
/* #undef EMBREE_BACKFACE_CULLING_CURVES */
/* #undef EMBREE_BACKFACE_CULLING_SPHERES */
/* #undef EMBREE_FILTER_FUNCTION */
/* #undef EMBREE_IGNORE_INVALID_RAYS */
/* #undef EMBREE_GEOMETRY_TRIANGLE */
/* #undef EMBREE_GEOMETRY_QUAD */
/* #undef EMBREE_GEOMETRY_CURVE */
/* #undef EMBREE_GEOMETRY_SUBDIVISION */
/* #undef EMBREE_GEOMETRY_USER */
/* #undef EMBREE_GEOMETRY_INSTANCE */
// EMBREE_GEOMETRY_INSTANCE_ARRAY is defined in rtcore_config.h
/* #undef EMBREE_GEOMETRY_GRID */
/* #undef EMBREE_GEOMETRY_POINT */
/* #undef EMBREE_RAY_PACKETS */
/* #undef EMBREE_COMPACT_POLYS */

#define EMBREE_CURVE_SELF_INTERSECTION_AVOIDANCE_FACTOR 
/* #undef EMBREE_DISC_POINT_SELF_INTERSECTION_AVOIDANCE */

#if defined(EMBREE_GEOMETRY_TRIANGLE)
  #define IF_ENABLED_TRIS(x) x
#else
  #define IF_ENABLED_TRIS(x)
#endif

#if defined(EMBREE_GEOMETRY_QUAD)
  #define IF_ENABLED_QUADS(x) x
#else
  #define IF_ENABLED_QUADS(x)
#endif

#if defined(EMBREE_GEOMETRY_CURVE) || defined(EMBREE_GEOMETRY_POINT)
  #define IF_ENABLED_CURVES_OR_POINTS(x) x
#else
  #define IF_ENABLED_CURVES_OR_POINTS(x)
#endif

#if defined(EMBREE_GEOMETRY_CURVE)
  #define IF_ENABLED_CURVES(x) x
#else
  #define IF_ENABLED_CURVES(x)
#endif

#if defined(EMBREE_GEOMETRY_POINT)
  #define IF_ENABLED_POINTS(x) x
#else
  #define IF_ENABLED_POINTS(x)
#endif

#if defined(EMBREE_GEOMETRY_SUBDIVISION)
  #define IF_ENABLED_SUBDIV(x) x
#else
  #define IF_ENABLED_SUBDIV(x)
#endif

#if defined(EMBREE_GEOMETRY_USER)
  #define IF_ENABLED_USER(x) x
#else
  #define IF_ENABLED_USER(x)
#endif

#if defined(EMBREE_GEOMETRY_INSTANCE)
  #define IF_ENABLED_INSTANCE(x) x
#else
  #define IF_ENABLED_INSTANCE(x)
#endif

#if defined(EMBREE_GEOMETRY_INSTANCE_ARRAY)
  #define IF_ENABLED_INSTANCE_ARRAY(x) x
#else
  #define IF_ENABLED_INSTANCE_ARRAY(x)
#endif

#if defined(EMBREE_GEOMETRY_GRID)
  #define IF_ENABLED_GRIDS(x) x
#else
  #define IF_ENABLED_GRIDS(x)
#endif




