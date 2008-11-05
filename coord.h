#ifndef COORD_H
#define COORD_H

#include "compiler.h"
#include "mat.h"
#include "vec.h"

/* A WGS84 geodetic coordinate in units of radians and meters. */
typedef struct geodetic {
	double latitude, longitude, altitude;
} geodetic;

vec3 geodetic_to_ECEF(geodetic geodetic) ATTR_WARN_UNUSED_RESULT;
mat3 make_LTP_rotation(geodetic geodetic) ATTR_WARN_UNUSED_RESULT;
vec3 ECEF_to_LTP(vec3 origin, mat3 rotation, vec3 ecef) ATTR_WARN_UNUSED_RESULT;
vec3 LTP_to_ECEF(vec3 origin, mat3 rotation, vec3 ltp) ATTR_WARN_UNUSED_RESULT;
geodetic ECEF_to_geodetic(vec3 ecef) ATTR_WARN_UNUSED_RESULT;

#endif /* COORD_H */
