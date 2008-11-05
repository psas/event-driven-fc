#ifndef COORD_H
#define COORD_H

#include "mat.h"
#include "vec.h"

/* A WGS84 geodetic coordinate in units of radians and meters. */
typedef struct geodetic {
	double latitude, longitude, altitude;
} geodetic;

vec3 geodetic_to_ECEF(geodetic geodetic);
mat3 make_LTP_rotation(geodetic geodetic);
vec3 ECEF_to_LTP(vec3 origin, mat3 rotation, vec3 ecef);
vec3 LTP_to_ECEF(vec3 origin, mat3 rotation, vec3 ltp);
geodetic ECEF_to_geodetic(vec3 ecef);

#endif /* COORD_H */
