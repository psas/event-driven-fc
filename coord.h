#ifndef COORD_H
#define COORD_H

#include "mat.h"
#include "vec.h"

enum {
	LATITUDE,
	LONGITUDE,
	ALTITUDE
};

void geodetic_to_ECEF(vec3 ecef, const vec3 geodetic);
void make_origin(vec3 origin, mat3 *rotation, const vec3 geodetic);
void ECEF_to_tangent_plane(vec3 ltp, const vec3 origin, const mat3 *rotation, const vec3 ecef);
void tangent_plane_to_ECEF(vec3 ecef, const vec3 origin, const mat3 *rotation, const vec3 ltp);
void ECEF_to_geodetic(vec3 geodetic, const vec3 ecef);

#endif /* COORD_H */
