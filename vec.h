#ifndef VEC_H
#define VEC_H
#include <math.h>

typedef union vec3 {
	struct {
		double x, y, z;
	};
	double component[3];
} vec3;

static inline vec3 vec_add(vec3 a, vec3 b)
{
	return (vec3){{
		.x = a.x + b.x,
		.y = a.y + b.y,
		.z = a.z + b.z
	}};
}

static inline double vec_abs(vec3 v)
{
	return sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
}

/* A WGS84 geodetic coordinate in units of radians and meters. */
typedef struct geodetic {
	double latitude, longitude, altitude;
} geodetic;

#endif /* VEC_H */
