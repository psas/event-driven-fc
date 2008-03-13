#ifndef VEC_H
#define VEC_H
#include <math.h>

enum { X, Y, Z };

typedef double vec3[3];

static inline void vec_add(vec3 dst, vec3 src)
{
	dst[X] += src[X];
	dst[Y] += src[Y];
	dst[Z] += src[Z];
}

static inline double vec_abs(vec3 v)
{
	return sqrt(v[X]*v[X] + v[Y]*v[Y] + v[Z]*v[Z]);
}

#endif /* VEC_H */
