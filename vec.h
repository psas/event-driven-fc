#ifndef VEC_H
#define VEC_H
#include <math.h>

typedef union vec3 {
	struct {
		double x, y, z;
	};
	double component[3];
} vec3;

vec3 vec_add(vec3 a, vec3 b);
double vec_abs(vec3 v);

#endif /* VEC_H */
