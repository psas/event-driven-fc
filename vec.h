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
vec3 vec_sub(vec3 a, vec3 b);
double vec_abs(vec3 v);
vec3 vec_scale(vec3 v, double scale);

#endif /* VEC_H */
