#ifndef VEC_H
#define VEC_H
#include <math.h>
#include "compiler.h"

typedef union vec3 {
	struct {
		double x, y, z;
	};
	double component[3];
} vec3;

vec3 vec_add(vec3 a, vec3 b) ATTR_WARN_UNUSED_RESULT;
vec3 vec_sub(vec3 a, vec3 b) ATTR_WARN_UNUSED_RESULT;
double vec_dot(vec3 a, vec3 b) ATTR_WARN_UNUSED_RESULT;
double vec_abs(vec3 v) ATTR_WARN_UNUSED_RESULT;
vec3 vec_scale(vec3 v, double scale) ATTR_WARN_UNUSED_RESULT;

#endif /* VEC_H */
