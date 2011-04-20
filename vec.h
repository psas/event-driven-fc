/* Copyright © 2010 Portland State Aerospace Society
 * See version control history for detailed authorship information.
 *
 * This program is licensed under the GPL version 2 or later.  Please see the
 * file COPYING in the source distribution of this software for license terms.
 */
#ifndef VEC_H
#define VEC_H
#include <math.h>
#include "compiler.h"

typedef struct vec3 {
	double x, y, z;
} vec3;

union vec_array {
	vec3 vec;
	double component[3];
};

vec3 vec_add(vec3 a, vec3 b) ATTR_WARN_UNUSED_RESULT;
vec3 vec_sub(vec3 a, vec3 b) ATTR_WARN_UNUSED_RESULT;
double vec_dot(vec3 a, vec3 b) ATTR_WARN_UNUSED_RESULT;
double vec_abs(vec3 v) ATTR_WARN_UNUSED_RESULT;
vec3 vec_scale(vec3 v, double scale) ATTR_WARN_UNUSED_RESULT;

#endif /* VEC_H */
