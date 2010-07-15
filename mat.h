/* Copyright © 2010 Portland State Aerospace Society
 * See version control history for detailed authorship information.
 *
 * This program is licensed under the GPL version 2 or later.  Please see the
 * file COPYING in the source distribution of this software for license terms.
 */
#ifndef MAT_H
#define MAT_H

#include "compiler.h"
#include "vec.h"

typedef union mat3
{
	struct {
		double x1, y1, z1;
		double x2, y2, z2;
		double x3, y3, z3;
	};
	double component[3][3];
} mat3;

mat3 axis_angle_to_mat3(vec3 axis_angle) ATTR_WARN_UNUSED_RESULT;
mat3 mat3_mul(mat3 left, mat3 right) ATTR_WARN_UNUSED_RESULT;
vec3 mat3_vec3_mul(mat3 left, vec3 right) ATTR_WARN_UNUSED_RESULT;
mat3 mat3_transpose(mat3 m) ATTR_WARN_UNUSED_RESULT;

#endif /* MAT_H */
