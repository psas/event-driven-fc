#ifndef MAT_H
#define MAT_H

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

mat3 axis_angle_to_mat3(vec3 axis_angle);
mat3 mat3_mul(mat3 left, mat3 right);
vec3 mat3_vec3_mul(mat3 left, vec3 right);

#endif /* MAT_H */
