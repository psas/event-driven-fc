#include <math.h>
#include <string.h>
#include "mat.h"

/* http://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToMatrix/ */
void axis_angle_to_mat3(mat3 *dst, vec3 axis_angle)
{
	double x = axis_angle[X], y = axis_angle[Y], z = axis_angle[Z];

	double angle = sqrt(x*x + y*y + z*z);
	x /= angle;
	y /= angle;
	z /= angle;

	double c = cos(angle);
	double s = sin(angle);
	double t = 1.0 - c;

	dst->component[0][0] = c + x*x*t;
	dst->component[1][1] = c + y*y*t;
	dst->component[2][2] = c + z*z*t;

	double tmp1 = x*y*t;
	double tmp2 = z*s;
	dst->component[1][0] = tmp1 + tmp2;
	dst->component[0][1] = tmp1 - tmp2;
	tmp1 = x*z*t;
	tmp2 = y*s;
	dst->component[2][0] = tmp1 - tmp2;
	dst->component[0][2] = tmp1 + tmp2;
	tmp1 = y*z*t;
	tmp2 = x*s;
	dst->component[2][1] = tmp1 + tmp2;
	dst->component[1][2] = tmp1 - tmp2;
}

void mat3_mul(mat3 *dst, mat3 *left, mat3 *right)
{
	mat3 tmp = { };
	int i, j, k;
	for(i = 0; i < 3; ++i)
		for(j = 0; j < 3; ++j)
			for(k = 0; k < 3; ++k)
				tmp.component[i][j] += left->component[i][k] * right->component[k][j];
	*dst = tmp;
}

void mat3_vec3_mul(vec3 dst, const mat3 *left, const vec3 right)
{
	vec3 tmp = { };
	int i, k;
	for(i = 0; i < 3; ++i)
		for(k = 0; k < 3; ++k)
			tmp[i] += left->component[i][k] * right[k];
	memcpy(dst, tmp, sizeof(tmp));
}
