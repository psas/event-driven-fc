/* Copyright © 2010 Portland State Aerospace Society
 * See version control history for detailed authorship information.
 *
 * This program is licensed under the GPL version 2 or later.  Please see the
 * file COPYING in the source distribution of this software for license terms.
 */
#include "vec.h"

vec3 vec_add(vec3 a, vec3 b)
{
	return (vec3){{
		.x = a.x + b.x,
		.y = a.y + b.y,
		.z = a.z + b.z
	}};
}

vec3 vec_sub(vec3 a, vec3 b)
{
	return (vec3){{
		.x = a.x - b.x,
		.y = a.y - b.y,
		.z = a.z - b.z
	}};
}

double vec_dot(vec3 a, vec3 b)
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

double vec_abs(vec3 v)
{
	return sqrt(vec_dot(v, v));
}

vec3 vec_scale(vec3 v, double scale)
{
	return (vec3){{
		.x = scale*v.x,
		.y = scale*v.y,
		.z = scale*v.z
	}};
}
