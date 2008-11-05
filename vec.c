#include "vec.h"

vec3 vec_add(vec3 a, vec3 b)
{
	return (vec3){{
		.x = a.x + b.x,
		.y = a.y + b.y,
		.z = a.z + b.z
	}};
}

double vec_abs(vec3 v)
{
	return sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
}
