#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include "coord.h"

static const double DOUBLE_ERROR_BOUND = 0.000001;

static bool vec3_similar(const vec3 expected, const vec3 actual)
{
	int i;
	for(i = 0; i < 3; ++i)
		if(fabs(expected.component[i] - actual.component[i]) > DOUBLE_ERROR_BOUND)
			return false;
	return true;
}

static bool geodetic_similar(geodetic expected, geodetic actual)
{
	return (fabs(expected.latitude  - actual.latitude ) < DOUBLE_ERROR_BOUND)
	    && (fabs(expected.longitude - actual.longitude) < DOUBLE_ERROR_BOUND)
	    && (fabs(expected.altitude  - actual.altitude ) < DOUBLE_ERROR_BOUND);
}

static bool mat3_similar(const mat3 expected, const mat3 actual)
{
	int i, j;
	for(i = 0; i < 3; ++i)
		for(j = 0; j < 3; ++j)
			if(fabs(expected.component[i][j] - actual.component[i][j]) > DOUBLE_ERROR_BOUND)
				return false;
	return true;
}

static void mat3_show(const mat3 mat)
{
	int i;
	for(i = 0; i < 3; ++i)
		printf("| % f % f % f |\n", mat.component[i][0], mat.component[i][1], mat.component[i][2]);
}

int main(void)
{
	int fail = 0;
	const geodetic geodetic_ref = {
		.latitude = 0.59341195,
		.longitude = -2.0478571,
		.altitude = 251.702,
	};
	const vec3 ecef_ref = {{
		.x = -2430601.795708,
		.y = -4702442.736094,
		.z =  3546587.336483,
	}};
	const mat3 rotation_ref = { .component = {
		{  0.88834836, -0.45917011,  0.00000000 },
		{  0.25676467,  0.49675810,  0.82903757 },
		{ -0.38066927, -0.73647416,  0.55919291 },
	}};
	vec3 v;
	geodetic g;
	mat3 rot;

	g = ECEF_to_geodetic(ecef_ref);
	if(!geodetic_similar(geodetic_ref, g))
	{
		printf("ECEF_to_geodetic returned <lat %f, long %f, alt %f>, expected <lat %f, long %f, alt %f>\n",
		       g.latitude, g.longitude, g.altitude,
		       geodetic_ref.latitude, geodetic_ref.longitude, geodetic_ref.altitude);
		++fail;
	}

	v = geodetic_to_ECEF(geodetic_ref);
	if(!vec3_similar(ecef_ref, v))
	{
		printf("geodetic_to_ECEF returned <%f,%f,%f>, expected <%f,%f,%f>\n",
		       v.x, v.y, v.z,
		       ecef_ref.x, ecef_ref.y, ecef_ref.z);
		++fail;
	}

	rot = make_LTP_rotation(geodetic_ref);
	if(!mat3_similar(rotation_ref, rot))
	{
		printf("make_LTP_rotation rotation matrix returned was\n");
		mat3_show(rot);
		printf("but expected\n");
		mat3_show(rotation_ref);
		++fail;
	}

	exit(fail);
}
