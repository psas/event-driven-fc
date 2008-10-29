#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include "coord.h"

static bool vec3_similar(const vec3 expected, const vec3 actual)
{
	int i;
	for(i = 0; i < 3; ++i)
		if(fabs(expected[i] - actual[i]) > 0.000001)
			return false;
	return true;
}

static bool mat3_similar(const mat3 expected, const mat3 actual)
{
	int i;
	for(i = 0; i < 3; ++i)
		if(!vec3_similar(expected.component[i], actual.component[i]))
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
	const vec3 geodetic_ref = {
		[LATITUDE] = 0.59341195,
		[LONGITUDE] = -2.0478571,
		[ALTITUDE] = 251.702,
	};
	const vec3 ecef_ref = {
		[X] = -2430601.795708,
		[Y] = -4702442.736094,
		[Z] =  3546587.336483,
	};
	const mat3 rotation_ref = { .component = {
		{  0.88834836, -0.45917011,  0.00000000 },
		{  0.25676467,  0.49675810,  0.82903757 },
		{ -0.38066927, -0.73647416,  0.55919291 },
	}};
	vec3 tmp;
	mat3 rot;

	ECEF_to_geodetic(tmp, ecef_ref);
	if(!vec3_similar(geodetic_ref, tmp))
	{
		printf("ECEF_to_geodetic returned <lat %f, long %f, alt %f>, expected <lat %f, long %f, alt %f>\n",
		       tmp[X], tmp[Y], tmp[Z],
		       geodetic_ref[X], geodetic_ref[Y], geodetic_ref[Z]);
		++fail;
	}

	geodetic_to_ECEF(tmp, geodetic_ref);
	if(!vec3_similar(ecef_ref, tmp))
	{
		printf("geodetic_to_ECEF returned <%f,%f,%f>, expected <%f,%f,%f>\n",
		       tmp[X], tmp[Y], tmp[Z],
		       ecef_ref[X], ecef_ref[Y], ecef_ref[Z]);
		++fail;
	}

	make_origin(tmp, &rot, geodetic_ref);
	if(!mat3_similar(rotation_ref, rot))
	{
		printf("make_origin rotation matrix returned was\n");
		mat3_show(rot);
		printf("but expected\n");
		mat3_show(rotation_ref);
		++fail;
	}
	if(!vec3_similar(ecef_ref, tmp))
	{
		printf("make_origin origin returned was <%f,%f,%f>, expected <%f,%f,%f>\n",
		       tmp[X], tmp[Y], tmp[Z],
		       ecef_ref[X], ecef_ref[Y], ecef_ref[Z]);
		++fail;
	}

	exit(fail);
}
