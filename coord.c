#include <math.h>
#include "coord.h"
#include "mat.h"
#include "vec.h"

/* WGS-84 parameters: */
static const double WGS84_A = 6378137.0;    /* semi-major axis (meters) */
static const double WGS84_B = 6356752.3142; /* semi-minor axis (meters) */

/* ellipsoid flatness */
#define WGS84_FLATNESS ((WGS84_A - WGS84_B) / WGS84_A)

/* eccentricity (squared) */
#define WGS84_ESQ (WGS84_FLATNESS * (2 - WGS84_FLATNESS))

/* distance from surface to z-axis along ellipsoid normal (plumb line) */
static double N(double latitude)
{
	double sinlat = sin(latitude);
	return WGS84_A / sqrt(1 - WGS84_ESQ * sinlat * sinlat);
}

/* from http://psas.pdx.edu/CoordinateSystem/Latitude_to_LocalTangent.pdf */
void geodetic_to_ECEF(vec3 ecef, const vec3 geodetic)
{
	double Nlat = N(geodetic[LATITUDE]);
	double coslat = cos(geodetic[LATITUDE]);
	double sinlat = sin(geodetic[LATITUDE]);
	double coslong = cos(geodetic[LONGITUDE]);
	double sinlong = sin(geodetic[LONGITUDE]);
	ecef[X] = (geodetic[ALTITUDE] + Nlat) * coslat * coslong;
	ecef[Y] = (geodetic[ALTITUDE] + Nlat) * coslat * sinlong;
	ecef[Z] = (geodetic[ALTITUDE] + (1 - WGS84_ESQ) * Nlat) * sinlat;
}

void make_origin(vec3 origin, mat3 *rotation, const vec3 geodetic)
{
	geodetic_to_ECEF(origin, geodetic);
	double sinlong = sin(geodetic[LONGITUDE]);
	double coslong = cos(geodetic[LONGITUDE]);
	double sinlat = sin(geodetic[LATITUDE]);
	double coslat = cos(geodetic[LATITUDE]);
	*rotation = (mat3) { .component = {
		{          -sinlong,           coslong,      0 },
		{ -coslong * sinlat, -sinlat * sinlong, coslat },
		{  coslat * coslong,  coslat * sinlong, sinlat },
	}};
}

void ECEF_to_tangent_plane(vec3 ltp, const vec3 origin, const mat3 *rotation, const vec3 ecef)
{
	int i;
	vec3 tmp;
	for(i = 0; i < 3; ++i)
		tmp[i] = ecef[i] - origin[i];
	mat3_vec3_mul(ltp, rotation, tmp);
}

void tangent_plane_to_ECEF(vec3 ecef, const vec3 origin, const mat3 *rotation, const vec3 ltp)
{
	mat3 tmp;
	int i, j;
	for(i = 0; i < 3; ++i)
		for(j = 0; j < 3; ++j)
			tmp.component[j][i] = rotation->component[i][j];
	mat3_vec3_mul(ecef, &tmp, ltp);
	for(i = 0; i < 3; ++i)
		ecef[i] += origin[i];
}

/* from http://www.colorado.edu/geography/gcraft/notes/datum/gif/xyzllh.gif
  According to the comment there:
  "This conversion is not exact and provides centimeter accuracy for
  heights < 1,000 km" */
void ECEF_to_geodetic(vec3 geodetic, const vec3 ecef)
{
	const double EDOTSQ = (WGS84_A * WGS84_A - WGS84_B * WGS84_B) /
		(WGS84_B * WGS84_B);

	double p = sqrt(ecef[X] * ecef[X] + ecef[Y] * ecef[Y]);
	double theta = atan((ecef[Z] * WGS84_A) / (p * WGS84_B));

	double st = sin(theta);
	double ct = cos(theta);
	geodetic[LATITUDE] = atan(
		(ecef[Z] + EDOTSQ * WGS84_B * st * st * st) /
		(p    - WGS84_ESQ * WGS84_A * ct * ct * ct));
	geodetic[LONGITUDE] = atan2(ecef[Y], ecef[X]);
	geodetic[ALTITUDE] = p / cos(geodetic[LATITUDE]) - N(geodetic[LATITUDE]);
}
