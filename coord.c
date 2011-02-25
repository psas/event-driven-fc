/* Copyright © 2010 Portland State Aerospace Society
 * See version control history for detailed authorship information.
 *
 * This program is licensed under the GPL version 2 or later.  Please see the
 * file COPYING in the source distribution of this software for license terms.
 */
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
vec3 geodetic_to_ECEF(geodetic geodetic)
{
	double Nlat = N(geodetic.latitude);
	double coslat = cos(geodetic.latitude);
	double sinlat = sin(geodetic.latitude);
	double coslong = cos(geodetic.longitude);
	double sinlong = sin(geodetic.longitude);
	return (vec3) {
		.x = (geodetic.altitude + Nlat) * coslat * coslong,
		.y = (geodetic.altitude + Nlat) * coslat * sinlong,
		.z = (geodetic.altitude + (1 - WGS84_ESQ) * Nlat) * sinlat,
	};
}

mat3 make_LTP_rotation(geodetic geodetic)
{
	double sinlong = sin(geodetic.longitude);
	double coslong = cos(geodetic.longitude);
	double sinlat = sin(geodetic.latitude);
	double coslat = cos(geodetic.latitude);
	return (mat3){ .component = {
		{          -sinlong,           coslong,      0 },
		{ -coslong * sinlat, -sinlat * sinlong, coslat },
		{  coslat * coslong,  coslat * sinlong, sinlat },
	}};
}

vec3 ECEF_to_LTP(vec3 origin, mat3 rotation, vec3 ecef)
{
	return mat3_vec3_mul(rotation, vec_sub(ecef, origin));
}

vec3 LTP_to_ECEF(vec3 origin, mat3 rotation, vec3 ltp)
{
	return vec_add(mat3_vec3_mul(mat3_transpose(rotation), ltp), origin);
}

/* from http://www.colorado.edu/geography/gcraft/notes/datum/gif/xyzllh.gif
  According to the comment there:
  "This conversion is not exact and provides centimeter accuracy for
  heights < 1,000 km" */
geodetic ECEF_to_geodetic(vec3 ecef)
{
	const double EDOTSQ = (WGS84_A * WGS84_A - WGS84_B * WGS84_B) /
		(WGS84_B * WGS84_B);

	double p = sqrt(ecef.x * ecef.x + ecef.y * ecef.y);
	/* Handle discontinuity at the North and South poles. */
	if(fabs(p) < 1e-30)
		return (geodetic) {
			.latitude = copysign(M_PI_2, ecef.z),
			.longitude = 0,
			.altitude = fabs(ecef.z) - WGS84_B,
		};
	double theta = atan((ecef.z * WGS84_A) / (p * WGS84_B));

	double st = sin(theta);
	double ct = cos(theta);
	geodetic geodetic;
	geodetic.latitude = atan(
		(ecef.z + EDOTSQ * WGS84_B * st * st * st) /
		(p   - WGS84_ESQ * WGS84_A * ct * ct * ct));
	geodetic.longitude = atan2(ecef.y, ecef.x);
	double sinlat = sin(geodetic.latitude);
	double Nlat = N(geodetic.latitude);
	/* Altitude computation from the MathWorks documentation for the
	 * Aerospace Blockset: "ECEF Position to LLA" */
	geodetic.altitude = p * cos(geodetic.latitude) + (ecef.z + WGS84_ESQ * Nlat * sinlat) * sinlat - Nlat;
	return geodetic;
}
