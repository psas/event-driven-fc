/* Copyright © 2010 Portland State Aerospace Society
 * See version control history for detailed authorship information.
 *
 * This program is licensed under the GPL version 2 or later.  Please see the
 * file COPYING in the source distribution of this software for license terms.
 */
#ifndef SPHERICAL_HARMONICS_H_INCLUDED
#define SPHERICAL_HARMONICS_H_INCLUDED

#include <math.h>
#include "coord.h"
#include "mat.h"
#include "vec.h"


/*struct spherical_harmonic_coefficient
{
	double g;
	double h;
};*/

vec3 magnetic_field(geodetic position) ATTR_WARN_UNUSED_RESULT;
//vec3 spherical_harmonic_expansion_mag (const struct spherical_harmonic_coefficient coef[][MAX_DEGREE], const geodetic coord, const int order) ATTR_WARN_UNUSED_RESULT;
//vec3 spherical_harmonic_expansion_grav(const struct spherical_harmonic_coefficient **coef, geodetic coord, int degree);
#endif // SPHERICAL_HARMONICS_H_INCLUDED

