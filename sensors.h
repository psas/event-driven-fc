/* Copyright © 2010 Portland State Aerospace Society
 * See version control history for detailed authorship information.
 *
 * This program is licensed under the GPL version 2 or later.  Please see the
 * file COPYING in the source distribution of this software for license terms.
 */
#ifndef SENSORS_H
#define SENSORS_H

#include "compiler.h"
#include "physics.h"

typedef struct accelerometer_d {
	double x, y, z, q;
} accelerometer_d;

accelerometer_d accelerometer_measurement(struct rocket_state *state) ATTR_WARN_UNUSED_RESULT;
vec3 gyroscope_measurement(struct rocket_state *state);
double pressure_measurement(struct rocket_state *state) ATTR_WARN_UNUSED_RESULT;
vec3 magnetometer_measurement(struct rocket_state *state);

#endif /* SENSORS_H */
