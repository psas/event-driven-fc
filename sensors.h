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
