#ifndef SENSORS_H
#define SENSORS_H

#include "physics.h"
#include "vec.h"

vec3 accelerometer_measurement(struct rocket_state *state);
double pressure_measurement(struct rocket_state *state);

#endif /* SENSORS_H */
