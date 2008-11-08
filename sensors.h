#ifndef SENSORS_H
#define SENSORS_H

#include "compiler.h"
#include "physics.h"
#include "vec.h"

vec3 accelerometer_measurement(struct rocket_state *state) ATTR_WARN_UNUSED_RESULT;
double pressure_measurement(struct rocket_state *state) ATTR_WARN_UNUSED_RESULT;

#endif /* SENSORS_H */
