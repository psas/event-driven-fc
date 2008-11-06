#include "coord.h"
#include "mat.h"
#include "physics.h"
#include "pressure_sensor.h"
#include "sensors.h"
#include "vec.h"

vec3 accelerometer_measurement(struct rocket_state *state)
{
	vec3 gravity_acc = vec_scale(gravity_force(state), 1/state->mass);
	vec3 ecef = vec_sub(state->acc, gravity_acc);
	return mat3_vec3_mul(state->rotpos, ecef);
}

double pressure_measurement(struct rocket_state *state)
{
	return altitude_to_pressure(ECEF_to_geodetic(state->pos).altitude);
}
