#include <math.h>

#include "coord.h"
#include "mat.h"
#include "physics.h"
#include "pressure_sensor.h"
#include "sensors.h"
#include "vec.h"

#define G 9.80665

/* TODO: move sensor bias/gain to rocket_state so BPF can estimate them */

accelerometer_d accelerometer_measurement(struct rocket_state *state)
{
	const accelerometer_d bias = { 2400.45  , 2462.06  , 1918.72  , 1907.53   };
	const accelerometer_d gain = {  392.80/G,  386.90/G,   77.00/G,   75.40/G };
	vec3 gravity_acc = vec_scale(gravity_force(state), 1/state->mass);
	vec3 ecef = vec_sub(state->acc, gravity_acc);
	vec3 rocket = ECEF_to_rocket(state, ecef);
	return (accelerometer_d) {
		.x = rocket.x * gain.x + bias.x,
		.y = rocket.y * gain.y + bias.y,
		.z = rocket.z * gain.z + bias.z,
		.q = (rocket.x + rocket.y) * M_SQRT1_2 * gain.q + bias.q,
	};
}

double pressure_measurement(struct rocket_state *state)
{
	const double bias = -470.734;
	const double gain = 44.549779924087175 / 1000;
	double rocket = altitude_to_pressure(ECEF_to_geodetic(state->pos).altitude);
	return rocket * gain + bias;
}
