#include "coord.h"
#include "mat.h"
#include "physics.h"
#include "pressure_sensor.h"
#include "sensors.h"
#include "vec.h"

#define G 9.80665

/* TODO: move sensor bias/gain to rocket_state so BPF can estimate them */

vec3 accelerometer_measurement(struct rocket_state *state)
{
	const vec3 bias = {{ 2400.45  , 2462.06  , 1918.72   }};
	const vec3 gain = {{  392.80/G,  386.90/G,   77.00/G }};
	vec3 gravity_acc = vec_scale(gravity_force(state), 1/state->mass);
	vec3 ecef = vec_sub(state->acc, gravity_acc);
	vec3 rocket = mat3_vec3_mul(state->rotpos, ecef);
	/* TODO: return a four-vector with "Q" accelerometer */
	return (vec3) {{
		.x = rocket.x * gain.x + bias.x,
		.y = rocket.y * gain.y + bias.y,
		.z = rocket.z * gain.z + bias.z,
	}};
}

double pressure_measurement(struct rocket_state *state)
{
	const double bias = -470.734;
	const double gain = 44.549779924087175 / 1000;
	double rocket = altitude_to_pressure(ECEF_to_geodetic(state->pos).altitude);
	return rocket * gain + bias;
}
