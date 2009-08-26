#include "physics.h"
#include "coord.h"

vec3 ECEF_to_rocket(struct rocket_state *rocket_state, vec3 v)
{
	return mat3_vec3_mul(rocket_state->rotpos, v);
}

vec3 rocket_to_ECEF(struct rocket_state *rocket_state, vec3 v)
{
	return mat3_vec3_mul(mat3_transpose(rocket_state->rotpos), v);
}

vec3 gravity_acceleration(struct rocket_state *rocket_state)
{
	/* TODO: apply gravity at the approximate center of mass */
	return vec_scale(rocket_state->pos, -EARTH_GRAVITY / vec_abs(rocket_state->pos));
}

void update_rocket_state(struct rocket_state *rocket_state, double delta_t)
{
	/* FIXME: this should use a better numerical integration technique,
	 * such as Runge-Kutta or leapfrog integration. */
	rocket_state->pos = vec_add(rocket_state->pos, vec_scale(rocket_state->vel, delta_t));
	rocket_state->vel = vec_add(rocket_state->vel, vec_scale(rocket_state->acc, delta_t));
	rocket_state->rotpos = mat3_mul(rocket_state->rotpos, axis_angle_to_mat3(vec_scale(rocket_state->rotvel, delta_t)));
}
