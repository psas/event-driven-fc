#include "physics.h"
#include "coord.h"

/* Drag constants */
static const double MAIN_CHUTE_DRAG_COEFFICIENT = 0.8;
static const double MAIN_CHUTE_CROSS_SECTION = 7.429812032713523;
static const double DROGUE_CHUTE_DRAG_COEFFICIENT = 0.8;
static const double DROGUE_CHUTE_CROSS_SECTION = 0.836954282802814;
static const double ROCKET_DRAG_COEFFICIENT = 0.36559;
static const double ROCKET_CROSS_SECTION = 0.015327901242699;
static const double AIR_DENSITY = 1.225;

vec3 gravity_force(struct rocket_state *rocket_state)
{
	/* TODO: apply gravity at the approximate center of mass */
	return vec_scale(rocket_state->pos, -EARTH_GRAVITY * rocket_state->mass / vec_abs(rocket_state->pos));
}

static vec3 drag_force(struct rocket_state *rocket_state)
{
	/* TODO: fix drag for rocket orientation */
	double drag_coefficient, cross_section;
        if(rocket_state->main_chute_deployed)
	{
		drag_coefficient = MAIN_CHUTE_DRAG_COEFFICIENT;
		cross_section = MAIN_CHUTE_CROSS_SECTION;
	}
	else if(rocket_state->drogue_chute_deployed)
	{
		drag_coefficient = DROGUE_CHUTE_DRAG_COEFFICIENT;
		cross_section = DROGUE_CHUTE_CROSS_SECTION;
	}
	else
	{
		drag_coefficient = ROCKET_DRAG_COEFFICIENT;
		cross_section = ROCKET_CROSS_SECTION;
	}
	return vec_scale(rocket_state->vel, -0.5 * AIR_DENSITY
	                 * vec_abs(rocket_state->vel)
	                 * cross_section * drag_coefficient);
}

static vec3 thrust_force(struct rocket_state *rocket_state)
{
	if(!rocket_state->engine_burning)
	        return (vec3){{ 0, 0, 0 }};
	return mat3_vec3_mul(mat3_transpose(rocket_state->rotpos), (vec3){{ 0, 0, ENGINE_THRUST }});
}

void update_rocket_state(struct rocket_state *rocket_state, double delta_t)
{
	vec3 force;

	if(rocket_state->engine_burning)
		rocket_state->mass -= FUEL_MASS * delta_t / ENGINE_BURN_TIME;

	/* TODO: add coefficient of normal force at the center of pressure */
	force = thrust_force(rocket_state);
	force = vec_add(force, gravity_force(rocket_state));
	force = vec_add(force, drag_force(rocket_state));

	/* FIXME: this should use a better numerical integration technique,
	 * such as Runge-Kutta or leapfrog integration. */
	rocket_state->pos = vec_add(rocket_state->pos, vec_scale(rocket_state->vel, delta_t));
	rocket_state->vel = vec_add(rocket_state->vel, vec_scale(rocket_state->acc, delta_t));
	rocket_state->acc = vec_scale(force, 1/rocket_state->mass);
	rocket_state->rotpos = mat3_mul(rocket_state->rotpos, axis_angle_to_mat3(vec_scale(rocket_state->rotvel, delta_t)));

	/* Implement the ground: When you find yourself in a hole, stop
	 * digging. */
	if(ECEF_to_geodetic(rocket_state->pos).altitude < 0 &&
	   (vec_dot(rocket_state->pos, rocket_state->vel) < 0 ||
	    vec_dot(rocket_state->pos, rocket_state->acc) < 0))
		rocket_state->acc = rocket_state->vel = (vec3) {{ 0, 0, 0 }};
}
