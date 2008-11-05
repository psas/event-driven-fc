#include "physics.h"

/* Drag constants */
static const double MAIN_CHUTE_DRAG_COEFFICIENT = 0.8;
static const double MAIN_CHUTE_CROSS_SECTION = 7.429812032713523;
static const double DROGUE_CHUTE_DRAG_COEFFICIENT = 0.8;
static const double DROGUE_CHUTE_CROSS_SECTION = 0.836954282802814;
static const double ROCKET_DRAG_COEFFICIENT = 0.36559;
static const double ROCKET_CROSS_SECTION = 0.015327901242699;
static const double AIR_DENSITY = 1.225;

static double sign(double x)
{
	if(x < 0)
		return -1;
	if(x > 0)
		return 1;
	return 0;
}

static vec3 gravity_force(struct rocket_state *rocket_state)
{
	/* TODO: apply gravity at the approximate center of mass */
	return (vec3){{ 0, 0, -rocket_state->mass * EARTH_GRAVITY }};
}

static vec3 drag_force(struct rocket_state *rocket_state)
{
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
	return (vec3){{
		.x = 0,
		.y = 0,
		.z = -sign(rocket_state->vel.z) * 0.5 * AIR_DENSITY
		   * rocket_state->vel.z * rocket_state->vel.z
		   * cross_section * drag_coefficient
	}};
}

static vec3 thrust_force(struct rocket_state *rocket_state)
{
	if(rocket_state->engine_burning)
	        return (vec3){{ 0, 0, ENGINE_THRUST }};
	else
	        return (vec3){{ 0, 0, 0 }};
}

void update_rocket_state(struct rocket_state *rocket_state, double delta_t)
{
	int i;
	vec3 force = {{ 0, 0, 0 }};

	if(rocket_state->engine_burning)
		rocket_state->mass -= FUEL_MASS * delta_t / ENGINE_BURN_TIME;

	/* TODO: add coefficient of normal force at the center of pressure */
	if(rocket_state->engine_burning || rocket_state->pos.z > 0.0)
	{
		force = vec_add(force, gravity_force(rocket_state));
		force = vec_add(force, drag_force(rocket_state));
	}
	else
	{
		rocket_state->pos.z = 0.0;
		rocket_state->vel.z = 0.0;
	}
	force = vec_add(force, thrust_force(rocket_state));

	/* FIXME: this should use a better numerical integration technique,
	 * such as Runge-Kutta or leapfrog integration. */
	vec3 rotvel;
	for(i = 0; i < 3; ++i)
	{
		rocket_state->pos.component[i] += rocket_state->vel.component[i] * delta_t;
		rocket_state->vel.component[i] += rocket_state->acc.component[i] * delta_t;
		rocket_state->acc.component[i] = force.component[i] / rocket_state->mass;
		rotvel.component[i] = rocket_state->rotvel.component[i] * delta_t;
	}
	mat3 rotdelta;
	axis_angle_to_mat3(&rotdelta, rotvel);
	mat3_mul(&rocket_state->rotpos, &rocket_state->rotpos, &rotdelta);
}
