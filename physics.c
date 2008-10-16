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

static void gravity_force(struct rocket_state *rocket_state, vec3 force)
{
	/* TODO: apply gravity at the approximate center of mass */
	force[X] = force[Y] = 0;
	force[Z] = -rocket_state->mass * EARTH_GRAVITY;
}

static void drag_force(struct rocket_state *rocket_state, vec3 force)
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
	force[X] = force[Y] = 0;
	force[Z] = -sign(rocket_state->vel[Z]) * 0.5 * AIR_DENSITY
		* rocket_state->vel[Z] * rocket_state->vel[Z]
		* cross_section * drag_coefficient;
}

static void thrust_force(struct rocket_state *rocket_state, vec3 force)
{
	force[X] = force[Y] = 0;
	if(rocket_state->engine_burning)
	        force[Z] = ENGINE_THRUST;
	else
		force[Z] = 0;
}

void update_rocket_state(struct rocket_state *rocket_state, double delta_t)
{
	int i;
	vec3 force = { 0.0, 0.0, 0.0 };
	vec3 tmp;

	if(rocket_state->engine_burning)
		rocket_state->mass -= FUEL_MASS * delta_t / ENGINE_BURN_TIME;

	/* TODO: add coefficient of normal force at the center of pressure */
	if(rocket_state->engine_burning || rocket_state->pos[Z] > 0.0)
	{
		gravity_force(rocket_state, tmp);
		vec_add(force, tmp);
		drag_force(rocket_state, tmp);
		vec_add(force, tmp);
	}
	else
	{
		rocket_state->pos[Z] = 0.0;
		rocket_state->vel[Z] = 0.0;
	}
	thrust_force(rocket_state, tmp);
	vec_add(force, tmp);

	/* FIXME: this should use a better numerical integration technique,
	 * such as Runge-Kutta or leapfrog integration. */
	vec3 rotvel;
	for(i = 0; i < 3; ++i)
	{
		rocket_state->pos[i] += rocket_state->vel[i] * delta_t;
		rocket_state->vel[i] += rocket_state->acc[i] * delta_t;
		rocket_state->acc[i] = force[i] / rocket_state->mass;
		rotvel[i] = rocket_state->rotvel[i] * delta_t;
	}
	mat3 rotdelta;
	axis_angle_to_mat3(&rotdelta, rotvel);
	mat3_mul(&rocket_state->rotpos, &rocket_state->rotpos, &rotdelta);
}
