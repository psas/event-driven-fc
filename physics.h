#ifndef PHYSICS_H
#define PHYSICS_H

#include <stdint.h>
#include <stdbool.h>
#include "compiler.h"
#include "mat.h"
#include "vec.h"

typedef uint64_t microseconds;

/* Gravity constant */
static const double EARTH_GRAVITY = 9.8;

/* Thrust constants */
static const double ENGINE_THRUST = 3094.65;
static const double ROCKET_EMPTY_MASS = 21.54;
static const double FUEL_MASS = 5.9;
static const microseconds ENGINE_BURN_TIME = 4300000;

struct rocket_state
{
	vec3 pos, vel, acc;  /* Earth-centered Earth-fixed */
	mat3 rotpos;         /* Launch-centered Earth-fixed */
	vec3 rotvel;         /* Launch-centered Earth-fixed */
	double mass;
	bool engine_burning;
	bool drogue_chute_deployed;
	bool main_chute_deployed;
};

vec3 ECEF_to_rocket(struct rocket_state *rocket_state, vec3 v) ATTR_WARN_UNUSED_RESULT;
vec3 rocket_to_ECEF(struct rocket_state *rocket_state, vec3 v) ATTR_WARN_UNUSED_RESULT;
vec3 gravity_acceleration(struct rocket_state *rocket_state) ATTR_WARN_UNUSED_RESULT;
vec3 expected_acceleration(struct rocket_state *rocket_state) ATTR_WARN_UNUSED_RESULT;
void update_rocket_state(struct rocket_state *rocket_state, double delta_t);

#endif
