#ifndef PHYSICS_H
#define PHYSICS_H

#include <stdint.h>
#include <stdbool.h>
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
	vec3 pos, vel, acc;
	vec3 rotpos, rotvel, rotacc;
	double mass;
	bool engine_burning;
	bool drogue_chute_deployed;
	bool main_chute_deployed;
};

void update_rocket_state(struct rocket_state *rocket_state, double delta_t);

#endif
