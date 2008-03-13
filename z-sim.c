#include <math.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "vec.h"
#include "fc.h"
#include "interface.h"

static const microseconds DELTA_T = 1000;
#define DELTA_T_SECONDS (DELTA_T / 1000000.0)

static const microseconds ARM_TIME = 0;
static const microseconds LAUNCH_TIME = 1000000; /* One-second countdown */
static const double ROCKET_EMPTY_MASS = 21.54;
static const double FUEL_MASS = 5.9;
static const microseconds ENGINE_BURN_TIME = 4300000;
static const double ENGINE_THRUST = 3094.65;
static const double EARTH_GRAVITY = 9.8;
static const double GAS_CONSTANT = 287.053; /* J / (kg * K) */
static const double BASE_PRESSURE = 100000; /* Pa */
static const double BASE_TEMP = 273.15; /* K */
static const double TEMP_LAPSE_RATE = -0.0065; /* K/m */

static const double ROCKET_DRAG_COEFFICIENT = 0.36559;
static const double DROGUE_CHUTE_DRAG_COEFFICIENT = 0.8;
static const double MAIN_CHUTE_DRAG_COEFFICIENT = 0.8;
static const double ROCKET_CROSS_SECTION = 0.015327901242699;
static const double DROGUE_CHUTE_CROSS_SECTION = 0.836954282802814;
static const double MAIN_CHUTE_CROSS_SECTION = 7.429812032713523;
static const double AIR_DENSITY = 1.225;

static bool trace, trace_physics;
static microseconds t;
static enum state fc_state;
static bool engine_ignited;
static microseconds engine_ignition_time;

/* State of the simulated rocket. */
static struct rocket_state rocket_state;

static double sign(double x)
{
	if(x < 0)
		return -1;
	if(x > 0)
		return 1;
	return 0;
}

static void trace_printf(char *fmt, ...)
{
	va_list args;
	if(trace)
	{
		va_start(args, fmt);
		printf("%0.3f: ", t / 1e6);
		vprintf(fmt, args);
		va_end(args);
	}
}

void report_state(enum state state)
{
	if(fc_state != state)
	{
		trace_printf("State changed from %d to %d.\n", fc_state, state);
		fc_state = state;
	}
}

/* FIXME: these functions should work more like they will with USB: set a flag,
 * and process it when handling an output frame. */
void ignite(bool go)
{
	if(go)
	{
		if(!engine_ignited)
		{
			trace_printf("Engine ignition\n");
			engine_ignited = true;
			rocket_state.engine_burning = true;
			engine_ignition_time = t;
		}
		else
			trace_printf("Rocket trying to reignite engine.\n");
	}
}

void drogue_chute(bool go)
{
	if(go)
	{
		if(!rocket_state.drogue_chute_deployed)
		{
			trace_printf("Drogue chute deployed\n");
			rocket_state.drogue_chute_deployed = true;
		}
		else
			trace_printf("Rocket trying to redeploy drogue chute.\n");
	}
}

void main_chute(bool go)
{
	if(go)
	{
		if(!rocket_state.main_chute_deployed)
		{
			trace_printf("Main chute deployed\n");
			rocket_state.main_chute_deployed = true;
		}
		else
			trace_printf("Rocket trying to redeploy main chute.\n");
	}
}

void enqueue_error(char *msg)
{
	trace_printf("Error message from rocket: %s\n", msg);
}

static double altitude_to_pressure(double z_pos)
{
	return BASE_PRESSURE
	     * pow(1 + z_pos * (TEMP_LAPSE_RATE/BASE_TEMP),
	           -EARTH_GRAVITY / (TEMP_LAPSE_RATE * GAS_CONSTANT));
}

static void gravity_force(struct rocket_state *rocket_state, vec3 force)
{
	force[X] = force[Y] = 0;
	force[Z] = -rocket_state->physics.mass * EARTH_GRAVITY;
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
	force[Z] = -sign(rocket_state->physics.vel[Z]) * 0.5 * AIR_DENSITY
		* rocket_state->physics.vel[Z] * rocket_state->physics.vel[Z]
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

static void update_rocket_state(struct rocket_state *rocket_state, double delta_t)
{
	int i;
	vec3 force = { 0.0, 0.0, 0.0 };
	vec3 tmp;

	if(rocket_state->engine_burning)
		rocket_state->physics.mass -= FUEL_MASS * delta_t / ENGINE_BURN_TIME;

	if(rocket_state->engine_burning || rocket_state->physics.pos[Z] > 0.0)
	{
		gravity_force(rocket_state, tmp);
		vec_add(force, tmp);
		drag_force(rocket_state, tmp);
		vec_add(force, tmp);
	}
	else
	{
		rocket_state->physics.pos[Z] = 0.0;
		rocket_state->physics.vel[Z] = 0.0;
	}
	thrust_force(rocket_state, tmp);
	vec_add(force, tmp);

	/* FIXME: this should use a better numerical integration technique,
	 * such as Runge-Kutta or leapfrog integration. */
	for(i = 0; i < 3; ++i)
	{
		rocket_state->physics.pos[i] += rocket_state->physics.vel[i] * delta_t;
		rocket_state->physics.vel[i] += rocket_state->physics.acc[i] * delta_t;
		rocket_state->physics.acc[i] = force[i] / rocket_state->physics.mass;
		rocket_state->physics.rotpos[i] += rocket_state->physics.rotvel[i] * delta_t;
	}
}

static void update_simulator(void)
{
	if(trace_physics)
		trace_printf("Rocket Z pos, vel, acc: %f %f %f\n",
				rocket_state.physics.pos[Z], rocket_state.physics.vel[Z], rocket_state.physics.acc[Z]);
	omniscience_9000(rocket_state.physics.pos,
			rocket_state.physics.vel,
			rocket_state.physics.acc,
			rocket_state.physics.rotpos,
			rocket_state.physics.rotvel);
	z_accelerometer(rocket_state.physics.acc[Z]);
	pressure_sensor(altitude_to_pressure(rocket_state.physics.pos[Z]));
	if(!engine_ignited && t >= LAUNCH_TIME)
	{
		trace_printf("Sending launch signal\n");
		launch();
	}
	if(rocket_state.engine_burning
	   && t - engine_ignition_time >= ENGINE_BURN_TIME)
	{
		trace_printf("Engine burn-out.\n");
		rocket_state.engine_burning = false;
	}
	if(fc_state == STATE_PREFLIGHT && t >= ARM_TIME)
	{
		trace_printf("Sending arm signal\n");
		arm();
	}
}

static void init_rocket_state(struct rocket_state *rocket_state)
{
	rocket_state->physics.mass = ROCKET_EMPTY_MASS + FUEL_MASS;
}

int main(int argc, char *argv[])
{
	int i;
	for(i = 1; i < argc; i++)
	{
		if(!strcmp(argv[i], "--trace"))
			trace = true;
		else if(!strcmp(argv[i], "--trace-physics"))
			trace = trace_physics = true;
	}

	init_rocket_state(&rocket_state);

	while(fc_state != STATE_RECOVERY)
	{
		t += DELTA_T;
		update_rocket_state(&rocket_state, DELTA_T_SECONDS);
		update_simulator();
	}
	return 0;
}
