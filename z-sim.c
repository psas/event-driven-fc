#include <math.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
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

/* State of the simulated rocket. */
static enum state rocket_state;
static microseconds t;
static vec3 pos, vel, acc, rotpos, rotvel;
static double pressure;
static bool engine_ignited;
static microseconds engine_on;
static bool drogue_chute_deployed;
static bool main_chute_deployed;

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
	if(rocket_state != state)
	{
		trace_printf("State changed from %d to %d.\n", rocket_state, state);
		rocket_state = state;
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
			engine_on = ENGINE_BURN_TIME;
		}
		else
			trace_printf("Rocket trying to reignite engine.\n");
	}
}

void drogue_chute(bool go)
{
	if(go)
	{
		if(!drogue_chute_deployed)
		{
			trace_printf("Drogue chute deployed\n");
			drogue_chute_deployed = true;
		}
		else
			trace_printf("Rocket trying to redeploy drogue chute.\n");
	}
}

void main_chute(bool go)
{
	if(go)
	{
		if(!main_chute_deployed)
		{
			trace_printf("Main chute deployed\n");
			main_chute_deployed = true;
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

static void update_rocket_state(void)
{
	int i;
	double force[3] = { 0.0, 0.0, 0.0 };
	double mass = ROCKET_EMPTY_MASS
	            + FUEL_MASS * (engine_ignited ? engine_on/ENGINE_BURN_TIME : 1);
	double drag_coefficient, cross_section;
	t += DELTA_T;

	if(engine_on)
	{
	        force[Z] += ENGINE_THRUST;
		engine_on -= DELTA_T;
		if(engine_on <= 0)
		{
			trace_printf("Engine burn-out.\n");
			engine_on = 0;
		}
	}
	else if(pos[Z] <= 0.0)
	{
		acc[Z] = 0.0;
		vel[Z] = 0.0;
		pos[Z] = 0.0;
	}
	else
		force[Z] += -mass * EARTH_GRAVITY;

        if(main_chute_deployed)
	{
		drag_coefficient = MAIN_CHUTE_DRAG_COEFFICIENT;
		cross_section = MAIN_CHUTE_CROSS_SECTION;
	}
	else if(drogue_chute_deployed)
	{
		drag_coefficient = DROGUE_CHUTE_DRAG_COEFFICIENT;
		cross_section = DROGUE_CHUTE_CROSS_SECTION;
	}
	else
	{
		drag_coefficient = ROCKET_DRAG_COEFFICIENT;
		cross_section = ROCKET_CROSS_SECTION;
	}
	force[Z] += -sign(vel[Z]) * 0.5 * AIR_DENSITY * vel[Z] * vel[Z] * cross_section * drag_coefficient;

	/* FIXME: this should use a better numerical integration technique,
	 * such as Runge-Kutta or leapfrog integration. */
	for(i = 0; i < 3; ++i)
	{
		pos[i] += vel[i] * DELTA_T_SECONDS;
		vel[i] += acc[i] * DELTA_T_SECONDS;
		acc[i] = force[i] / mass;
		rotpos[i] += rotvel[i] * DELTA_T_SECONDS;
	}

	pressure = altitude_to_pressure(pos[Z]);
}

static void call_rocket_functions(void)
{
	if(trace_physics)
		trace_printf("Rocket Z pos, vel, acc: %f %f %f\n", pos[Z], vel[Z], acc[Z]);
	omniscience_9000(t, pos, vel, acc, rotpos, rotvel);
	z_accelerometer(acc[Z]);
	pressure_sensor(pressure);
	if(!engine_ignited && t >= LAUNCH_TIME)
	{
		trace_printf("Sending launch signal\n");
		launch();
	}
	if(rocket_state == STATE_PREFLIGHT && t >= ARM_TIME)
	{
		trace_printf("Sending arm signal\n");
		arm();
	}
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

	while(rocket_state != STATE_RECOVERY)
	{
		update_rocket_state();
		call_rocket_functions();
	}
	return 0;
}
