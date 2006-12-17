#include <stdbool.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "fc.h"

static const microseconds DELTA_T = 1000;
#define DELTA_T_SECONDS (DELTA_T / 1000000.0)

static const microseconds ARM_TIME = 0;
static const microseconds LAUNCH_TIME = 10000000; /* Traditional ten-second countdown */
static const microseconds ENGINE_BURN_TIME = 10000000;
static const double ENGINE_THRUST = 11.196314;

static bool trace, trace_physics;

/* State of the simulated rocket. */
static enum state rocket_state;
static microseconds t;
static vec3 pos, vel, acc, rotpos, rotvel;
static bool engine_ignited;
static microseconds engine_on;
static bool drogue_chute_deployed;
static bool main_chute_deployed;

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

static void update_rocket_state(void)
{
	int i;
	t += DELTA_T;
	for(i = 0; i < 3; ++i)
	{
		pos[i] += vel[i] * DELTA_T_SECONDS;
		vel[i] += acc[i] * DELTA_T_SECONDS;
		rotpos[i] += rotvel[i] * DELTA_T_SECONDS;
	}
	if(engine_on)
	{
		acc[Z] = ENGINE_THRUST;
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
	else if(main_chute_deployed)
	{
		acc[Z] = 0.0;
		vel[Z] = -10.0;
	}
	else if(drogue_chute_deployed)
	{
		acc[Z] = 0.0;
		vel[Z] = -30.0;
	}
	else
		acc[Z] = -9.8;
}

static void call_rocket_functions(void)
{
	if(trace_physics)
		trace_printf("Rocket Z pos, vel, acc: %f %f %f\n", pos[Z], vel[Z], acc[Z]);
	omniscience_9000(t, pos, vel, acc, rotpos, rotvel);
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
