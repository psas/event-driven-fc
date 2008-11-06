#include <math.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "coord.h"
#include "vec.h"
#include "fc.h"
#include "interface.h"
#include "physics.h"
#include "pressure_sensor.h"
#include "sensors.h"

static const microseconds DELTA_T = 1000;
#define DELTA_T_SECONDS (DELTA_T / 1000000.0)

static const microseconds ARM_TIME = 0;
static const microseconds LAUNCH_TIME = 1000000; /* One-second countdown */

static bool trace, trace_physics;
static microseconds t;
static enum state fc_state;
static bool engine_ignited;
static microseconds engine_ignition_time;

/* State of the simulated rocket. */
static struct rocket_state rocket_state;

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

static void update_simulator(void)
{
	if(trace_physics)
		trace_printf("Rocket Z pos, vel, acc: %f %f %f\n",
				rocket_state.pos.z, rocket_state.vel.z, rocket_state.acc.z);
	accelerometer_sensor(accelerometer_measurement(&rocket_state));
	if(t % (DELTA_T * 100) == 0)
		pressure_sensor(pressure_measurement(&rocket_state));
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
	/* TODO: accept an initial orientation for leaving the tower */
	rocket_state->mass = ROCKET_EMPTY_MASS + FUEL_MASS;
	const geodetic initial_geodetic = {
		.latitude = M_PI_2,
		.longitude = 0,
		.altitude = 0,
	};
	rocket_state->pos = geodetic_to_ECEF(initial_geodetic);
	rocket_state->rotpos = make_LTP_rotation(initial_geodetic);
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
	init();

	while(fc_state != STATE_RECOVERY)
	{
		t += DELTA_T;
		update_rocket_state(&rocket_state, DELTA_T_SECONDS);
		update_simulator();
		tick(DELTA_T_SECONDS);
	}
	return 0;
}
