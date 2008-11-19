#include <math.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "coord.h"
#include "vec.h"
#include "interface.h"
#include "physics.h"
#include "pressure_sensor.h"
#include "sensors.h"

static const microseconds DELTA_T = 1000;
#define DELTA_T_SECONDS (DELTA_T / 1000000.0)

static const microseconds LAUNCH_TIME = 1000000; /* One-second countdown */
static const geodetic initial_geodetic = {
	.latitude = M_PI_2,
	.longitude = 0,
	.altitude = 0,
};

static bool trace, trace_physics, trace_ltp;
static microseconds t;
static enum state fc_state;
static bool engine_ignited;
static microseconds engine_ignition_time;

/* State of the simulated rocket. */
static struct rocket_state rocket_state;

static ATTR_FORMAT(printf,1,2) void trace_printf(const char *fmt, ...)
{
	va_list args;
	if(trace)
	{
		va_start(args, fmt);
		printf("%9.3f: ", t / 1e6);
		vprintf(fmt, args);
		va_end(args);
	}
}

void trace_state(const char *source, struct rocket_state *state, const char *fmt, ...)
{
	va_list args;
	if(trace_physics)
	{
		va_start(args, fmt);
		printf("%9.3f: %s %8.2f alt, %8.2f vel, %8.2f acc, %c%c%c",
		       t / 1e6, source,
		       ECEF_to_geodetic(state->pos).altitude,
		       vec_abs(state->vel), vec_abs(state->acc),
		       state->engine_burning        ? 'B' : '-',
		       state->drogue_chute_deployed ? 'D' : '-',
		       state->main_chute_deployed   ? 'M' : '-');
		vprintf(fmt, args);
		va_end(args);
	}

	if(trace_ltp && !strcmp(source, "sim"))
	{
	        vec3 ltp = ECEF_to_LTP(geodetic_to_ECEF(initial_geodetic), make_LTP_rotation(initial_geodetic), state->pos);
		printf("%f,%f,%f\n", ltp.x, ltp.z, ltp.y);
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

void enqueue_error(const char *msg)
{
	trace_printf("Error message from rocket: %s\n", msg);
}

static unsigned quantize(double value, unsigned mask)
{
	long int rounded = lround(value);
	if(rounded < 0)
		return 0;
	if(rounded > (signed)mask)
		return mask;
	return rounded;
}

static accelerometer_i quantize_accelerometer(accelerometer_d value, unsigned mask)
{
	return (accelerometer_i) {
		.x = quantize(value.x, mask),
		.y = quantize(value.y, mask),
		.z = quantize(value.z, mask),
		.q = quantize(value.q, mask),
	};
}

static void update_simulator(void)
{
	trace_state("sim", &rocket_state, "\n");
	accelerometer_sensor(quantize_accelerometer(accelerometer_measurement(&rocket_state), 0xfff));
	if(t % (DELTA_T * 100) == 0)
		pressure_sensor(quantize(pressure_measurement(&rocket_state), 0xfff));
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
	if(fc_state == STATE_PREFLIGHT)
	{
		trace_printf("Sending arm signal\n");
		arm();
	}
}

static void init_rocket_state(struct rocket_state *rocket_state)
{
	/* TODO: accept an initial orientation for leaving the tower */
	rocket_state->mass = ROCKET_EMPTY_MASS + FUEL_MASS;
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
		else if(!strcmp(argv[i], "--trace-ltp"))
			trace_ltp = true;
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
