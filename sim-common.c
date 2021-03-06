/* Copyright © 2010 Portland State Aerospace Society
 * See version control history for detailed authorship information.
 *
 * This program is licensed under the GPL version 2 or later.  Please see the
 * file COPYING in the source distribution of this software for license terms.
 */
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "compiler.h"
#include "coord.h"
#include "interface.h"
#include "sim-common.h"

static bool trace, trace_physics, trace_ltp;
static enum state fc_state;
geodetic initial_geodetic;

void parse_trace_args(int argc, const char *const argv[])
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
}

void trace_printf(const char *fmt, ...)
{
	va_list args;
	if(trace)
	{
		va_start(args, fmt);
		printf("%9.3f: ", current_timestamp());
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
		geodetic geodetic = ECEF_to_geodetic(state->pos);
		printf("%9.3f: %s %2.6f° lat, %3.6f° long, %8.2f alt, %8.2f vel, %8.2f acc",
		       current_timestamp(), source,
		       180 * geodetic.latitude / M_PI, 180 * geodetic.longitude / M_PI, geodetic.altitude,
		       vec_abs(state->vel), vec_abs(state->acc));
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

enum state last_reported_state(void)
{
	return fc_state;
}

void enqueue_error(const char *msg)
{
	trace_printf("Error message from rocket: %s\n", msg);
}
