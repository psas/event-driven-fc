#include <stdbool.h>
#include <math.h>
#include "fc.h"
#include "interface.h"

/* Position and rotation use local tangent plane (LTP); when on the launch
 * tower, all values read 0. Positive x goes east.  Positive y goes north.
 * Positive z goes up.  Rotation vectors are in (pitch, yaw, roll) order:
 * rotation around x, y, and z, respectively.
 *
 * Unless explicitly stated otherwise, all values use SI units: meters,
 * meters/second, meters/second^2, radians.
 */


static enum state state = STATE_PREFLIGHT;

static bool can_arm;

static void change_state(enum state new_state)
{
	state = new_state;
	report_state(state);
}

void arm(void)
{
	if(state == STATE_PREFLIGHT)
	{
		if(can_arm)
			change_state(STATE_ARMED);
		else
			enqueue_error("Cannot arm: safety conditions not met.");
	}
	else
		enqueue_error("Cannot arm: not in preflight state.");
}

void launch(void)
{
	if(state == STATE_ARMED)
		ignite(true);
	else
		enqueue_error("Cannot launch: not armed.");
}

void omniscience_9000(microseconds timestamp,
                      vec3 pos, vec3 vel, vec3 acc,
                      vec3 rotpos, vec3 rotvel)
{
	switch(state)
	{
		case STATE_PREFLIGHT:
			can_arm = pos[Z] <= 2.0
				  && rotpos[X] <= 0.01 && rotpos[Y] <= 0.01;
			break;
		case STATE_ARMED:
			if(acc[Z] >= 1.0)
				change_state(STATE_BOOST);
			break;
		case STATE_BOOST:
			ignite(false);
			if(acc[Z] <= 0.0)
				change_state(STATE_COAST);
			break;
		case STATE_COAST:
			if(fabs(vel[Z]) <= 5.0)
			{
				drogue_chute(true);
				change_state(STATE_DROGUE_DESCENT);
			}
			break;
		case STATE_DROGUE_DESCENT:
			drogue_chute(false);
			if(pos[Z] <= 500.0)
			{
				main_chute(true);
				change_state(STATE_MAIN_DESCENT);
			}
			break;
		case STATE_MAIN_DESCENT:
			main_chute(false);
			if(pos[Z] <= 2.0 && fabs(vel[Z]) <= 0.01)
				change_state(STATE_RECOVERY);
			break;
		case STATE_RECOVERY:
			/* profit(); */
			break;
	}
}

void z_accelerometer(double z_acc)
{
}

void pressure_sensor(double pressure)
{
}
