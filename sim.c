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
#include "sim-common.h"
#include "ziggurat/zrandom.h"

static const microseconds DELTA_T = 1000;
#define DELTA_T_SECONDS (DELTA_T / 1000000.0)

static const microseconds LAUNCH_TIME = 1000000; /* One-second countdown */

static const accelerometer_d accelerometer_sd = { 1, 1, 1, 1 };
static const vec3 gyroscope_sd = {{ 1, 1, 1 }};
static const vec3 gps_pos_sd = {{ 1, 1, 1 }};
static const vec3 gps_vel_sd = {{ 1, 1, 1 }};
static const double pressure_sd = 1;

/* Drag constants */
static const double MAIN_CHUTE_DRAG_COEFFICIENT = 0.8;
static const double MAIN_CHUTE_CROSS_SECTION = 7.429812032713523;
static const double DROGUE_CHUTE_DRAG_COEFFICIENT = 0.8;
static const double DROGUE_CHUTE_CROSS_SECTION = 0.836954282802814;
static const double ROCKET_DRAG_COEFFICIENT = 0.36559;
static const double ROCKET_CROSS_SECTION = 0.015327901242699;
static const double AIR_DENSITY = 1.225;
static const double R = 287.05; //gas constant
static const double TEMPERATURE = 297; //about room temp in kelvins

static microseconds t;
static double mass;
static bool engine_ignited;
static microseconds engine_ignition_time;
static bool engine_burning;
static bool drogue_chute_deployed;
static bool main_chute_deployed;

/* State of the simulated rocket. */
static struct rocket_state rocket_state;

double current_timestamp(void)
{
	return t / 1e6;
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
			engine_burning = true;
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

static double air_density(vec3 rocket_pos)
{
    geodetic pos = ECEF_to_geodetic(rocket_pos);
    return altitude_to_pressure(pos.altitude)/(R*TEMPERATURE);
}

static vec3 drag_force(struct rocket_state *rocket_state)
{
	/* TODO: fix drag for rocket orientation */
	double drag_coefficient, cross_section;
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
	return vec_scale(rocket_state->vel, -0.5 * air_density(rocket_state->pos)
	                 * vec_abs(rocket_state->vel)
	                 * cross_section * drag_coefficient);
}

static vec3 thrust_force(struct rocket_state *rocket_state, microseconds time)
{
	if(!engine_burning)
	        return (vec3){{ 0, 0, 0 }};
	const microseconds ENGINE_RAMP_TIME = 200000;
	double scale = 1.0;
	if(time - engine_ignition_time < ENGINE_RAMP_TIME)
		scale = (double) (time - engine_ignition_time) / ENGINE_RAMP_TIME;
	else if(time - engine_ignition_time > ENGINE_BURN_TIME - ENGINE_RAMP_TIME)
		scale = (double) (engine_ignition_time + ENGINE_BURN_TIME - time) / ENGINE_RAMP_TIME;
	return rocket_to_ECEF(rocket_state, (vec3){{ 0, 0, scale * ENGINE_THRUST }});
}

static vec3 expected_acceleration(double time, struct rocket_state *rocket_state)
{
	/* TODO: add coefficient of normal force at the center of pressure */
	vec3 force = vec_add(thrust_force(rocket_state, (microseconds) time), drag_force(rocket_state));
	return vec_add(gravity_acceleration(rocket_state), vec_scale(force, 1/mass));
}

static unsigned quantize(double value, unsigned mask)
{
	long int rounded = lround(value);
	if(rounded < 0)
		return 0;
	if((unsigned long)rounded > mask)
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

static vec3_i quantize_vec(vec3 value, unsigned mask)
{
	return (vec3_i) {
		.x = quantize(value.x, mask),
		.y = quantize(value.y, mask),
		.z = quantize(value.z, mask),
	};
}

static accelerometer_d add_accelerometer_noise(accelerometer_d value)
{
	return (accelerometer_d) {
		.x = value.x + gaussian(accelerometer_sd.x),
		.y = value.y + gaussian(accelerometer_sd.y),
		.z = value.z + gaussian(accelerometer_sd.z),
		.q = value.q + gaussian(accelerometer_sd.q),
	};
}

static vec3 vec_noise(vec3 value, vec3 sd)
{
	return (vec3) {{
		.x = value.x + gaussian(sd.x),
		.y = value.y + gaussian(sd.y),
		.z = value.z + gaussian(sd.z),
	}};
}

static void ground_clip(vec3 *v, mat3 rot)
{
	const vec3 zero = {{ 0, 0, 0 }};
	vec3 ltp = ECEF_to_LTP(zero, rot, *v);
	if(ltp.z < 0)
	{
		ltp.z = 0;
		*v = LTP_to_ECEF(zero, rot, ltp);
	}
}

static void update_simulator(void)
{
	trace_state("sim", &rocket_state, ", %4.1f kg, %c%c%c\n",
	       mass,
	       engine_burning        ? 'B' : '-',
	       drogue_chute_deployed ? 'D' : '-',
	       main_chute_deployed   ? 'M' : '-');
	accelerometer_sensor(quantize_accelerometer(add_accelerometer_noise(accelerometer_measurement(&rocket_state)), 0xfff));
	if(t % 2000 == 0)
		gyroscope_sensor(quantize_vec(vec_noise(gyroscope_measurement(&rocket_state), gyroscope_sd), 0xfff));
	if(t % 100000 == 0)
		pressure_sensor(quantize(pressure_measurement(&rocket_state) + gaussian(pressure_sd), 0xfff));
        if(t % 100000 == 25000)
		magnetometer_sensor(quantize_vec(vec_noise(magnetometer_measurement(&rocket_state), gyroscope_sd), 0xfff)); 
	if(t % 100000 == 50000)
		gps_sensor(vec_noise(rocket_state.pos, gps_pos_sd),
		           vec_noise(rocket_state.vel, gps_vel_sd));
	if(!engine_ignited && t >= LAUNCH_TIME && last_reported_state() == STATE_ARMED)
	{
		trace_printf("Sending launch signal\n");
		launch();
	}
	if(engine_burning
	   && t - engine_ignition_time >= ENGINE_BURN_TIME)
	{
		trace_printf("Engine burn-out.\n");
		engine_burning = false;
	}
	if(last_reported_state() == STATE_PREFLIGHT)
	{
		trace_printf("Sending arm signal\n");
		arm();
	}
	if(engine_burning)
		mass -= FUEL_MASS * DELTA_T_SECONDS / (ENGINE_BURN_TIME / 1e6);
	rocket_state.acc = expected_acceleration((double)t, &rocket_state);
	geodetic pos = ECEF_to_geodetic(rocket_state.pos);
	if(pos.altitude <= initial_geodetic.altitude)
	{
		if(pos.altitude < initial_geodetic.altitude)
		{
			pos.altitude = initial_geodetic.altitude;
			rocket_state.pos = geodetic_to_ECEF(pos);
		}
		mat3 rot = make_LTP_rotation(pos);
		ground_clip(&rocket_state.vel, rot);
		ground_clip(&rocket_state.acc, rot);
	}
}

static void init_rocket_state(struct rocket_state *rocket_state)
{
	/* TODO: accept an initial orientation for leaving the tower */
	mass = ROCKET_EMPTY_MASS + FUEL_MASS;
	rocket_state->pos = geodetic_to_ECEF(initial_geodetic);
	rocket_state->rotpos = make_LTP_rotation(initial_geodetic);
       // printf("Y:%f %f %f\n", rocket_state->pos.x,rocket_state->pos.y,rocket_state->pos.z);
}

int main(int argc, const char *const argv[])
{
	parse_trace_args(argc, argv);
	initial_geodetic = (geodetic) {
		.latitude = M_PI_2,
		.longitude = 0,
		.altitude = 0,
	};

        /*TODO: get init to set things like temp, humidity, time of day etc*/

	init_rocket_state(&rocket_state);
	init(initial_geodetic);

	while(last_reported_state() != STATE_RECOVERY)
	{
		t += DELTA_T;
		update_rocket_state(&rocket_state, DELTA_T_SECONDS, expected_acceleration, (double)t);
		update_simulator();
		tick(DELTA_T_SECONDS);
	}
	return 0;
}
