#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include "coord.h"
#include "gprob.h"
#include "interface.h"
#include "particle.h"
#include "physics.h"
#include "pressure_sensor.h"
#include "resample.h"
#include "sensors.h"
#include "ziggurat/zrandom.h"

/* Unless explicitly stated otherwise, all values use SI units: meters,
 * meters/second, meters/second^2, radians.
 */

static const accelerometer_d accelerometer_sd = { 1, 1, 1, 1 };
static const vec3 gyroscope_sd = {{ 1, 1, 1 }};
static const vec3 magnetometer_sd = {{ 1, 1, 1 }};
static const vec3 gps_pos_var = {{ 1, 1, 1 }};
static const vec3 gps_vel_var = {{ 1, 1, 1 }};
static const double pressure_sd = 1;

static const double z_acc_sd = 1;
static const double z_vel_sd = 1;
static const double z_pos_sd = 1;

#define PARTICLE_COUNT 1000
static struct particle particle_arrays[2][PARTICLE_COUNT];
static struct particle *particles = particle_arrays[0];
static unsigned int which_particles = 0;

#define for_each_particle(particle) \
	for(particle = particles; \
	    particle != particles + PARTICLE_COUNT; \
	    particle++)

static enum state state = STATE_PREFLIGHT;

static bool can_arm;

static geodetic initial_geodetic;
static vec3 initial_ecef;
static mat3 initial_rotation;

static void change_state(enum state new_state)
{
	state = new_state;
	report_state(state);
}

void init(geodetic initial_geodetic_in)
{
	struct particle *particle;
	initial_geodetic = initial_geodetic_in;
	initial_ecef = geodetic_to_ECEF(initial_geodetic);
	initial_rotation = make_LTP_rotation(initial_geodetic);
	for_each_particle(particle)
	{
		particle->weight = 1.0;
		particle->s.pos = initial_ecef;
		particle->s.rotpos = initial_rotation;
	}
}

static void hysteresis(double *duration, double delta_t, bool set)
{
	if(set)
		*duration += delta_t;
	else
		*duration = 0;
}

static bool ratelimit(double *delay, double delta_t, bool enable)
{
	if((*delay -= delta_t) > 0)
		return false;
	*delay = 0;
	if(!enable)
		return false;
	*delay = 1.0;
	return true;
}

static void update_state(double total_weight, double delta_t)
{
	static double on_ground_for, not_on_ground_for;
	static double deploy_drogue_for, drogue_wait;
	static double deploy_main_for, main_wait;

	struct particle *particle;
	double on_ground = 0;
	double deploy_drogue = 0;
	double deploy_main = 0;

	for_each_particle(particle)
	{
		double vel = vec_abs(particle->s.vel);
		double acc = vec_abs(particle->s.acc);
		if(vel <= 2.0 && acc <= 2.0)
			on_ground += particle->weight;
		bool going_down = vec_dot(particle->s.pos, particle->s.vel) < 0;
		if(state == STATE_FLIGHT && going_down)
		{
			bool in_freefall = vec_abs(vec_sub(gravity_acceleration(&particle->s), particle->s.acc)) <= 2.0;
			if(in_freefall)
				deploy_drogue += particle->weight;
			bool low_altitude = ECEF_to_geodetic(particle->s.pos).altitude - initial_geodetic.altitude <= 500.0;
			if(low_altitude && vel >= 10.0)
				deploy_main += particle->weight;
		}
	}

	hysteresis(&on_ground_for, delta_t, on_ground > total_weight / 2);
	hysteresis(&not_on_ground_for, delta_t, on_ground <= total_weight / 2);
	hysteresis(&deploy_drogue_for, delta_t, deploy_drogue > total_weight / 2);
	hysteresis(&deploy_main_for, delta_t, deploy_main > total_weight / 2);

	/* FIXME: check if pointing in the right direction. */
	can_arm = on_ground_for > 0.25;

	if(not_on_ground_for > 1.0 && state != STATE_FLIGHT)
		change_state(STATE_FLIGHT);
	if(on_ground_for > 1.0 && state == STATE_FLIGHT)
		change_state(STATE_RECOVERY);

	if(ratelimit(&drogue_wait, delta_t, deploy_drogue_for > 0.25))
		drogue_chute(true);
	if(ratelimit(&main_wait, delta_t, deploy_main_for > 0.25))
		main_chute(true);
}

/*
1) update based on physics state
2) add noise
3) process sensor
4) make control decisions
5) possibly resample
*/

void tick(double delta_t)
{
	static double last_resample = 0.0;
	double total_weight = 0.0;
	int max_belief;
	struct particle *particle;

	for_each_particle(particle)
		total_weight += particle->weight;

	update_state(total_weight, delta_t);

	last_resample += delta_t;
	if(total_weight < PARTICLE_COUNT / 10000.0 || last_resample >= 1.0)
	{
		last_resample = 0.0;
		max_belief = resample_optimal(total_weight, PARTICLE_COUNT, particles, PARTICLE_COUNT, particle_arrays[!which_particles]);
		which_particles = !which_particles;
		particles = particle_arrays[which_particles];
	}
	else
	{
		max_belief = 0;
		for_each_particle(particle)
		{
			particle->weight *= PARTICLE_COUNT / total_weight;
			if(particle->weight > particles[max_belief].weight)
				max_belief = particle - particles;
		}
	}

	trace_state("bpf", &particles[max_belief].s, " weight %6.2f\n", total_weight);

	for_each_particle(particle)
		update_rocket_state(&particle->s, delta_t);
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

static double quantized_gprob(unsigned measured, double expected, double standard_dev, unsigned mask)
{
	double width = standard_dev * M_SQRT2;
	double hi = measured == mask ? 1 : (1 + erf((measured + 0.5 - expected) / width)) / 2;
	double lo = measured == 0    ? 0 : (1 + erf((measured - 0.5 - expected) / width)) / 2;
	double scale = (erf(0.5 / width) - erf(-0.5 / width)) / 2;
	return (hi - lo) / scale;
}

void accelerometer_sensor(accelerometer_i acc)
{
	struct particle *particle;
	for_each_particle(particle)
	{
		particle->s.acc.z += gaussian(z_acc_sd);
		accelerometer_d local = accelerometer_measurement(&particle->s);
		particle->weight *=
			quantized_gprob(acc.x, local.x, accelerometer_sd.x, 0xfff) *
			quantized_gprob(acc.y, local.y, accelerometer_sd.y, 0xfff) *
			quantized_gprob(acc.z, local.z, accelerometer_sd.z, 0xfff) *
			quantized_gprob(acc.q, local.q, accelerometer_sd.q, 0xfff);
	}
}

void gyroscope_sensor(vec3_i rotvel)
{
	struct particle *particle;
	for_each_particle(particle)
	{
		vec3 local = gyroscope_measurement(&particle->s);
		particle->weight *=
			quantized_gprob(rotvel.x, local.x, gyroscope_sd.x, 0xfff) *
			quantized_gprob(rotvel.y, local.y, gyroscope_sd.y, 0xfff) *
			quantized_gprob(rotvel.z, local.z, gyroscope_sd.z, 0xfff);
	}
}

void gps_sensor(vec3 ecef_pos, vec3 ecef_vel)
{
	struct particle *particle;
	for_each_particle(particle)
	{
		particle->s.pos.z += gaussian(z_pos_sd);
		particle->s.vel.z += gaussian(z_vel_sd);
		particle->weight *=
			gprob(ecef_pos.x - particle->s.pos.x, gps_pos_var.x) *
			gprob(ecef_pos.y - particle->s.pos.y, gps_pos_var.y) *
			gprob(ecef_pos.z - particle->s.pos.z, gps_pos_var.z) *
			gprob(ecef_vel.x - particle->s.vel.x, gps_vel_var.x) *
			gprob(ecef_vel.y - particle->s.vel.y, gps_vel_var.y) *
			gprob(ecef_vel.z - particle->s.vel.z, gps_vel_var.z);
	}
}

void pressure_sensor(unsigned pressure)
{
	struct particle *particle;
	for_each_particle(particle)
	{
		particle->s.pos.z += gaussian(z_pos_sd);
		double local = pressure_measurement(&particle->s);
		particle->weight *= quantized_gprob(pressure, local, pressure_sd, 0xfff);
	}
}

void magnetometer_sensor(vec3_i mag_vec)
{
	struct particle *particle;
	for_each_particle(particle)
	{
		vec3 local = magnetometer_measurement(&particle->s);
		particle->weight *=
			quantized_gprob(mag_vec.x, local.x, magnetometer_sd.x, 0xfff) *
			quantized_gprob(mag_vec.y, local.y, magnetometer_sd.y, 0xfff) *
			quantized_gprob(mag_vec.z, local.z, magnetometer_sd.z, 0xfff);
	}
}