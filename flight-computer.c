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
#include "ziggurat/random.h"

/* Unless explicitly stated otherwise, all values use SI units: meters,
 * meters/second, meters/second^2, radians.
 */

static const accelerometer_d accelerometer_sd = { 1, 1, 1, 1 };
static const vec3 gyroscope_sd = {{ 1, 1, 1 }};
static const vec3 gps_pos_var = {{ 1, 1, 1 }};
static const vec3 gps_vel_var = {{ 1, 1, 1 }};
static const double pressure_sd = 1;

static const double prob_engine_trans = 0.01;
static const double prob_drogue_trans = 0.01;
static const double prob_main_trans  = 0.01;

static const double z_acc_sd = 100;
static const double z_vel_sd = 100;
static const double z_pos_sd = 100;
static const double mass_sd  = 0.1;

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
		particle->s.mass = ROCKET_EMPTY_MASS + FUEL_MASS;
		particle->s.pos = initial_ecef;
		particle->s.rotpos = initial_rotation;
	}
}

static void add_discrete_noise(double __attribute__((__unused__)) delta_t, struct particle *particle)
{
	double state_trans = uniform();
	if ((state_trans -= prob_engine_trans) < 0) {
		particle->s.engine_burning = ! particle->s.engine_burning;
		particle->s.acc = expected_acceleration(&particle->s);
		particle->weight *= 0.1;
	} else if ((state_trans -= prob_drogue_trans) < 0) {
		particle->s.drogue_chute_deployed = ! particle->s.drogue_chute_deployed;
		particle->s.acc = expected_acceleration(&particle->s);
		particle->weight *= 0.1;
	} else if ((state_trans -= prob_main_trans) < 0) {
		particle->s.main_chute_deployed = ! particle->s.main_chute_deployed;
		particle->s.acc = expected_acceleration(&particle->s);
		particle->weight *= 0.1;
	};
}

static void add_continuous_noise(double delta_t, struct particle *particle)
{
	particle->s.pos.z += delta_t * gaussian(z_pos_sd);
	particle->s.vel.z += delta_t * gaussian(z_vel_sd);
	particle->s.acc.z += delta_t * gaussian(z_acc_sd);
	particle->s.mass  += delta_t * gaussian(mass_sd);
}

static void update_state(double total_weight)
{
	struct particle *particle;
	double consensus_weight = 0;

	switch(state)
	{
		case STATE_PREFLIGHT:
			/* FIXME: check if pointing in the right direction. */
			for_each_particle(particle)
				if(vec_abs(vec_sub(initial_ecef, particle->s.pos)) <= 5.0)
					consensus_weight += particle->weight;
			can_arm = consensus_weight > total_weight/2;
			break;
		case STATE_ARMED:
			for_each_particle(particle)
				if(ECEF_to_rocket(&particle->s, particle->s.acc).z >= 1.0)
					consensus_weight += particle->weight;
			if(consensus_weight > total_weight/2)
				change_state(STATE_BOOST);
			break;
		case STATE_BOOST:
			ignite(false);
			for_each_particle(particle)
				if(ECEF_to_rocket(&particle->s, particle->s.acc).z <= 0.0)
					consensus_weight += particle->weight;
			if(consensus_weight > total_weight/2)
				change_state(STATE_COAST);
			break;
		case STATE_COAST:
			for_each_particle(particle)
				if(vec_dot(particle->s.pos, particle->s.vel) < 0)
					consensus_weight += particle->weight;
			if(consensus_weight > total_weight/2)
			{
				drogue_chute(true);
				change_state(STATE_DROGUE_DESCENT);
			}
			break;
		case STATE_DROGUE_DESCENT:
			drogue_chute(false);
			for_each_particle(particle)
				if(ECEF_to_geodetic(particle->s.pos).altitude - initial_geodetic.altitude <= 500.0)
					consensus_weight += particle->weight;
			if(consensus_weight > total_weight/2)
			{
				main_chute(true);
				change_state(STATE_MAIN_DESCENT);
			}
			break;
		case STATE_MAIN_DESCENT:
			main_chute(false);
			for_each_particle(particle)
				if(ECEF_to_geodetic(particle->s.pos).altitude - initial_geodetic.altitude <= 2.0
				   && vec_abs(particle->s.vel) <= 1.0)
					consensus_weight += particle->weight;
			if(consensus_weight > total_weight/2)
				change_state(STATE_RECOVERY);
			break;
		case STATE_RECOVERY:
			/* profit(); */
			break;
	}
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
	double total_weight = 0.0;
	int max_belief;
	struct particle *particle;

	for_each_particle(particle)
		total_weight += particle->weight;

	update_state(total_weight);

	max_belief = resample_optimal(total_weight, PARTICLE_COUNT, particles, PARTICLE_COUNT, particle_arrays[!which_particles]);
	which_particles = !which_particles;
	particles = particle_arrays[which_particles];

	trace_state("bpf", &particles[max_belief].s, " weight %6.2f\n", total_weight);

	for_each_particle(particle)
	{
		add_discrete_noise(delta_t, particle);
		update_rocket_state(&particle->s, delta_t);
		add_continuous_noise(delta_t, particle);
	}
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
		double local = pressure_measurement(&particle->s);
		particle->weight *= quantized_gprob(pressure, local, pressure_sd, 0xfff);
	}
}
