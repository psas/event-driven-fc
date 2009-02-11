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
static const double pressure_sd = 1;

static const double prob_engine_trans = 0.01;
static const double prob_drogue_trans = 0.01;
static const double prob_main_trans  = 0.01;

static const double z_acc_sd = 100;
static const double z_vel_sd = 100;
static const double z_pos_sd = 100;
static const double mass_sd  = 1;

#define PARTICLE_COUNT 1000
#define PARTICLE_THRESHOLD (PARTICLE_COUNT/2)
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

static void add_random_noise(double delta_t, struct particle *particle)
{
	if (uniform() < prob_engine_trans) {
		particle->s.engine_burning = ! particle->s.engine_burning;
		particle->weight *= 0.1;
	} else if (uniform() < prob_drogue_trans) {
		particle->s.drogue_chute_deployed = ! particle->s.drogue_chute_deployed;
		particle->weight *= 0.1;
	} else if (uniform() < prob_main_trans) {
		particle->s.main_chute_deployed = ! particle->s.main_chute_deployed;
		particle->weight *= 0.1;
	};

	particle->s.pos.z += delta_t * gaussian(z_pos_sd);
	particle->s.vel.z += delta_t * gaussian(z_vel_sd);
	particle->s.acc.z += delta_t * gaussian(z_acc_sd);
	particle->s.mass  += delta_t * gaussian(mass_sd);
}

static void update_state(void)
{
	struct particle *particle;
	unsigned int count = 0;

	switch(state)
	{
		case STATE_PREFLIGHT:
			/* FIXME: check if pointing in the right direction. */
			for_each_particle(particle)
				if(vec_abs(vec_sub(initial_ecef, particle->s.pos)) <= 5.0)
					count++;
			can_arm = count > PARTICLE_THRESHOLD;
			break;
		case STATE_ARMED:
			for_each_particle(particle)
				if(ECEF_to_rocket(&particle->s, particle->s.acc).z >= 1.0)
					count++;
			if(count > PARTICLE_THRESHOLD)
				change_state(STATE_BOOST);
			break;
		case STATE_BOOST:
			ignite(false);
			for_each_particle(particle)
				if(ECEF_to_rocket(&particle->s, particle->s.acc).z <= 0.0)
					count++;
			if(count > PARTICLE_THRESHOLD)
				change_state(STATE_COAST);
			break;
		case STATE_COAST:
			for_each_particle(particle)
				if(vec_dot(particle->s.pos, particle->s.vel) < 0)
					count++;
			if(count > PARTICLE_THRESHOLD)
			{
				drogue_chute(true);
				change_state(STATE_DROGUE_DESCENT);
			}
			break;
		case STATE_DROGUE_DESCENT:
			drogue_chute(false);
			for_each_particle(particle)
				if(ECEF_to_geodetic(particle->s.pos).altitude - initial_geodetic.altitude <= 500.0)
					count++;
			if(count > PARTICLE_THRESHOLD)
			{
				main_chute(true);
				change_state(STATE_MAIN_DESCENT);
			}
			break;
		case STATE_MAIN_DESCENT:
			main_chute(false);
			for_each_particle(particle)
				if(ECEF_to_geodetic(particle->s.pos).altitude - initial_geodetic.altitude <= 2.0
				   && vec_abs(particle->s.vel) <= 0.01)
					count++;
			if(count > PARTICLE_THRESHOLD)
				change_state(STATE_RECOVERY);
			break;
		case STATE_RECOVERY:
			/* profit(); */
			break;
	}
}

void tick(double delta_t)
{
	double total_weight = 0.0;
	int max_belief;
	struct particle *particle;

	for_each_particle(particle)
		total_weight += particle->weight;

	max_belief = resample_optimal(total_weight, PARTICLE_COUNT, particles, PARTICLE_COUNT, particle_arrays[!which_particles]);
	which_particles = !which_particles;
	particles = particle_arrays[which_particles];

	update_state();

	trace_state("bpf", &particles[max_belief].s, " weight %6.2f\n", total_weight);

	for_each_particle(particle)
	{
		add_random_noise(delta_t, particle);
		update_rocket_state(&particle->s, delta_t);
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

void pressure_sensor(unsigned pressure)
{
	struct particle *particle;
	for_each_particle(particle)
	{
		double local = pressure_measurement(&particle->s);
		particle->weight *= quantized_gprob(pressure, local, pressure_sd, 0xfff);
	}
}
