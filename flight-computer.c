#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include "fc.h"
#include "gprob.h"
#include "interface.h"
#include "particle.h"
#include "physics.h"
#include "pressure_sensor.h"
#include "resample.h"
#include "ziggurat/random.h"

/* Position and rotation use local tangent plane (LTP); when on the launch
 * tower, all values read 0. Positive x goes east.  Positive y goes north.
 * Positive z goes up.  Rotation vectors are in (pitch, yaw, roll) order:
 * rotation around x, y, and z, respectively.
 *
 * Unless explicitly stated otherwise, all values use SI units: meters,
 * meters/second, meters/second^2, radians.
 */

static const double z_accelerometer_sd = 0.01;
static const double pressure_sd = 10;

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

static void change_state(enum state new_state)
{
	state = new_state;
	report_state(state);
}

void init(void)
{
	struct particle *particle;
	for_each_particle(particle)
	{
		particle->weight = 1.0;
		particle->s.mass = ROCKET_EMPTY_MASS + FUEL_MASS;
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

	particle->s.pos[Z] += delta_t * gaussian(z_pos_sd);
	particle->s.vel[Z] += delta_t * gaussian(z_vel_sd);
	particle->s.acc[Z] += delta_t * gaussian(z_acc_sd);
	particle->s.mass   += delta_t * gaussian(mass_sd);
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
				if(particle->s.pos[Z] <= 2.0)
					count++;
			can_arm = count > PARTICLE_THRESHOLD;
			break;
		case STATE_ARMED:
			for_each_particle(particle)
				if(particle->s.acc[Z] >= 1.0)
					count++;
			if(count > PARTICLE_THRESHOLD)
				change_state(STATE_BOOST);
			break;
		case STATE_BOOST:
			ignite(false);
			for_each_particle(particle)
				if(particle->s.acc[Z] <= 0.0)
					count++;
			if(count > PARTICLE_THRESHOLD)
				change_state(STATE_COAST);
			break;
		case STATE_COAST:
			for_each_particle(particle)
				if(fabs(particle->s.vel[Z]) <= 5.0)
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
				if(particle->s.pos[Z] <= 500.0)
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
				if(particle->s.pos[Z] <= 2.0
				   && fabs(particle->s.vel[Z]) <= 0.01)
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

	update_state();

	printf("BPF: total weight: %.2f likely Z pos, vel, acc: %.2f %.2f %.2f (%s %s %s)\n",
	       total_weight, particles[max_belief].s.pos[Z], particles[max_belief].s.vel[Z], particles[max_belief].s.acc[Z],
	       particles[max_belief].s.engine_burning ? "BURN" : "", particles[max_belief].s.drogue_chute_deployed ? "DROGUE" : "", particles[max_belief].s.main_chute_deployed ? "MAIN" : "");
	which_particles = !which_particles;
	particles = particle_arrays[which_particles];

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

void z_accelerometer(double z_accelerometer)
{
	struct particle *particle;
	for_each_particle(particle)
	{
		particle->weight *= gprob(particle->s.acc[Z] - z_accelerometer, z_accelerometer_sd);
	}
}

void pressure_sensor(double pressure)
{
	struct particle *particle;
	double altitude = pressure_to_altitude(pressure);
	for_each_particle(particle)
	{
		particle->weight *= gprob(particle->s.pos[Z] - altitude, pressure_sd);
	}
}