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

static const double z_acc_sd = 100;
static const double z_vel_sd = 100;
static const double z_pos_sd = 100;
static const double mass_sd  = 1;

#define PARTICLE_COUNT 1000
static struct particle particle_arrays[2][PARTICLE_COUNT];
static struct particle *particles = particle_arrays[0];
static unsigned int which_particles = 0;

#define for_each_particle(v) \
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
	particle->s.pos[Z] += delta_t * gaussian(z_pos_sd);
	particle->s.vel[Z] += delta_t * gaussian(z_vel_sd);
	particle->s.acc[Z] += delta_t * gaussian(z_acc_sd);
	particle->s.mass   += delta_t * gaussian(mass_sd);
}

void tick(double delta_t)
{
	double total_weight = 0.0;
	struct rocket_state centroid = { };
	struct particle *particle;

	for_each_particle(particle)
	{
		total_weight += particle->weight;
		centroid.pos[Z] += particle->weight * particle->s.pos[Z];
		centroid.vel[Z] += particle->weight * particle->s.vel[Z];
		centroid.acc[Z] += particle->weight * particle->s.acc[Z];
		centroid.mass   += particle->weight * particle->s.mass;

		add_random_noise(delta_t, particle);

		update_rocket_state(&particle->s, delta_t);
	}

	centroid.pos[Z] /= total_weight;
	centroid.vel[Z] /= total_weight;
	centroid.acc[Z] /= total_weight;
	centroid.mass   /= total_weight;

	printf("BPF: total weight: %f  centroid Z pos, vel, acc: %f %f %f\n",
	       total_weight, centroid.pos[Z], centroid.vel[Z], centroid.acc[Z]);

	resample_optimal(total_weight, PARTICLE_COUNT, particles, PARTICLE_COUNT, particle_arrays[!which_particles]);
	which_particles = !which_particles;
	particles = particle_arrays[which_particles];
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

void omniscience_9000(vec3 pos, vec3 vel, vec3 acc,
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
