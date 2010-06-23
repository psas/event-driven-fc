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

static const accelerometer_d accelerometer_var = { 1, 1, 1, 1 };
static const vec3 gyroscope_var = {{ 1, 1, 1 }};
static const vec3 magnetometer_var = {{ 1, 1, 1 }};
static const vec3 gps_pos_var = {{ 1, 1, 1 }};
static const vec3 gps_vel_var = {{ 1, 1, 1 }};
static const double pressure_var = 1;

static const vec3 acc_sd_rel = {{ 0.01, 0.01, 1 }};
static const vec3 vel_sd = {{ 0.2, 0.2, 0.2 }};
static const vec3 pos_sd = {{ 0.2, 0.2, 0.2 }};

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
		particle->weight = -log(PARTICLE_COUNT);
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

/* Returns the estimated number of effective particles */
static double normalize_particles(void)
{
	struct particle *particle;

	/* take everything down to below the maximum weight
	   so that exp() will underflow rather than overflow */
	double max_weight = particles[0].weight;
	for_each_particle(particle)
		if (particle->weight > max_weight)
			max_weight = particle->weight;

	/* compute the total adjusted weight */
	double total_weight = 0.0;
	for_each_particle(particle)
	{
		particle->weight -= max_weight;
		total_weight += exp(particle->weight);
	}
	total_weight = log(total_weight);

	/* adjust the particles */
	double squared_weights = 0.0;
	for_each_particle(particle)
	{
		particle->weight -= total_weight;
		squared_weights += exp(2.0 * particle->weight);
	}

	return 1.0 / squared_weights;
}

static void update_state(double delta_t)
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
			on_ground += exp(particle->weight);
		bool going_down = vec_dot(particle->s.pos, particle->s.vel) < 0;
		if(state == STATE_FLIGHT && going_down)
		{
			bool in_freefall = vec_abs(vec_sub(gravity_acceleration(&particle->s), particle->s.acc)) <= 2.0;
			if(in_freefall)
				deploy_drogue += exp(particle->weight);
			bool low_altitude = ECEF_to_geodetic(particle->s.pos).altitude - initial_geodetic.altitude <= 500.0;
			if(low_altitude && vel >= 10.0)
				deploy_main += exp(particle->weight);
		}
	}

	hysteresis(&on_ground_for, delta_t, on_ground > 0.5);
	hysteresis(&not_on_ground_for, delta_t, on_ground <= 0.5);
	hysteresis(&deploy_drogue_for, delta_t, deploy_drogue > 0.5);
	hysteresis(&deploy_main_for, delta_t, deploy_main > 0.5);

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
4) normalize weights
5) make control decisions
6) possibly resample
*/

void tick(double delta_t)
{
	struct particle *particle;

	double effective_particles = normalize_particles();

	update_state(delta_t);

	if(effective_particles < 50.0)
	{
		resample_regular(PARTICLE_COUNT, particles, PARTICLE_COUNT, particle_arrays[!which_particles], 1);
		which_particles = !which_particles;
		particles = particle_arrays[which_particles];
	}

	struct rocket_state centroid = {{}, {}, {}, {}, {}};
	for_each_particle(particle)
	{
		centroid.pos = vec_add(centroid.pos, vec_scale(particle->s.pos, exp(particle->weight)));
		centroid.vel = vec_add(centroid.vel, vec_scale(particle->s.vel, exp(particle->weight)));
		centroid.acc = vec_add(centroid.acc, vec_scale(particle->s.acc, exp(particle->weight)));
		centroid.rotvel = vec_add(centroid.rotvel, vec_scale(particle->s.rotvel, exp(particle->weight)));
	}

	trace_state("bpf", &centroid, "\n");

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

void accelerometer_sensor(accelerometer_i acc)
{
	struct particle *particle;
	for_each_particle(particle)
	{
		vec3 acc_noise = {{
			gaussian(acc_sd_rel.x),
			gaussian(acc_sd_rel.y),
			gaussian(acc_sd_rel.z),
		}};
		particle->s.acc = vec_add(particle->s.acc, rocket_to_ECEF(&particle->s, acc_noise));
		accelerometer_d local = accelerometer_measurement(&particle->s);
		particle->weight +=
			log_gprob(acc.x - local.x, accelerometer_var.x) +
			log_gprob(acc.y - local.y, accelerometer_var.y) +
			log_gprob(acc.z - local.z, accelerometer_var.z) +
			log_gprob(acc.q - local.q, accelerometer_var.q);
	}
}

void gyroscope_sensor(vec3_i rotvel)
{
	struct particle *particle;
	for_each_particle(particle)
	{
		vec3 local = gyroscope_measurement(&particle->s);
		particle->weight +=
			log_gprob(rotvel.x - local.x, gyroscope_var.x) +
			log_gprob(rotvel.y - local.y, gyroscope_var.y) +
			log_gprob(rotvel.z - local.z, gyroscope_var.z);
	}
}

void gps_sensor(vec3 ecef_pos, vec3 ecef_vel)
{
	struct particle *particle;
	for_each_particle(particle)
	{
		particle->s.pos.x += gaussian(pos_sd.x);
		particle->s.pos.y += gaussian(pos_sd.y);
		particle->s.pos.z += gaussian(pos_sd.z);
		particle->s.vel.x += gaussian(vel_sd.x);
		particle->s.vel.y += gaussian(vel_sd.y);
		particle->s.vel.z += gaussian(vel_sd.z);
		particle->weight +=
			log_gprob(ecef_pos.x - particle->s.pos.x, gps_pos_var.x) +
			log_gprob(ecef_pos.y - particle->s.pos.y, gps_pos_var.y) +
			log_gprob(ecef_pos.z - particle->s.pos.z, gps_pos_var.z) +
			log_gprob(ecef_vel.x - particle->s.vel.x, gps_vel_var.x) +
			log_gprob(ecef_vel.y - particle->s.vel.y, gps_vel_var.y) +
			log_gprob(ecef_vel.z - particle->s.vel.z, gps_vel_var.z);
	}
}

void pressure_sensor(unsigned pressure)
{
	struct particle *particle;
	for_each_particle(particle)
	{
		particle->s.pos.x += gaussian(pos_sd.x);
		particle->s.pos.y += gaussian(pos_sd.y);
		particle->s.pos.z += gaussian(pos_sd.z);
		double local = pressure_measurement(&particle->s);
		particle->weight += log_gprob(pressure - local, pressure_var);
	}
}

void magnetometer_sensor(vec3_i mag_vec)
{
	struct particle *particle;
	for_each_particle(particle)
	{
		vec3 local = magnetometer_measurement(&particle->s);
		particle->weight +=
			log_gprob(mag_vec.x - local.x, magnetometer_var.x) +
			log_gprob(mag_vec.y - local.y, magnetometer_var.y) +
			log_gprob(mag_vec.z - local.z, magnetometer_var.z);
	}
}
