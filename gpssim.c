#define _GNU_SOURCE
#include <fenv.h>

#include <stdint.h>
#include <stdio.h>

#include "coord.h"
#include "gprob.h"
#include "gps.h"
#include "vec.h"
#include "ziggurat/zrandom.h"

#define NUM_PARTICLES 500

struct state {
	vec3 pos;
	vec3 vel;
};

static struct particle {
	struct state state;
	double weight;
} particle_set[2][NUM_PARTICLES];
static int which_particles;
#define particles particle_set[which_particles]

static const double duration = 100; /* seconds to simulate */
static const double init_vel_sd = 1; /* m/s */
static const double init_pos_sd = 1; /* m */

static const double acceleration_sd = 0.1; /* m/s/s */

static const double doppler_sd = 1e-9;
static const double doppler_var = 1e-18;

struct ephemeris ephemeris;
static vec3 ground;
static vec3 acceleration_axis;

static void init_simulation(struct state *state)
{
	/* Data from PSAS 2005-08-20 flight, satellite 13.  Parity already removed. */
	const uint32_t subframe_2[] = { 0xc40d92, 0x2b475f, 0x772e13, 0x0bee01, 0x63fdf3, 0x0d5ca1, 0x0d6475, 0x00007f };
	const uint32_t subframe_3[] = { 0xfffb2e, 0xd811cd, 0xffe128, 0x4a5fe4, 0x21d82d, 0x42f0d9, 0xffa8f3, 0xc4198b };
	parse_ephemeris(&ephemeris, subframe_2, subframe_3);

	vec3 satpos, satvel;
	gps_satellite_position(&ephemeris, ephemeris.t_oe + duration / 2, &satpos, &satvel);

	state->vel = (vec3) { 0, 0, 0 };
	geodetic g = ECEF_to_geodetic(satpos);
	g.altitude = 0;
	ground = geodetic_to_ECEF(g);
	state->pos = ground;
	acceleration_axis = vec_scale(state->pos, 1 / vec_abs(state->pos));
}

static void init_process(struct state *state)
{
	state->vel = vec_scale(acceleration_axis, gaussian(init_vel_sd));
	state->pos = vec_add(ground, vec_scale(acceleration_axis, gaussian(init_pos_sd)));
}

static void simulate_process(double delta_t, struct state *state)
{
	state->vel = vec_add(state->vel, vec_scale(acceleration_axis, gaussian(acceleration_sd) * delta_t));
	state->pos = vec_add(state->pos, vec_scale(state->vel, delta_t));
}

static double measure_doppler(vec3 pos, vec3 vel, vec3 satpos, vec3 satvel)
{
	const double c = 2.99792458e8; /* speed of light, from IS-GPS-200D (WGS-84) */

	/* from Global Position System, Theory and Applications, volume 1, chapter 9:
	 * Axelrad, Brown. GPS Navigation Algorithms */
	/* see also http://en.wikipedia.org/wiki/Relativistic_Doppler_effect */
	vec3 range = vec_sub(satpos, pos);
	vec3 line_of_sight = vec_scale(range, 1 / vec_abs(range));
	return 1 / c * vec_dot(vec_sub(satvel, vel), line_of_sight);
}

static double altitude(struct state *state)
{
	return vec_dot(vec_sub(state->pos, ground), acceleration_axis);
}

static double speed(struct state *state)
{
	return vec_dot(state->vel, acceleration_axis);
}

struct stats {
	double min;
	double mean;
	double max;
};

static struct stats init_stats(void)
{
	return (struct stats) {
		.min = INFINITY,
		.mean = 0,
		.max = -INFINITY,
	};
}

static void add_stats(struct stats *stats, double weight, double value)
{
	stats->mean += weight * value;
	if(value < stats->min)
		stats->min = value;
	if(value > stats->max)
		stats->max = value;
}

static double normalize(double total_weight)
{
	double squared_weights = 0;
	for(int i = 0; i < NUM_PARTICLES; ++i)
	{
		particles[i].weight /= total_weight;
		squared_weights += particles[i].weight * particles[i].weight;
	}
	return 1 / squared_weights;
}

static void resample(void)
{
	struct particle *source = particle_set[which_particles];
	which_particles = !which_particles;

	double init = uniform() * (1.0 / (NUM_PARTICLES + 1));
	double total = source->weight;
	for(int i = 0; i < NUM_PARTICLES; ++i)
	{
		while(total < init + i * (1.0 / (NUM_PARTICLES + 1)))
		{
			++source;
			total += source->weight;
		}
		particles[i].weight = 1.0 / NUM_PARTICLES;
		particles[i].state = source->state;
	}
}

int main(void)
{
	FILE *particle_log = fopen("particle_log.txt", "w");
	FILE *summary_log = fopen("summary_log.txt", "w");

	feenableexcept(FE_DIVBYZERO | FE_INVALID);

	struct state sim;
	init_simulation(&sim);

	for(int i = 0; i < NUM_PARTICLES; ++i)
	{
		init_process(&particles[i].state);
		particles[i].weight = 1.0 / NUM_PARTICLES;
	}

	const double delta_t = 1;
	for(double t = 0; t <= duration; t += delta_t)
	{
		vec3 satpos, satvel;
		gps_satellite_position(&ephemeris, ephemeris.t_oe + t, &satpos, &satvel);
		double true_measurement = measure_doppler(sim.pos, sim.vel, satpos, satvel);
		double measurement = true_measurement + gaussian(doppler_sd);

		double total_weight = 0;
		struct stats measurement_stats = init_stats();
		struct stats pos_stats = init_stats();
		struct stats vel_stats = init_stats();
		for(int i = 0; i < NUM_PARTICLES; ++i)
		{
			double expected_measurement = measure_doppler(particles[i].state.pos, particles[i].state.vel, satpos, satvel);
			particles[i].weight *= exp(log_gprob(measurement - expected_measurement, doppler_var));
			simulate_process(delta_t, &particles[i].state);

			total_weight += particles[i].weight;
			add_stats(&measurement_stats, particles[i].weight, expected_measurement);
			double pos = altitude(&particles[i].state);
			add_stats(&pos_stats, particles[i].weight, pos);
			double vel = speed(&particles[i].state);
			add_stats(&vel_stats, particles[i].weight, vel);

			fprintf(particle_log, "%4.1f\t%e\t%e\t%f\t%f\n", t, particles[i].weight, expected_measurement, pos, vel);
		}

		double effective_particles = normalize(total_weight);
		if(effective_particles < NUM_PARTICLES * 0.2)
			resample();

		fprintf(summary_log, "%4.1f %e %f\t%e %e %f %f\t%e %f %f\n",
			t, total_weight, effective_particles,
			true_measurement, measurement,
			altitude(&sim), speed(&sim),
			measurement_stats.mean / total_weight,
			pos_stats.mean / total_weight,
			vel_stats.mean / total_weight);

		simulate_process(delta_t, &sim);
	}
}
