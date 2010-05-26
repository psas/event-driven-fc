#ifndef INTERFACE_H
#define INTERFACE_H

#include <stdbool.h>
#include <stdint.h>

#include "coord.h"
#include "physics.h"

/* Flight computer begins in preflight state. When preflight checks pass
 * and an "arm" command is received, switch to armed state. However, at
 * any time, if the rocket doesn't seem to be sitting on the ground,
 * switch to flight state. When the rocket seems to be sitting on the
 * ground and it was in flight state, switch to recovery state. */
enum state
{
	STATE_PREFLIGHT,
	STATE_ARMED,
	STATE_FLIGHT,
	STATE_RECOVERY
};

typedef struct accelerometer_i {
	uint16_t x, y, z, q;
} accelerometer_i;

typedef struct vec3_i {
	uint16_t x, y, z;
} vec3_i;

/* Implemented by the flight computer */
void init(geodetic initial_geodetic);
void tick(double delta_t);
void arm(void);
void launch(void);
void accelerometer_sensor(accelerometer_i acc);
void gyroscope_sensor(vec3_i rotvel);
void magnetometer_sensor(vec3_i mag_vec);
void gps_sensor(vec3 ecef_pos, vec3 ecef_vel);
void pressure_sensor(unsigned pressure);

/* Implemented by the driver harness */
void trace_state(const char *source, struct rocket_state *state, const char *fmt, ...) ATTR_FORMAT(printf,3,4);
void report_state(enum state state);
void ignite(bool go);
void drogue_chute(bool go);
void main_chute(bool go);
void enqueue_error(const char *msg);

#endif /* INTERFACE_H */
