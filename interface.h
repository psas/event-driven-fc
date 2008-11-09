#ifndef INTERFACE_H
#define INTERFACE_H

#include <stdbool.h>
#include <stdint.h>

#include "physics.h"

enum state
{
	STATE_PREFLIGHT,      // until both given command to arm, and ready to arm
	STATE_ARMED,          // until launch command given and upward thrust detected
	STATE_BOOST,          // until burnout detected
	STATE_COAST,          // until apogee detected and drogue deployed
	STATE_DROGUE_DESCENT, // until z < 500m
	STATE_MAIN_DESCENT,   // until touchdown
	STATE_RECOVERY        // forever
};

typedef struct accelerometer_i {
	uint16_t x, y, z, q;
} accelerometer_i;

/* Implemented by the flight computer */
void init(void);
void tick(double delta_t);
void arm(void);
void launch(void);
void accelerometer_sensor(accelerometer_i acc);
void pressure_sensor(unsigned pressure);

/* Implemented by the driver harness */
void trace_state(const char *source, struct rocket_state *state, const char *fmt, ...) ATTR_FORMAT(printf,3,4);
void report_state(enum state state);
void ignite(bool go);
void drogue_chute(bool go);
void main_chute(bool go);
void enqueue_error(const char *msg);

#endif /* INTERFACE_H */
