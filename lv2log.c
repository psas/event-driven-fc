#include <stdint.h>
#include <stdio.h>
#include <arpa/inet.h>

#include "interface.h"
#include "sim-common.h"

static uint32_t last_timestamp;

static double to_seconds(uint32_t timestamp)
{
	return timestamp / 100.0;
}

double current_timestamp(void)
{
	return to_seconds(last_timestamp);
}

void ignite(bool go)
{
	if(go)
		trace_printf("FC turned on igniter\n");
	else
		trace_printf("FC turned off igniter\n");
}

void drogue_chute(bool go)
{
	if(go)
		trace_printf("FC deployed drogue chute\n");
	else
		trace_printf("FC stopped deploying drogue chute\n");
}

void main_chute(bool go)
{
	if(go)
		trace_printf("FC deployed main chute\n");
	else
		trace_printf("FC stopped deploying main chute\n");
}

struct canmsg_t {
	uint32_t        id;             /* id<<5 + rtr<<4 + len */
	uint32_t        timestamp;
	unsigned char   data[8];
};

int main(int argc, const char *const argv[])
{
	parse_trace_args(argc, argv);
	/* Hardcoded because we can't extract it from the log. */
	initial_geodetic = (geodetic) {
		.latitude = 43.79575081,
		.longitude = -120.65137954,
		.altitude = 1373.46,
	};
	init(initial_geodetic);

	struct canmsg_t msg;
	bool processed_message = false;
	while(fread(&msg, sizeof(msg), 1, stdin) == 1)
	{
		msg.id = ntohl(msg.id);
		msg.timestamp = ntohl(msg.timestamp);
		if(msg.timestamp != 0 && processed_message && last_timestamp != msg.timestamp)
		{
			if(last_timestamp)
				tick(to_seconds(msg.timestamp - last_timestamp));
			last_timestamp = msg.timestamp;
			processed_message = false;
		}
		switch(msg.id)
		{
		case /* FC_REQUEST_STATE */ 0x0021:
			if(msg.data[0] == /* ArmingState */ 5)
				arm();
			break;
		case /* UMB_SET_ROCKETREADY */ 0x3241:
			/* We approximate launch time as the time LV2 indicated
			 * readiness to launch. */
			if(msg.data[0] == 1)
				launch();
			break;
		case /* IMU_ACCEL_DATA */ 0x1B88:
			accelerometer_sensor((accelerometer_i) {
				.x = (uint16_t) msg.data[0] << 8 | msg.data[1],
				.y = (uint16_t) msg.data[2] << 8 | msg.data[3],
				.z = (uint16_t) msg.data[4] << 8 | msg.data[5],
				.q = (uint16_t) msg.data[6] << 8 | msg.data[7],
			});
			processed_message = true;
			break;
		case /* PRESS_REPORT_DATA */ 0x6322:
			pressure_sensor((uint16_t) msg.data[0] << 8 | msg.data[1]);
			processed_message = true;
			break;
		case /* REC_SET_PYRO */ 0x0802:
			if(msg.data[0] == 1)
				trace_printf("LV2 fired drogue chute pyro\n");
			if(msg.data[0] == 3)
				trace_printf("LV2 fired main chute pyro\n");
			break;
		}
	}

	return 0;
}
