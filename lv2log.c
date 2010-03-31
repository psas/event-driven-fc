#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <arpa/inet.h>

#include "interface.h"
#include "sim-common.h"

static bool processed_message = false;
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

static uint16_t read16(const unsigned char *buf)
{
	return (uint16_t) buf[0] << 8 | buf[1];
}

static uint16_t read16le(const uint8_t *buf)
{
	return (uint16_t) buf[1] << 8 | buf[0];
}

static uint32_t read32le(const uint8_t *buf)
{
	return (uint32_t) buf[3] << 24 | buf[2] << 16 | buf[1] << 8 | buf[0];
}

static size_t consume_gps(uint8_t gps_buffer[], size_t gps_length)
{
	if (gps_length < 10)
		return 0;
	if (gps_buffer[0] != 0xFF || gps_buffer[1] != 0x81)
		return 1;
	uint16_t sum = 0;
	size_t sum_word;
	for (sum_word = 0; sum_word < 5; ++sum_word)
		sum += read16le(gps_buffer + sum_word * 2);
	if (sum != 0)
		return 2;
	uint16_t word_count = read16le(gps_buffer + 4) + 1;
	if (gps_length < 10U + word_count * 2U)
		return 0;
	for (; sum_word < 5U + word_count; ++sum_word)
		sum += read16le(gps_buffer + sum_word * 2);
	if (sum != 0)
		return 2;
	switch (read16le(gps_buffer + 2))
	{
	case 1009: ;
		vec3 pos = {{
			.x = (int32_t) read32le(gps_buffer + 18) / 100.0,
			.y = (int32_t) read32le(gps_buffer + 22) / 100.0,
			.z = (int32_t) read32le(gps_buffer + 26) / 100.0,
		}};
		vec3 vel = {{
			.x = (int32_t) read32le(gps_buffer + 30) / 100.0,
			.y = (int32_t) read32le(gps_buffer + 34) / 100.0,
			.z = (int32_t) read32le(gps_buffer + 38) / 100.0,
		}};
		gps_sensor(pos, vel);
		processed_message = true;
		break;
	}
	return 10 + 2 * word_count;
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
	uint8_t gps_buffer[4096];
	size_t gps_length = 0;
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
		size_t length = msg.id & 0xF;
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
				.x = read16(msg.data + 0),
				.y = read16(msg.data + 2),
				.z = read16(msg.data + 4),
				.q = read16(msg.data + 6),
			});
			processed_message = true;
			break;
		case /* PRESS_REPORT_DATA */ 0x6322:
			pressure_sensor(read16(msg.data));
			processed_message = true;
			break;
		case /* REC_SET_PYRO */ 0x0802:
			if(msg.data[0] == 1)
				trace_printf("LV2 fired drogue chute pyro\n");
			if(msg.data[0] == 3)
				trace_printf("LV2 fired main chute pyro\n");
			break;
		case /* GPS_UART_TRANSMIT */ 0x5301 ... 0x5308:
			assert (gps_length + length < sizeof(gps_buffer));
			memcpy(gps_buffer + gps_length, msg.data, length);
			gps_length += length;
			while((length = consume_gps(gps_buffer, gps_length)))
			{
				gps_length -= length;
				memmove(gps_buffer, gps_buffer + length, gps_length);
			}
			break;
		}
	}

	return 0;
}
