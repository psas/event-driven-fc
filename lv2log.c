/* Copyright © 2010 Portland State Aerospace Society
 * See version control history for detailed authorship information.
 *
 * This program is licensed under the GPL version 2 or later.  Please see the
 * file COPYING in the source distribution of this software for license terms.
 */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <arpa/inet.h>

#include "binary.h"
#include "interface.h"
#include "gps.h"
#include "sim-common.h"

static bool processed_message = false;
static uint32_t last_timestamp;
static struct gps_navigation_buffer channels[32];

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

static void add_navigation_word(uint8_t prn, uint16_t offset, uint32_t word)
{
	if((word >> 30) == 1)
	{
		struct gps_navigation_buffer *buffer = &channels[prn - 1];
		uint8_t old_IODE = buffer->IODE;
		gps_add_navigation_word(buffer, offset % 10, word & 0xFFFFFF);
		if(old_IODE != buffer->IODE)
		{
			printf("%02d", prn);
			printf(" IODE=%02x", buffer->IODE);
			printf(" C_rs=%+010.5f", buffer->ephemeris.C_rs);
			printf(" delta_n=%+e", buffer->ephemeris.delta_n);
			printf(" M_0=%+f", buffer->ephemeris.M_0);
			printf(" C_uc=%+e", buffer->ephemeris.C_uc);
			printf(" e=%f", buffer->ephemeris.e);
			printf(" C_us=%+e", buffer->ephemeris.C_us);
			printf(" sqrt_A=%f", buffer->ephemeris.sqrt_A);
			printf(" t_oe=%-6.0f", buffer->ephemeris.t_oe);
			printf(" C_ic=%+e", buffer->ephemeris.C_ic);
			printf(" OMEGA_0=%+f", buffer->ephemeris.OMEGA_0);
			printf(" C_is=%+e", buffer->ephemeris.C_is);
			printf(" i_0=%+f", buffer->ephemeris.i_0);
			printf(" C_rc=%+e", buffer->ephemeris.C_rc);
			printf(" omega=%+f", buffer->ephemeris.omega);
			printf(" OMEGADOT=%+e", buffer->ephemeris.OMEGADOT);
			printf(" IDOT=%+e", buffer->ephemeris.IDOT);
			printf("\n");
		}
	}
}

static size_t consume_gps(uint8_t gps_buffer[], size_t gps_length)
{
	static vec3 lastpos;
	static vec3 lastvel;
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
		vec3 pos = {
			.x = (int32_t) read32le(gps_buffer + 18) / 100.0,
			.y = (int32_t) read32le(gps_buffer + 22) / 100.0,
			.z = (int32_t) read32le(gps_buffer + 26) / 100.0,
		};
		vec3 vel = {
			.x = (int32_t) read32le(gps_buffer + 30) / 100.0,
			.y = (int32_t) read32le(gps_buffer + 34) / 100.0,
			.z = (int32_t) read32le(gps_buffer + 38) / 100.0,
		};
		lastpos = pos;
		lastvel = vel;
		printf("ECEF pos=(%.2f, %.2f, %.2f) vel=(%.2f, %.2f, %.2f)\n", pos.x, pos.y, pos.z, vel.x, vel.y, vel.z);
		gps_sensor(pos, vel);
		processed_message = true;
		break;
	case 1102: ;
		uint32_t gps_time_int = read32le(gps_buffer + 16);
		int32_t gps_time_frac = read32le(gps_buffer + 20);
		double gps_time = gps_time_int * 2.0 / 100.0 + gps_time_frac / 50.0 / (1 << 29);
		for (int i = 0; i < 12; ++i)
		{
			uint8_t *base = gps_buffer + (24 + 19 * i) * 2;
			uint16_t prn = read16le(base + 3 * 2);
			if (prn == 0 || prn > 32)
				continue;
			uint16_t offset = read16le(base);
			uint32_t word1 = read32le(base + 15 * 2);
			uint32_t word2 = read32le(base + 17 * 2);
			add_navigation_word(prn, offset, word1);
			add_navigation_word(prn, offset + 1, word2);
			if(channels[prn - 1].valid_ephemeris)
			{
				const double c = 2.99792458e8; /* speed of light, from IS-GPS-200D (WGS-84) */

				double code_phase = read48le(base + 5 * 2) / 50.0 / (UINT64_C(1) << 45);
				double carrier_velocity = (int32_t) read32le(base + 11 * 2) / (double) (UINT64_C(1) << 45);
				vec3 satpos, satvel;
				gps_satellite_position(&channels[prn - 1].ephemeris, gps_time - code_phase, &satpos, &satvel);

				/* from Global Position System, Theory and Applications, volume 1, chapter 9:
				 * Axelrad, Brown. GPS Navigation Algorithms */
				/* see also http://en.wikipedia.org/wiki/Relativistic_Doppler_effect */
				vec3 range = vec_sub(satpos, lastpos);
				vec3 line_of_sight = vec_scale(range, 1 / vec_abs(range));
				double doppler = 1 / c * vec_dot(vec_sub(satvel, lastvel), line_of_sight);

				printf("ECEF sat%02d pos=(%f, %f, %f) measured=%f expected=%f error=%f\n",
					prn, satpos.x, satpos.y, satpos.z, code_phase * c, vec_abs(range), code_phase * c - vec_abs(range));
				printf("ECEF sat%02d vel=(%f, %f, %f) measured=%g expected=%g error=%g\n",
					prn, satvel.x, satvel.y, satvel.z, carrier_velocity, doppler, carrier_velocity - doppler);
			}
		}
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
	init(initial_geodetic, make_LTP_rotation(initial_geodetic));

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
				.x = read16be(msg.data + 0),
				.y = read16be(msg.data + 2),
				.z = read16be(msg.data + 4),
				.q = read16be(msg.data + 6),
			});
			processed_message = true;
			break;
		case /* PRESS_REPORT_DATA */ 0x6322:
			pressure_sensor(read16be(msg.data));
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
