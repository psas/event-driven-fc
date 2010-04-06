#include <stdint.h>
#include <stdio.h>

#include "gps.h"
#include "vec.h"

int main(void)
{
	/* Data from PSAS 2005-08-20 flight, satellite 13.  Parity already removed. */
	const uint32_t subframe_2[] = { 0xc40d92, 0x2b475f, 0x772e13, 0x0bee01, 0x63fdf3, 0x0d5ca1, 0x0d6475, 0x00007f };
	const uint32_t subframe_3[] = { 0xfffb2e, 0xd811cd, 0xffe128, 0x4a5fe4, 0x21d82d, 0x42f0d9, 0xffa8f3, 0xc4198b };
	struct ephemeris ephemeris;
	parse_ephemeris(&ephemeris, subframe_2, subframe_3);
	for (uint32_t minute = 0; minute < 24*60; minute += 15) {
		vec3 pos, vel;
		gps_satellite_position(&ephemeris, 86400*6 + minute*60, &pos, &vel);
		printf("%f %f %f %f %f %f\n", pos.x, pos.y, pos.z, vel.x, vel.y, vel.z);
	}
	return 0;
}
