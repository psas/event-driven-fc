#ifndef GPS_H
#define GPS_H

#include "vec.h"

struct ephemeris {
	uint8_t IODE; /* Issue of data (ephemeris) */
	double C_rs; /* meters */
	double delta_n; /* semicircles */
	double M_0; /* semicircles */
	double C_uc; /* radians */
	double e; /* effective range 0.03 */
	double C_us; /* radians */
	double sqrt_A; /* sqrt(meters) */
	double t_oe; /* seconds; effective range 604,784 */
	double C_ic; /* radians */
	double OMEGA_0; /* semicircles */
	double C_is; /* radians */
	double i_0; /* semicircles */
	double C_rc; /* meters */
	double omega; /* semicircles */
	double OMEGADOT; /* semicircles/second */
	double IDOT; /* semicircles/second */
};


void parse_ephemeris(struct ephemeris *ephemeris, const uint32_t subframe_2[], const uint32_t subframe_3[]);
vec3 gps_satellite_position(const struct ephemeris *ephemeris, double t /* seconds */);

#endif /* GPS_H */
