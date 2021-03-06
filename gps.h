/* Copyright © 2010 Portland State Aerospace Society
 * See version control history for detailed authorship information.
 *
 * This program is licensed under the GPL version 2 or later.  Please see the
 * file COPYING in the source distribution of this software for license terms.
 */
#ifndef GPS_H
#define GPS_H

#include "vec.h"

struct ephemeris {
	double C_rs; /* meters */
	double delta_n; /* radians */
	double M_0; /* radians */
	double C_uc; /* radians */
	double e; /* effective range 0.03 */
	double C_us; /* radians */
	double sqrt_A; /* sqrt(meters) */
	double t_oe; /* seconds; effective range 604,784 */
	double C_ic; /* radians */
	double OMEGA_0; /* radians */
	double C_is; /* radians */
	double i_0; /* radians */
	double C_rc; /* meters */
	double omega; /* radians */
	double OMEGADOT; /* radians/second */
	double IDOT; /* radians/second */
};

struct gps_navigation_buffer {
	uint8_t IODE;
	uint8_t valid_ephemeris;
	uint8_t next_offset;
	uint32_t TLM;
	uint32_t HOW;
	uint32_t subframe_2[8];
	uint32_t subframe_3[8];
	struct ephemeris ephemeris;
};

void gps_add_navigation_word(struct gps_navigation_buffer *buffer, uint32_t offset, uint32_t word);
void parse_ephemeris(struct ephemeris *ephemeris, const uint32_t subframe_2[], const uint32_t subframe_3[]);
void gps_satellite_position(const struct ephemeris *ephemeris, double t /* seconds */, vec3 *pos, vec3 *vel);

#endif /* GPS_H */
