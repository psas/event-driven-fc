/* Copyright © 2010 Portland State Aerospace Society
 * See version control history for detailed authorship information.
 *
 * This program is licensed under the GPL version 2 or later.  Please see the
 * file COPYING in the source distribution of this software for license terms.
 */
/* Orbital equations using GPS ephemeris
 * Implemented based on "Global Positioning System: Theory and Application",
 * Chapter 4, "GPS Navigation Data", J.J. Spilker Jr.,
 * and based on IS-GPS-200D.
 */
#include <math.h>
#include <stdint.h>

#include "gps.h"
#include "vec.h"

static const double mu = 3.986005e14; /* meters^3/seconds^2 */
static const double sqrt_mu = 1.9964981843217388e7; /* meters^(3/2)/seconds */
static const double OMEGADOT_e = 7.2921151467e-5; /* radians/second */
static const double pi = 3.1415926535898; /* GPS value of pi */

void gps_add_navigation_word(struct gps_navigation_buffer *buffer, uint32_t offset, uint32_t word)
{
	if(offset != buffer->next_offset && offset != 0)
	{
		/* not the offset we were looking for; start over */
		buffer->next_offset = 0;
		return;
	}
	buffer->next_offset = offset + 1;
	if(offset == 0)
	{
		buffer->TLM = word;
		return;
	}
	if(offset == 1)
	{
		buffer->HOW = word;
		return;
	}
	int subframe = (buffer->HOW >> 2) & 7;
	if(subframe != 2 && subframe != 3)
	{
		/* we don't care about other subframes yet */
		buffer->next_offset = 0;
		return;
	}
	(subframe == 2 ? buffer->subframe_2 : buffer->subframe_3)[offset - 2] = word;
	if(offset < 9)
		return;
	/* got a complete subframe */
	buffer->next_offset = 0;
	uint8_t IODE2 = (buffer->subframe_2[0] >> 16) & 0xFF;
	uint8_t IODE3 = (buffer->subframe_3[7] >> 16) & 0xFF;
	if(IODE2 == IODE3 && IODE2 != buffer->IODE)
	{
		buffer->IODE = IODE2;
		parse_ephemeris(&buffer->ephemeris, buffer->subframe_2, buffer->subframe_3);
		buffer->valid_ephemeris = 1;
	}
}

static double solve_kepler(double M_k, double e)
{
	/* 10 iterations bounds the error by (0.03**11)/0.97.
	 * (e <= 0.03, |sin(M)| <= 1) */
	double X = 0;
	for (unsigned i = 0; i < 10; i++)
		X = e * sin(M_k + X);
	return M_k + X;
}

/* Position computation from IS-GPS-200D, and Global Position System,
 * Theory and Applications, volume 1. Velocity computation from Benjamin
 * W. Remondi, "Computing satellite velocity using the broadcast
 * ephemeris", GPS Solutions (2004) 8:181-183. */
void gps_satellite_position(const struct ephemeris *ephemeris, double t /* seconds */, vec3 *pos, vec3 *vel)
{
	double A = ephemeris->sqrt_A * ephemeris->sqrt_A;
	double n_0 = sqrt_mu / (A * ephemeris->sqrt_A);
	double t_k = t - ephemeris->t_oe;
	if (t_k > 302400)
		t_k -= 604800;
	else if (t_k < -302400)
		t_k += 604800;
	double n = n_0 + ephemeris->delta_n;
	double M_k = ephemeris->M_0 + n * t_k;
	double Mdot_k = n;
	double E_k = solve_kepler(M_k, ephemeris->e);
	double Edot_k = Mdot_k / (1 - ephemeris->e * cos(E_k));
	double nu_k = atan2(sqrt(1 - ephemeris->e*ephemeris->e) * sin(E_k), cos(E_k) - ephemeris->e);
	double nudot_k = sin(E_k) * Edot_k * (1 + ephemeris->e * cos(nu_k)) / (sin(nu_k) * (1 - ephemeris->e * cos(E_k)));
	double PHI_k = nu_k + ephemeris->omega;

	double delta_u_k = ephemeris->C_us * sin(2 * PHI_k) + ephemeris->C_uc * cos(2 * PHI_k);
	double delta_r_k = ephemeris->C_rs * sin(2 * PHI_k) + ephemeris->C_rc * cos(2 * PHI_k);
	double delta_i_k = ephemeris->C_is * sin(2 * PHI_k) + ephemeris->C_ic * cos(2 * PHI_k);

	double u_k = PHI_k + delta_u_k;
	double r_k = A * (1 - ephemeris->e * cos(E_k)) + delta_r_k;
	double i_k = ephemeris->i_0 + delta_i_k + ephemeris->IDOT * t_k;

	double udot_k = nudot_k + 2 * (ephemeris->C_us * cos(2 * u_k) - ephemeris->C_uc * sin(2 * u_k)) * nudot_k;
	double rdot_k = A * ephemeris->e * sin(E_k) * n / (1 - ephemeris->e * cos(E_k)) + 2 * (ephemeris->C_rs * cos(2 * u_k) - ephemeris->C_rc * sin(2 * u_k)) * nudot_k;
	double idot_k = ephemeris->IDOT + (ephemeris->C_is * cos(2 * u_k) - ephemeris->C_ic * sin(2 * u_k)) * 2 * nudot_k;

	double x_k_prime = r_k * cos(u_k);
	double y_k_prime = r_k * sin(u_k);

	double xdot_k_prime = rdot_k * cos(u_k) - y_k_prime * udot_k;
	double ydot_k_prime = rdot_k * sin(u_k) + x_k_prime * udot_k;

	double OMEGA_k = ephemeris->OMEGA_0 + (ephemeris->OMEGADOT - OMEGADOT_e) * t_k - OMEGADOT_e * ephemeris->t_oe;
	double OMEGADOT_k = ephemeris->OMEGADOT - OMEGADOT_e;

	*pos = (vec3) {
		.x = x_k_prime * cos(OMEGA_k) - y_k_prime * cos(i_k) * sin(OMEGA_k),
		.y = x_k_prime * sin(OMEGA_k) + y_k_prime * cos(i_k) * cos(OMEGA_k),
		.z = y_k_prime * sin(i_k),
	};
	*vel = (vec3) {
		.x = (xdot_k_prime - y_k_prime * cos(i_k) * OMEGADOT_k) * cos(OMEGA_k) -
		     (x_k_prime * OMEGADOT_k + ydot_k_prime * cos(i_k) - y_k_prime * sin(i_k) * idot_k) * sin(OMEGA_k),
		.y = (xdot_k_prime - y_k_prime * cos(i_k) * OMEGADOT_k) * sin(OMEGA_k) +
		     (x_k_prime * OMEGADOT_k + ydot_k_prime * cos(i_k) - y_k_prime * sin(i_k) * idot_k) * cos(OMEGA_k),
		.z = ydot_k_prime * sin(i_k) + y_k_prime * cos(i_k) * idot_k,
	};
}

static int32_t mask_signed(uint32_t value, int bits)
{
    uint32_t mask = (UINT64_C(1) << bits) - 1;
    uint32_t sign_bit = UINT64_C(1) << (bits - 1);
    return ((value & mask) ^ sign_bit) - sign_bit;
}

void parse_ephemeris(struct ephemeris *ephemeris, const uint32_t subframe_2[], const uint32_t subframe_3[])
{
	*ephemeris = (struct ephemeris) {
		.C_rs = ldexp(mask_signed(subframe_2[0], 16), -5),
		.delta_n = ldexp(mask_signed(subframe_2[1] >> 8, 16), -43) * pi,
		.M_0 = ldexp(mask_signed((subframe_2[1] << 24) | subframe_2[2], 32), -31) * pi,
		.C_uc = ldexp(mask_signed(subframe_2[3] >> 8, 16), -29),
		.e = ldexp(((subframe_2[3] & 0xFF) << 24) | subframe_2[4], -33),
		.C_us = ldexp(mask_signed(subframe_2[5] >> 8, 16), -29),
		.sqrt_A = ldexp(((subframe_2[5] & 0xFF) << 24) | subframe_2[6], -19),
		.t_oe = ((subframe_2[7] >> 8) & 0xFFFF) * 16.0,
		.C_ic = ldexp(mask_signed(subframe_3[0] >> 8, 16), -29),
		.OMEGA_0 = ldexp(mask_signed((subframe_3[0] << 24) | subframe_3[1], 32), -31) * pi,
		.C_is = ldexp(mask_signed(subframe_3[2] >> 8, 16), -29),
		.i_0 = ldexp(mask_signed((subframe_3[2] << 24) | subframe_3[3], 32), -31) * pi,
		.C_rc = ldexp(mask_signed(subframe_3[4] >> 8, 16), -5),
		.omega = ldexp(mask_signed((subframe_3[4] << 24) | subframe_3[5], 32), -31) * pi,
		.OMEGADOT = ldexp(mask_signed(subframe_3[6], 24), -43) * pi,
		.IDOT = ldexp(mask_signed(subframe_3[7] >> 2, 14), -43) * pi,
	};
}
