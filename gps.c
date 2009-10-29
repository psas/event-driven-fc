/* Orbital equations using GPS ephemeris
 * Implemented based on "Global Positioning System: Theory and Application",
 * Chapter 4, "GPS Navigation Data", J.J. Spilker Jr.,
 * and based on IS-GPS-200D.
 */

#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "vec.h"

struct ephemeris {
	uint8_t IODE; /* Issue of data (ephemeris) */
	int16_t C_rs; /* 2^-5 meters */
	int16_t delta_n; /* 2^-43 semicircles */
	int32_t M_0; /* 2^-31 semicircles */
	int16_t C_uc; /* 2^-29 radians */
	uint32_t e; /* 2^-33; effective range 0.03 */
	int16_t C_us; /* 2^-29 radians */
	uint32_t sqrt_A; /* 2^-19 sqrt(meters) */
	uint16_t t_oe; /* 2^4 seconds; effective range 604,784 */
	int16_t C_ic; /* 2^-29 radians */
	int32_t OMEGA_0; /* 2^-31 semicircles */
	int16_t C_is; /* 2^-29 radians */
	int32_t i_0; /* 2^-31 semicircles */
	int16_t C_rc; /* 2^-5 meters */
	int32_t omega; /* 2^-31 semicircles */
	int32_t OMEGADOT; /* 24-bit, 2^-43 semicircles/second */
	int16_t IDOT; /* 14-bit, 2^-43 semicircles/second */
};

static const double mu = 3.986005e14; /* meters^3/seconds^2 */
static const double sqrt_mu = 1.9964981843217388e7; /* meters^(3/2)/seconds */
static const double OMEGADOT_e = 7.2921151467e-5; /* radians/second */
static const double pi = 3.1415926535898; /* GPS value of pi */

static double scale(double x, int e)
{
	return x / (double) (INT64_C(1) << e);
}

static double solve_kepler(double M_k, double e)
{
	/* 10 iterations bounds the error by (0.03**11)/0.97.
	 * (e <= 0.03, |sin(M)| <= 1) */
	double X = 0;
	for (unsigned i = 0; i < 10; i++)
		X = e * sin(M_k + X);
	return X;
}

static vec3 gps_satellite_position(const struct ephemeris *ephemeris, double t /* seconds */)
{
	double sqrt_A = scale(ephemeris->sqrt_A, 19);
	double A = sqrt_A * sqrt_A;
	double n_0 = sqrt_mu / (A * sqrt_A);
	double t_oe = ephemeris->t_oe * 16.0;
	double t_k = t - t_oe;
	double n = n_0 + scale(ephemeris->delta_n, 43) * pi;
	double M_k = scale(ephemeris->M_0, 31) * pi + n * t_k;
	double e = scale(ephemeris->e, 33);
	double E_k = solve_kepler(M_k, e);
	double nu_k = atan2(sqrt(1 - e*e) * sin(E_k), cos(E_k) - e);
	double PHI_k = nu_k + scale(ephemeris->omega, 31) * pi;

	double delta_u_k = scale(ephemeris->C_us, 29) * sin(2 * PHI_k) + scale(ephemeris->C_uc, 29) * cos(2 * PHI_k);
	double delta_r_k = scale(ephemeris->C_rs, 5) * sin(2 * PHI_k) + scale(ephemeris->C_rc, 5) * cos(2 * PHI_k);
	double delta_i_k = scale(ephemeris->C_is, 29) * sin(2 * PHI_k) + scale(ephemeris->C_ic, 29) * cos(2 * PHI_k);

	double u_k = PHI_k + delta_u_k;
	double r_k = A * (1 - e * cos(E_k)) + delta_r_k;
	double i_k = scale(ephemeris->i_0, 31) * pi + delta_i_k + scale(ephemeris->IDOT, 43) * pi * t_k;

	double x_k_prime = r_k * cos(u_k);
	double y_k_prime = r_k * sin(u_k);

	double OMEGA_k = scale(ephemeris->OMEGA_0, 31) * pi + (scale(ephemeris->OMEGADOT, 43) * pi - OMEGADOT_e) * t_k - OMEGADOT_e * t_oe;

	return (vec3) {{
		.x = x_k_prime * cos(OMEGA_k) - y_k_prime * cos(i_k) * sin(OMEGA_k),
		.y = x_k_prime * sin(OMEGA_k) + y_k_prime * cos(i_k) * cos(OMEGA_k),
		.z = y_k_prime * sin(i_k),
	}};
}

int main(void)
{
	/* Data from PSAS 2005-08-20 flight, satellite 13.  Parity already removed. */
	const uint32_t subframe_2[] = { 0, 0, 0xc40d92, 0x2b475f, 0x772e13, 0x0bee01, 0x63fdf3, 0x0d5ca1, 0x0d6475, 0x00007f };
	const uint32_t subframe_3[] = { 0, 0, 0xfffb2e, 0xd811cd, 0xffe128, 0x4a5fe4, 0x21d82d, 0x42f0d9, 0xffa8f3, 0xc4198b };
	const struct ephemeris ephemeris = {
		.IODE = (subframe_2[2] >> 16) & 0xFF,
		.C_rs = subframe_2[2] & 0xFFFF,
		.delta_n = (subframe_2[3] >> 8) & 0xFFFF,
		.M_0 = ((subframe_2[3] & 0xFF) << 24) | subframe_2[4],
		.C_uc = (subframe_2[5] >> 8) & 0xFFFF,
		.e = ((subframe_2[5] & 0xFF) << 24) | subframe_2[6],
		.C_us = (subframe_2[7] >> 8) & 0xFFFF,
		.sqrt_A = ((subframe_2[7] & 0xFF) << 24) | subframe_2[8],
		.t_oe = (subframe_2[9] >> 8) & 0xFFFF,
		.C_ic = (subframe_3[2] >> 8) & 0xFFFF,
		.OMEGA_0 = ((subframe_3[2] & 0xFF) << 24) | subframe_3[3],
		.C_is = (subframe_3[4] >> 8) & 0xFFFF,
		.i_0 = ((subframe_3[4] & 0xFF) << 24) | subframe_3[5],
		.C_rc = (subframe_3[6] >> 8) & 0xFFFF,
		.omega = ((subframe_3[6] & 0xFF) << 24) | subframe_3[7],
		.OMEGADOT = subframe_3[8],
		.IDOT = (subframe_3[9] >> 2) & 0x3FFF,
	};
	for (uint32_t minute = 0; minute < 24*60; minute += 15) {
		vec3 pos = gps_satellite_position(&ephemeris, 86400*6 + minute*60);
		printf("%f %f %f\n", pos.x, pos.y, pos.z);
	}
	return 0;
}
