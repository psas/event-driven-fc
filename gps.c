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
	double A = ephemeris->sqrt_A * ephemeris->sqrt_A;
	double n_0 = sqrt_mu / (A * ephemeris->sqrt_A);
	double t_oe = ephemeris->t_oe * 16.0;
	double t_k = t - t_oe;
	if (t_k > 302400)
		t_k -= 604800;
	else if (t_k < -302400)
		t_k += 604800;
	double n = n_0 + ephemeris->delta_n * pi;
	double M_k = ephemeris->M_0 * pi + n * t_k;
	double E_k = solve_kepler(M_k, ephemeris->e);
	double nu_k = atan2(sqrt(1 - ephemeris->e*ephemeris->e) * sin(E_k), cos(E_k) - ephemeris->e);
	double PHI_k = nu_k + ephemeris->omega * pi;

	double delta_u_k = ephemeris->C_us * sin(2 * PHI_k) + ephemeris->C_uc * cos(2 * PHI_k);
	double delta_r_k = ephemeris->C_rs * sin(2 * PHI_k) + ephemeris->C_rc * cos(2 * PHI_k);
	double delta_i_k = ephemeris->C_is * sin(2 * PHI_k) + ephemeris->C_ic * cos(2 * PHI_k);

	double u_k = PHI_k + delta_u_k;
	double r_k = A * (1 - ephemeris->e * cos(E_k)) + delta_r_k;
	double i_k = ephemeris->i_0 * pi + delta_i_k + ephemeris->IDOT * pi * t_k;

	double x_k_prime = r_k * cos(u_k);
	double y_k_prime = r_k * sin(u_k);

	double OMEGA_k = ephemeris->OMEGA_0 * pi + (ephemeris->OMEGADOT * pi - OMEGADOT_e) * t_k - OMEGADOT_e * t_oe;

	return (vec3) {{
		.x = x_k_prime * cos(OMEGA_k) - y_k_prime * cos(i_k) * sin(OMEGA_k),
		.y = x_k_prime * sin(OMEGA_k) + y_k_prime * cos(i_k) * cos(OMEGA_k),
		.z = y_k_prime * sin(i_k),
	}};
}

static int32_t mask_signed(uint32_t value, int bits)
{
    uint32_t mask = (UINT64_C(1) << bits) - 1;
    uint32_t sign_bit = UINT64_C(1) << (bits - 1);
    return ((value & mask) ^ sign_bit) - sign_bit;
}

int main(void)
{
	/* Data from PSAS 2005-08-20 flight, satellite 13.  Parity already removed. */
	const uint32_t subframe_2[] = { 0, 0, 0xc40d92, 0x2b475f, 0x772e13, 0x0bee01, 0x63fdf3, 0x0d5ca1, 0x0d6475, 0x00007f };
	const uint32_t subframe_3[] = { 0, 0, 0xfffb2e, 0xd811cd, 0xffe128, 0x4a5fe4, 0x21d82d, 0x42f0d9, 0xffa8f3, 0xc4198b };
	const struct ephemeris ephemeris = {
		.IODE = (subframe_2[2] >> 16) & 0xFF,
		.C_rs = scale(mask_signed(subframe_2[2], 16), 5),
		.delta_n = scale(mask_signed(subframe_2[3] >> 8, 16), 43),
		.M_0 = scale(mask_signed((subframe_2[3] << 24) | subframe_2[4], 32), 31),
		.C_uc = scale(mask_signed(subframe_2[5] >> 8, 16), 29),
		.e = scale(((subframe_2[5] & 0xFF) << 24) | subframe_2[6], 33),
		.C_us = scale(mask_signed(subframe_2[7] >> 8, 16), 29),
		.sqrt_A = scale(((subframe_2[7] & 0xFF) << 24) | subframe_2[8], 19),
		.t_oe = ((subframe_2[9] >> 8) & 0xFFFF) * 16.0,
		.C_ic = scale(mask_signed(subframe_3[2] >> 8, 16), 29),
		.OMEGA_0 = scale(mask_signed((subframe_3[2] << 24) | subframe_3[3], 32), 31),
		.C_is = scale(mask_signed(subframe_3[4] >> 8, 16), 29),
		.i_0 = scale(mask_signed((subframe_3[4] << 24) | subframe_3[5], 32), 31),
		.C_rc = scale(mask_signed(subframe_3[6] >> 8, 16), 5),
		.omega = scale(mask_signed((subframe_3[6] << 24) | subframe_3[7], 32), 31),
		.OMEGADOT = scale(mask_signed(subframe_3[8], 24), 43),
		.IDOT = scale(mask_signed(subframe_3[9] >> 2, 14), 43),
	};
	for (uint32_t minute = 0; minute < 24*60; minute += 15) {
		vec3 pos = gps_satellite_position(&ephemeris, 86400*6 + minute*60);
		printf("%f %f %f\n", pos.x, pos.y, pos.z);
	}
	return 0;
}
