#include <stdio.h>

#include "interface.h"
#include "pressure_sensor.h"
#include "sensors.h"

double time;

void tick(double delta_t)
{
	time += delta_t;
}

void arm(void)
{
}

void launch(void)
{
}

#define G 9.80665

void accelerometer_sensor(accelerometer_i acc)
{
	const accelerometer_d bias = { 2420.722538, 2476.898031, 1830.218031 + 77, 1940.079375 };
	const accelerometer_d gain = {  392.80/G,  386.90/G,   77.00/G,   75.40/G };
	printf("accel\t%f\t%f\t%f\t%f\t%f\n", time,
		(acc.x - bias.x) / gain.x,
		(acc.y - bias.y) / gain.y,
		(acc.z - bias.z) / gain.z,
		(acc.q - bias.q) / gain.q);
}

void gyroscope_sensor(vec3_i rotvel)
{
	(void) rotvel;
}

void magnetometer_sensor(vec3_i mag_vec)
{
	(void) mag_vec;
}

void gps_sensor(vec3 ecef_pos, vec3 ecef_vel)
{
	double altitude = ECEF_to_geodetic(ecef_pos).altitude;
	printf("gps\t%f\t%f\n", time, altitude);
	(void) ecef_vel;
}

static double pressure_from_sensor(unsigned pressure)
{
	const double bias = -470.734;
	const double gain = 44.549779924087175;
	return (pressure - bias) / gain;
}

void pressure_sensor(unsigned pressure)
{
	double kPa = pressure_from_sensor(pressure);
	double altitude = pressure_to_altitude(kPa * 1000);
	printf("pressure\t%f\t%f\n", time, altitude);
}

void init(geodetic initial_geodetic_in, mat3 initial_rotation_in)
{
	(void) initial_geodetic_in;
	(void) initial_rotation_in;
}
