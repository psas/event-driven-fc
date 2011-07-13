/* Copyright © 2010 Portland State Aerospace Society
 * See version control history for detailed authorship information.
 *
 * This program is licensed under the GPL version 2 or later.  Please see the
 * file COPYING in the source distribution of this software for license terms.
 */
/* Originally written by Holly Grimes.
 *
 * Uses the International Standard Atmosphere as described in
 *   "A Quick Derivation relating altitude to air pressure" (version 1.03)
 *    from the Portland State Aerospace Society, except that the atmosphere
 *    is divided into layers with each layer having a different lapse rate.
 *
 * Lapse rate data for each layer was obtained from Wikipedia on Sept. 1, 2007
 *    at site <http://en.wikipedia.org/wiki/International_Standard_Atmosphere
 *
 * Height measurements use the local tangent plane.  The postive z-direction is up.
 *
 * All measurements are given in SI units (Kelvin, Pascal, meter, meters/second^2).
 *   The lapse rate is given in Kelvin/meter, the gas constant for air is given
 *   in Joules/(kilogram-Kelvin).
 */

#include <math.h>
#include "pressure_sensor.h"
/* lapse rate and base altitude for each layer in the atmosphere */
static const double lapse_rate[NUMBER_OF_LAYERS] =
     {-0.0065, 0.0, 0.001, 0.0028, 0.0, -0.0028, -0.002};
static const double base_altitude[NUMBER_OF_LAYERS] =
     {0, 11000, 20000, 32000, 47000, 51000, 71000};


/* Base temperature and pressure for each layer in the atmosphere. The values
 * given are the default, calculated from LAYER0_BASE_PRESSURE and
 * LAYER0_BASE_TEMPERATURE. The values are recalculated in a call to
 * init_atmosphere()
 */
static double base_pressure[NUMBER_OF_LAYERS] = {1.013250e+05, 2.263206e+04,
                        5.474885e+03, 8.680176e+02, 1.109061e+02, 6.693875e+01};
static double base_temperature[NUMBER_OF_LAYERS]= {2.881500e+02, 2.166500e+02,
                        2.166500e+02, 2.286500e+02, 2.706500e+02, 2.706500e+02};

void init_atmosphere(double ground_temperature, double ground_pressure){
    //TODO: accept altitude where measurements were taken, also humidity
    int layer_number;
    double delta_z;

    base_temperature[0] = ground_temperature;
    base_pressure[0] = ground_pressure;
    double base;
    double exponent;
    /* calculate the base temperature and pressure for all atmospheric layers
     */
    for(layer_number = 0; layer_number < NUMBER_OF_LAYERS - 1; ++layer_number) {
      delta_z = base_altitude[layer_number + 1] - base_altitude[layer_number];
      if (lapse_rate[layer_number] == 0.0) {
         exponent = GRAVITATIONAL_ACCELERATION * delta_z
              / AIR_GAS_CONSTANT / base_temperature[layer_number];
         base_pressure[layer_number+1] = base_pressure[layer_number] * exp(exponent);
      }
      else {
         base = (lapse_rate[layer_number] * delta_z / base_temperature[layer_number]) + 1.0;
         exponent = GRAVITATIONAL_ACCELERATION /
              (AIR_GAS_CONSTANT * lapse_rate[layer_number]);
         base_pressure[layer_number+1] = base_pressure[layer_number] * pow(base, exponent);
      }
      base_temperature[layer_number+1]= base_temperature[layer_number] + delta_z * lapse_rate[layer_number];
   }   
}


double altitude_to_air_density(double altitude)
{ //TODO: Humidity
    return altitude_to_pressure(altitude)/(AIR_GAS_CONSTANT*altitude_to_temperature(altitude));
}


double altitude_to_temperature(double altitude){
    int layer_number, delta_z;

    for(layer_number = 0; layer_number < NUMBER_OF_LAYERS - 1 && altitude > base_altitude[layer_number + 1]; ++layer_number)
        /* empty */;

    delta_z = altitude - base_altitude[layer_number];
    return base_temperature[layer_number] + delta_z*lapse_rate[layer_number];

}


/* outputs atmospheric pressure associated with the given altitude. altitudes
   are geopotential measured with respect to the mean sea level */
double altitude_to_pressure(double altitude) {

   double pressure;
   double base; /* base for function to determine pressure */
   double exponent; /* exponent for function to determine pressure */
   int layer_number; /* identifies layer in the atmosphere */
   int delta_z; /* difference between two altitudes */
   
   if (altitude > MAXIMUM_ALTITUDE) /* FIX ME: use sensor data to improve model */
      return 0;

   /*find correct layer_number*/
   for(layer_number = 0; layer_number < NUMBER_OF_LAYERS - 1 && altitude > base_altitude[layer_number + 1]; ++layer_number)
       /* empty */;

   /* calculate the pressure at the inputted altitude */
   delta_z = altitude - base_altitude[layer_number];
   if (lapse_rate[layer_number] == 0.0) {
      exponent = GRAVITATIONAL_ACCELERATION * delta_z 
           / AIR_GAS_CONSTANT / base_temperature[layer_number];
      pressure = base_pressure[layer_number] * exp(exponent);
   }
   else {
      base = (lapse_rate[layer_number] * delta_z / base_temperature[layer_number]) + 1.0;
      exponent = GRAVITATIONAL_ACCELERATION /
           (AIR_GAS_CONSTANT * lapse_rate[layer_number]);
      pressure = base_pressure[layer_number] * pow(base, exponent);
   } 
   return pressure;
}


/* outputs the altitude associated with the given pressure. the altitude
   returned is measured with respect to the mean sea level */
double pressure_to_altitude(double pressure) {
   double next_base_temperature = LAYER0_BASE_TEMPERATURE;
   double next_base_pressure = LAYER0_BASE_PRESSURE;

   double altitude;
   double base_pressure;
   double base_temperature;
   double base; /* base for function to determine base pressure of next layer */
   double exponent; /* exponent for function to determine base pressure
                             of next layer */
   double coefficient;
   int layer_number; /* identifies layer in the atmosphere */
   int delta_z; /* difference between two altitudes */

   if (pressure < 0)  /* illegal pressure */
      return -1;
   if (pressure < MINIMUM_PRESSURE) /* FIX ME: use sensor data to improve model */
      return MAXIMUM_ALTITUDE;

   /* calculate the base temperature and pressure for the atmospheric layer
      associated with the inputted pressure. */
   layer_number = -1;
   do {
      layer_number++;
      base_pressure = next_base_pressure;
      base_temperature = next_base_temperature;
      delta_z = base_altitude[layer_number + 1] - base_altitude[layer_number];
      if (lapse_rate[layer_number] == 0.0) {
         exponent = GRAVITATIONAL_ACCELERATION * delta_z 
              / AIR_GAS_CONSTANT / base_temperature;
         next_base_pressure *= exp(exponent);
      }
      else {
         base = (lapse_rate[layer_number] * delta_z / base_temperature) + 1.0;
         exponent = GRAVITATIONAL_ACCELERATION / 
              (AIR_GAS_CONSTANT * lapse_rate[layer_number]);
         next_base_pressure *= pow(base, exponent);
      }
      next_base_temperature += delta_z * lapse_rate[layer_number];
   }
   while(layer_number < NUMBER_OF_LAYERS - 1 && pressure < next_base_pressure);

   /* calculate the altitude associated with the inputted pressure */
   if (lapse_rate[layer_number] == 0.0) {
      coefficient = (AIR_GAS_CONSTANT / GRAVITATIONAL_ACCELERATION) 
                                                    * base_temperature;
      altitude = base_altitude[layer_number]
                    + coefficient * log(pressure / base_pressure);
   }
   else {
      base = pressure / base_pressure;
      exponent = AIR_GAS_CONSTANT * lapse_rate[layer_number] 
                                       / GRAVITATIONAL_ACCELERATION;
      coefficient = base_temperature / lapse_rate[layer_number];
      altitude = base_altitude[layer_number]
                      + coefficient * (pow(base, exponent) - 1);
   }

   return altitude;
}
