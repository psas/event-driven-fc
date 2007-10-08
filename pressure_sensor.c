/*
 * Pressure Sensor Model, version 1.1
 *
 * written by Holly Grimes
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

/* outputs atmospheric pressure associated with the given altitude. altitudes
   are measured with respect to the mean sea level */
double pressure_sensor(double altitude) {

   /* lapse rate and base altitude for each layer in the atmosphere */
   double lapse_rate[NUMBER_OF_LAYERS] = 
        {-0.0065, 0.0, 0.001, 0.0028, 0.0, -0.0028, -0.002};
   int base_altitude[NUMBER_OF_LAYERS] = 
        {0, 11000, 20000, 32000, 47000, 51000, 71000};

   double base_temperature = 288.15;
   double base_pressure = 101325;

   double pressure;
   double base; /* base for function to determine pressure */
   double exponent; /* exponent for function to determine pressure */
   int layer_number; /* identifies layer in the atmosphere */
   int delta_z; /* difference between two altitudes */
   int done;
   int i;
   
   if (altitude < 0) /* illegal altitude */
      return -1;
   else if (altitude > MAXIMUM_ALTITUDE) /* FIX ME: use sensor data to improve model */
      return 0;
   else {
      /* calculate the base temperature and pressure for the atmospheric layer
         associated with our altitude */
      layer_number = 0;
      done = 0;
      for(i = 0; (!done && i < NUMBER_OF_LAYERS - 1); i++) {
         if (altitude > base_altitude[layer_number + 1]) {
            delta_z = base_altitude[layer_number + 1] - base_altitude[layer_number];
            if (lapse_rate[layer_number] == 0.0) {
               exponent = GRAVITATIONAL_ACCELERATION * delta_z 
                    / AIR_GAS_CONSTANT / base_temperature;
               base_pressure *= exp(exponent);
            }
            else {
               base = (lapse_rate[layer_number] * delta_z / base_temperature) + 1.0;
               exponent = GRAVITATIONAL_ACCELERATION / 
                    (AIR_GAS_CONSTANT * lapse_rate[layer_number]);
               base_pressure *= pow(base, exponent);
            }
            base_temperature += delta_z * lapse_rate[layer_number];
            layer_number++;
         }
         else
            done = 1;
      }

      /* calculate the pressure at the inputted altitude */
      delta_z = altitude - base_altitude[layer_number];
      if (lapse_rate[layer_number] == 0.0) {
         exponent = GRAVITATIONAL_ACCELERATION * delta_z 
              / AIR_GAS_CONSTANT / base_temperature;
         pressure = base_pressure * exp(exponent);
      }
      else {
         base = (lapse_rate[layer_number] * delta_z / base_temperature) + 1.0;
         exponent = GRAVITATIONAL_ACCELERATION /
              (AIR_GAS_CONSTANT * lapse_rate[layer_number]);
         pressure = base_pressure * pow(base, exponent);
      } 

      return pressure;
   }
}
