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

/* lapse rate and base altitude for each layer in the atmosphere */
double lapse_rate[NUMBER_OF_LAYERS] = 
     {-0.0065, 0.0, 0.001, 0.0028, 0.0, -0.0028, -0.002};
int base_altitude[NUMBER_OF_LAYERS] = 
     {0, 11000, 20000, 32000, 47000, 51000, 71000};


/* outputs atmospheric pressure associated with the given altitude. altitudes
   are measured with respect to the mean sea level */
double altitude_to_pressure(double altitude) {
 
   double base_temperature = LAYER0_BASE_TEMPERATURE;
   double base_pressure = LAYER0_BASE_PRESSURE;

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
         associated with the inputted altitude */
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

   if (pressure < 0 || pressure > LAYER0_BASE_PRESSURE)  /* illegal pressure */
      return -1;
   else if (pressure < MINIMUM_PRESSURE) /* FIX ME: use sensor data to improve model */
      return MAXIMUM_ALTITUDE;
   else {
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
}
