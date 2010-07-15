/* Copyright © 2010 Portland State Aerospace Society
 * See version control history for detailed authorship information.
 *
 * This program is licensed under the GPL version 2 or later.  Please see the
 * file COPYING in the source distribution of this software for license terms.
 */
/* Originally written by Holly Grimes */

#ifndef PRESSURE_SENSOR_H
#define PRESSURE_SENSOR_H

#include "compiler.h"

#define GRAVITATIONAL_ACCELERATION -9.80665
#define UNIVERSAL_GAS_CONSTANT 8.314472
#define NUMBER_OF_LAYERS 7
#define MAXIMUM_ALTITUDE 84852
#define MINIMUM_PRESSURE 0.3734
#define LAYER0_BASE_TEMPERATURE 288.15
#define LAYER0_BASE_PRESSURE 101325
#define MOLAR_MASS_DRY_AIR 0.0289644
#define AIR_GAS_CONSTANT UNIVERSAL_GAS_CONSTANT/MOLAR_MASS_DRY_AIR

void init_atmosphere(double ground_temperature, double ground_pressure);
double altitude_to_pressure(double) ATTR_WARN_UNUSED_RESULT;
double pressure_to_altitude(double) ATTR_WARN_UNUSED_RESULT;
double altitude_to_temperature(double altitude)ATTR_WARN_UNUSED_RESULT;
double altitude_to_air_density(double altitude)ATTR_WARN_UNUSED_RESULT;
#endif
