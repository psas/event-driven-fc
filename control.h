// Copyright 2008 Julian Blake Kongslie
// Licensed under the GNU GPL version 2.

#pragma once

#define PARTICLE_COUNT  100

#define RESAMPLE_THRESHOLD  (1.0/3.0)

#define ACCELEROMETER_FREQ    0.001
#define PRESSURE_SENSOR_FREQ  0.1

void run_flight_control ( void );
