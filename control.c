// Copyright 2008 Julian Blake Kongslie
// Licensed under the GNU GPL version 2.

#include <assert.h>
#include <stdio.h>

#include "control.h"
#include "filter.h"
#include "flightsim.h"
#include "query.h"
#include "rocket.h"
#include "sensor.h"
#include "sensor/accelerometer.h"
#include "sensor/pressure_sensor.h"

void run_flight_control ( void ) {

  struct particle filter[2][PARTICLE_COUNT];
  unsigned int    which_filter = 0;

  struct flightsim_state  flightsim = initial_sim;

  double  time_until_accelerometer    = ACCELEROMETER_FREQ;
/*  double  time_until_pressure_sensor  = PRESSURE_SENSOR_FREQ;*/

  // The test functions.
  double test_accelerometer( struct rocket *state ) {
    return prob_given( state->accel.z, flightsim.rocket.accel.z, ACCELEROMETER_NOISE_SIGMA );
  };

/*  double test_pressure_sensor( struct rocket *state ) {*/
/*    return prob_given( altitude_to_pressure( state->position.z ), altitude_to_pressure( flightsim.rocket.position.z ), PRESSURE_SENSOR_NOISE_SIGMA );*/
/*  };*/

  // Initialize all the particles to the launch state.
  // If we want to be able to get a lock from scratch during mid-flight, we need a better distribution here.
  for ( int i = 0; i < PARTICLE_COUNT; ++i ) {
    filter[which_filter][i].weight            = 1;
    filter[which_filter][i].state.state       = STATE_WAITING;
    filter[which_filter][i].state.position.z  = 0;
    filter[which_filter][i].state.velocity.z  = 0;
    filter[which_filter][i].state.accel.z     = 0;
    filter[which_filter][i].state.fuel        = FUEL_MASS;
  };

  // This is the main control loop.
  do {

    // Update by the smaller of the two sensor times.
/*    double delta_t = time_until_accelerometer < time_until_pressure_sensor ? time_until_accelerometer : time_until_pressure_sensor;*/
    double delta_t = time_until_accelerometer;

    assert( delta_t > 0 );

    // Advance the flight simulator. Break out if it says the simulation has concluded.
    if ( ! flightsim_tick( delta_t, &flightsim ) )
      break;

    // Output the time.
    printf( "%8.03f "
      , flightsim.time
      );

    // Summarize the current flight simulator state.
    print_rocket( &(flightsim.rocket) );

    // Advance the particle filter.
    update_particles( delta_t, PARTICLE_COUNT, filter[which_filter] );

    // Apply the relevant sensor updates.
    // The = 0 is to fix a bogus gcc warning.
    double total_weight = 0;

    time_until_accelerometer -= delta_t;
/*    time_until_pressure_sensor -= delta_t;*/

    if ( time_until_accelerometer <= 0 ) {
      total_weight = test_particles( test_accelerometer, PARTICLE_COUNT, filter[which_filter] );
      time_until_accelerometer = ACCELEROMETER_FREQ;
    };

/*    if ( time_until_pressure_sensor <= 0 ) {*/
/*      total_weight = test_particles( test_pressure_sensor, PARTICLE_COUNT, filter[which_filter] );*/
/*      time_until_pressure_sensor = PRESSURE_SENSOR_FREQ;*/
/*    };*/

    // Query to find the most rocket state weights.
    double state_weights[STATE_COUNT];
    for ( int i = 0; i < STATE_COUNT; ++i )
      state_weights[i] = 0;

    for ( int i = 0; i < PARTICLE_COUNT; ++i )
      state_weights[filter[which_filter][i].state.state] += filter[which_filter][i].weight;

    printf( " (" );
    for ( int i = 0; i < STATE_COUNT; ++i )
      printf( " %3.0f", state_weights[i] );
    printf( ")" );

    // Finish the summary line.
    printf( " w<%3.0f>\n"
      , total_weight
      );

    // Make control decisions.
    if ( query_particles( detect_apogee_in_coast, PARTICLE_COUNT, filter[which_filter] ) ) {
      printf( "release drogue chute\n" );
      release_drogue_chute( &flightsim );
    } else if ( query_particles( detect_500m_in_fall, PARTICLE_COUNT, filter[which_filter] ) ) {
      printf( "release main chute\n" );
      release_main_chute( &flightsim );
    };

    // Resample if we drop below threshold.
    if ( total_weight < RESAMPLE_THRESHOLD ) {
      printf( "resample\n" );
      resample_particles( total_weight, PARTICLE_COUNT, filter[which_filter], PARTICLE_COUNT, filter[! which_filter] );
      which_filter = ! which_filter;
    };

  } while (1);

};
