// Copyright 2008 Julian Blake Kongslie
// Licensed under the GNU GPL version 2.

#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#include "flightsim.h"
#include "rocket.h"

bool flightsim_tick( double delta_t, struct flightsim_state *sim ) {

  // Update the rocket.
  update_rocket( delta_t, &(sim->rocket) );

  // If we cross the 1 second boundary while in the waiting state, start the burn.
  if ( sim->time < 1 && sim->time + delta_t >= 1 )
    start_burn( sim );

  // At the 3 second boundary, the engine dies early.
  if ( sim->time < 3 && sim->time + delta_t >= 3 )
    sim->rocket.engine_burn = false;

  // And at the 6 second boundary, the engine restarts.
  if ( sim->time < 6 && sim->time + delta_t >= 6 )
    sim->rocket.engine_burn = true;

  // When coming down, we lose the chutes at 250 meters.
  if ( sim->rocket.main_chute_deployed && sim->rocket.position.z < 250 )
    sim->rocket.drogue_chute_deployed = sim->rocket.main_chute_deployed = false;

  // Update simulation time.
  sim->time += delta_t;

  // Return false only if we have landed.
  if ( sim->rocket.beeninair && sim->rocket.position.z < 1 )
    return false;

  return true;

};

void start_burn( struct flightsim_state *sim ) {

  printf( "%8.03f start_burn\n", sim->time );

  sim->rocket.engine_burn = true;

};

void deploy_drogue_chute( struct flightsim_state *sim ) {

  printf( "%8.03f deploy_drogue_chute\n", sim->time );

  if ( ! sim->can_drogue_chute )
    return;

  sim->can_drogue_chute = false;

  sim->rocket.drogue_chute_deployed = true;

};

void deploy_main_chute( struct flightsim_state *sim ) {

  printf( "%8.03f deploy_main_chute\n", sim->time );

  if ( ! sim->can_main_chute )
    return;

  sim->can_main_chute = false;

  sim->rocket.main_chute_deployed = true;

};
