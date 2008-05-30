// Copyright 2008 Julian Blake Kongslie
// Licensed under the GNU GPL version 2.

#include <assert.h>
#include <stdbool.h>

#include "flightsim.h"
#include "rocket.h"

bool flightsim_tick( double delta_t, struct flightsim_state *sim ) {

  // Update the rocket.
  update_rocket( delta_t, &(sim->rocket) );

  // If we cross the 1 second boundary while in the waiting state, start the burn.
  if ( sim->time < 1 && sim->time + delta_t >= 1 )
    start_burn( sim );

  // Update simulation time.
  sim->time += delta_t;

  // Return false only if we have landed.
  if ( sim->rocket.beeninair && sim->rocket.position.z < 1 )
    return false;

  return true;

};

void start_burn( struct flightsim_state *sim ) {
  sim->rocket.state = STATE_BURN;
};

void release_drogue_chute( struct flightsim_state *sim ) {
  assert( sim->rocket.beeninair );
  sim->rocket.state = STATE_DROGUECHUTE;
};

void release_main_chute( struct flightsim_state *sim ) {
  assert( sim->rocket.beeninair );
  sim->rocket.state = STATE_MAINCHUTE;
};
