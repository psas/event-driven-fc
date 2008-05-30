// Copyright 2008 Julian Blake Kongslie
// Licensed under the GNU GPL version 2.

#include <stdbool.h>

#include "flightsim.h"
#include "rocket.h"

static bool flightsim_tick( double delta_t, struct flightsim_state *sim ) {

  // Update the rocket.
  update_rocket( delta_t, &(sim->rocket) );

  // If we get off the ground, remember that.
  if ( sim->rocket.position.z >= 10 )
    sim->beeninair = true;

  // If we cross the 1 second boundary while in the waiting state, start the burn.
  if ( sim->time < 1 && sim->time + delta_t >= 1 && sim->rocket.state == STATE_WAITING )
    sim->rocket.state = STATE_BURN;

  // Update simulation time.
  sim->time += delta_t;

  // Return false only if we have landed.
  if ( sim->beeninair && sim->rocket.position.z < 1 )
    return false;

  return true;

};
