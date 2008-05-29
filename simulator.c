// Copyright 2008 Julian Blake Kongslie
// Licensed under the GNU GPL version 2.

#include "rocket.h"

static void simulator_tick( double delta_t, struct simulator_state *sim ) {

  // Update the rocket.
  update_rocket( delta_t, &(sim->rocket) );

  // If we cross the 1 second boundary while in the waiting state, start the burn.
  if ( sim->time < 1 && sim->time + delta_t >= 1 && sim->rocket.state == STATE_WAITING )
    sim->rocket.state = STATE_BURN;

  // Update simulation time.
  sim->time += delta_t;

};
