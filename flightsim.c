// Copyright 2008 Julian Blake Kongslie
// Licensed under the GNU GPL version 2.

#include <assert.h>
#include <math.h>
#include <stdbool.h>

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
    sim->rocket.state = STATE_COAST;

  // And at the 6 second boundary, the engine restarts.
  if ( sim->time < 6 && sim->time + delta_t >= 6 )
    sim->rocket.state = STATE_BURN;

  // When coming down, we lose the chutes at 250 meters.
  if ( sim->rocket.state == STATE_MAINCHUTE && sim->rocket.position.z < 250 )
    sim->rocket.state = STATE_COAST;

  // Update simulation time.
  sim->time += delta_t;

  // Return false only if we have landed.
  if ( sim->rocket.beeninair && sim->rocket.position.z < 1 )
    return false;

  return true;

};

void start_burn( struct flightsim_state *sim ) {

  assert( sim->rocket.state == STATE_COAST || sim->rocket.state == STATE_BURN );

  sim->rocket.state = STATE_BURN;

};

void release_drogue_chute( struct flightsim_state *sim ) {

  static bool can_release = true;

  if ( ! can_release )
    return;

  can_release = false;

  assert( sim->rocket.beeninair );
  assert( sim->rocket.state == STATE_COAST || sim->rocket.state == STATE_DROGUECHUTE || sim->rocket.state == STATE_MAINCHUTE );
  assert( abs( sim->rocket.velocity.z ) <= 7 );

  sim->rocket.state = STATE_DROGUECHUTE;

};

void release_main_chute( struct flightsim_state *sim ) {

  static bool can_release = true;

  if ( ! can_release )
    return;

  can_release = false;

  assert( sim->rocket.beeninair );
  assert( sim->rocket.state == STATE_COAST || sim->rocket.state == STATE_DROGUECHUTE || sim->rocket.state == STATE_MAINCHUTE );
  assert( sim->rocket.position.z <= 600 );

  sim->rocket.state = STATE_MAINCHUTE;

};
