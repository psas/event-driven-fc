// Copyright 2008 Julian Blake Kongslie
// Based on code by
//  Jamey Sharp
//  Josh Triplett
// Licensed under the GNU GPL version 2.

#include <stdio.h>

#include "rocket.h"
#include "ziggurat/random.h"

static inline void incorporate_drag( struct rocket *rocket, double mass, double drag_coeff, double cross_section ) {

  struct vec drag =
    { .x  = 0
    , .y  = 0
    , .z  = (rocket->velocity.z > 0 ? -1 : 1)
          * 0.5 * AIR_DENSITY
          * rocket->velocity.z * rocket->velocity.z
          * drag_coeff
          * cross_section
          / mass
    };

  vec_add( &(rocket->accel), &drag );

};

static inline void incorporate_thrust( struct rocket *rocket, double mass, double thrust ) {

  struct vec thrust_v =
    { .x = 0
    , .y = 0
    , .z = thrust / mass
    };

  vec_add( &(rocket->accel), &thrust_v );

};

void update_rocket( double delta_t, struct rocket *rocket ) {

  struct vec  accel     = rocket->accel;
  struct vec  velocity  = rocket->velocity;
  double      mass      = rocket->fuel + DRY_MASS;
  double      burn      = 0;

  // Main chute implies drogue chute.
  if ( rocket->main_chute_deployed )
    rocket->drogue_chute_deployed = true;

  // Gravity applies to all states.
  rocket->accel.z = -EARTH_GRAVITY;

  // Burn fuel mass.
  if ( rocket->engine_burn ) {
    if ( BURN_RATE * delta_t < rocket->fuel ) {
      burn = 1;
      rocket->fuel -= BURN_RATE * delta_t;
    } else {
      burn = rocket->fuel / (BURN_RATE * delta_t);
      rocket->fuel        = 0;
      rocket->engine_burn = false;
    };
  };

  // Calculate the average mass for this timestep.
  mass = (mass + rocket->fuel + DRY_MASS) / 2.0;

  // If we're burning fuel, incorporate thrust.
  if ( burn > 0 )
    incorporate_thrust( rocket, mass, ENGINE_THRUST * burn );

  // Drag from the rocket body applies to all states.
  incorporate_drag( rocket, mass, ROCKET_DRAG, ROCKET_CROSS_SECTION );

  // Incorporate drag from chute or rocket.
  if ( rocket->drogue_chute_deployed )
    incorporate_drag( rocket, mass, DROGUE_CHUTE_DRAG, DROGUE_CHUTE_CROSS_SECTION );
  if ( rocket->main_chute_deployed )
    incorporate_drag( rocket, mass, MAIN_CHUTE_DRAG, MAIN_CHUTE_CROSS_SECTION );

  // Integrate terms. I assume a linear transition from beginning of timestep to end of timestep.
  vec_add( &accel, &(rocket->accel) );
  vec_mul( &accel, 0.5 * delta_t );
  vec_add( &(rocket->velocity), &accel );

  vec_add( &velocity, &(rocket->velocity) );
  vec_mul( &velocity, 0.5 * delta_t );
  vec_add( &(rocket->position), &velocity );

  // Clamp to ground.
  if (rocket->position.z <= 0) {
    rocket->position.z  = 0;
    rocket->velocity.z  = 0;
  // Note if we make it above ground.
  } else if (rocket->position.z >= 10)
    rocket->beeninair = true;

  // Clamp fuel.
  if (rocket->fuel <= 0)
    rocket->fuel = 0;

};

void permute_rocket( double delta_t, struct rocket *rocket ) {

  // I don't have a particularly good justification for the sigma for these permutations.
  permute_vec( &(rocket->position), delta_t );
  permute_vec( &(rocket->velocity), delta_t );
  permute_vec( &(rocket->accel), delta_t );
  rocket->fuel += gaussian( delta_t );

  // Likewise, I don't have a particularly good justification for these probabilities of state transitions.
  if ( uniform() < 0.01 )
    if ( rocket->engine_burn || rocket->fuel > 0 )
      rocket->engine_burn = ! rocket->engine_burn;

  if ( rocket->beeninair && uniform() < 0.01 )
    rocket->drogue_chute_deployed = ! rocket->drogue_chute_deployed;

  if ( rocket->beeninair && uniform() < 0.01 )
    rocket->main_chute_deployed = ! rocket->main_chute_deployed;

};

void print_rocket( struct rocket *rocket ) {
  printf( "p<%7.02f %7.02f %7.02f> v<%7.02f %7.02f %7.02f> a<%7.02f %7.02f %7.02f> f<%4.02f> s<%s-%s-%s>"
    , rocket->position.x, rocket->position.y, rocket->position.z
    , rocket->velocity.x, rocket->velocity.y, rocket->velocity.z
    , rocket->accel.x, rocket->accel.y, rocket->accel.z
    , rocket->fuel
    , rocket->engine_burn?"BURN":"", rocket->drogue_chute_deployed?"DROGUE":"", rocket->main_chute_deployed?"MAIN":""
    );
};
