// Copyright 2008 Julian Blake Kongslie
// Based on code by
//  Jamey Sharp
//  Josh Triplett
// Licensed under the GNU GPL version 2.

#include "rocket.h"
#include "ziggurat/random.h"

static inline void incorporate_drag( struct rocket *rocket, double drag_coeff, double cross_section ) {
  struct vec drag = {
    .z  = (rocket->velocity.z > 0 ? -1 : 1)
        * 0.5 * AIR_DENSITY
        * rocket->velocity.z * rocket->velocity.z
        * drag_coeff
        * cross_section
    };
  vec_add( &(rocket->accel), &drag );
};

static inline void incorporate_thrust( struct rocket *rocket, double thrust ) {
  struct vec thrust_v = { .z = thrust };
  vec_add( &(rocket->accel), &thrust_v );
};

void update_rocket( double delta_t, struct rocket *rocket ) {

  struct vec accel    = rocket->accel;
  struct vec velocity = rocket->velocity;

  // Gravity applies to all states.
  rocket->accel.z = -EARTH_GRAVITY;

  // Drag from the rocket applies to all states.
  incorporate_drag( rocket, ROCKET_DRAG, ROCKET_CROSS_SECTION );

  // In the burn state, we thrust and consume fuel. If all the fuel is consumed, we force ourselves into coast state.
  if (rocket->state == STATE_BURN) {
    rocket->fuel -= BURN_RATE * delta_t;
    if (rocket->fuel > 0)
      incorporate_thrust( rocket, ENGINE_THRUST );
    else {
      incorporate_thrust( rocket, ENGINE_THRUST * (BURN_RATE * delta_t + rocket->fuel) );
      rocket->state = STATE_COAST;
    };
  };

  // Incorporate drag from chutes.
  if (rocket->state == STATE_DROGUECHUTE || rocket->state == STATE_MAINCHUTE)
    incorporate_drag( rocket, DROGUE_CHUTE_DRAG, DROGUE_CHUTE_CROSS_SECTION );
  if (rocket->state == STATE_MAINCHUTE)
    incorporate_drag( rocket, MAIN_CHUTE_DRAG, MAIN_CHUTE_CROSS_SECTION );

  // Integrate terms. I assume a linear transition from beginning of timestep to end of timestep.
  vec_add( &accel, &(rocket->accel) );
  vec_mul( &accel, 0.5 * delta_t );
  vec_add( &(rocket->velocity), &accel );

  vec_add( &velocity, &(rocket->velocity) );
  vec_mul( &velocity, 0.5 * delta_t );
  vec_add( &(rocket->position), &velocity );

  // Clamp to ground.
  if (rocket->position.z <= 0)
    rocket->position.z = 0;

};

void permute_rocket( double delta_t, struct rocket *rocket ) {

  // I don't have a particularly good justification for the sigma for these permutations.
  permute_vec( &(rocket->position), delta_t );
  permute_vec( &(rocket->velocity), delta_t );
  permute_vec( &(rocket->accel), delta_t );

  // Likewise, I don't have a particularly good justification for the probability of state transition.
  if (rocket->state < STATE_COUNT && uniform( ) < delta_t)
    ++(rocket->state);

};
