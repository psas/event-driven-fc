#pragma once

#include <ziggurat/random.h>

#define EARTH_GRAVITY               9.80665
#define AIR_DENSITY                 1.225

#define DROGUE_CHUTE_DRAG           0.8
#define DROGUE_CHUTE_CROSS_SECTION  0.836954282802814
#define MAIN_CHUTE_DRAG             0.8
#define MAIN_CHUTE_CROSS_SECTION    7.429812032713523
#define ROCKET_DRAG                 0.36559
#define ROCKET_CROSS_SECTION        0.015327901242699

#define ENGINE_THRUST               3094.65

#define DRY_MASS                    21.54
#define FUEL_MASS                   5.9
#define BURN_RATE                   (5.9/4.3)

struct vec {
  double z;
};

struct rocket {

  enum
    { STATE_WAITING     // Waiting to launch.
    , STATE_BURN        // Launching, engine burn.
    , STATE_COAST       // Coasting.
    , STATE_DROGUECHUTE // Drogue chute deployed.
    , STATE_MAINCHUTE   // Main chute deployed.

    , STATE_COUNT       // Not a real state; used to count the states.
    } state;

  // These are in a global referential frame.
  struct vec position;
  struct vec velocity;
  struct vec accel;

  unsigned double fuel;

};

static inline void vec_add( struct vec *a, struct vec b ) {
  a->z += b->z;
};

static inline void vec_sub( struct vec *a, struct vec b ) {
  a->z -= b->z;
};

static inline void vec_mul( struct vec *a, double b ) {
  a->z *= b;
};

static inline void permute_vec( struct vec *a, unsigned double sigma ) {
  a->z += gaussian( sigma );
};

static inline void incorporate_drag( struct rocket *rocket, double drag_coeff, double cross_section ) {
  struct vec drag = {
    .z  = -sign(rocket->velocity.z)
        * 0.5 * AIR_DENSITY
        * rocket->velocity.z * rocket->velocity.z
        * drag_coeff
        * cross_section
    };
  vec_add( &(rocket->accel), &drag );
};

static inline void incorporate_thrust( struct rocket *rocket, double thrust ) {
  struct vec thrust = { .z = thrust };
  vec_add( &(rocket->accel), &thrust );
};

static inline void update_rocket( double delta_t, struct rocket *rocket ) {

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

static inline void permute_rocket( double delta_t, struct rocket *rocket ) {

  // I don't have a particularly good justification for the sigma for these permutations.
  permute_vec( &(rocket->position), delta_t );
  permute_vec( &(rocket->velocity), delta_t );
  permute_vec( &(rocket->accel), delta_t );

  // Likewise, I don't have a particularly good justification for the probability of state transition.
  if (rocket->state < STATE_COUNT && uniform( ) < 0.10)
    ++(rocket->state);

};
