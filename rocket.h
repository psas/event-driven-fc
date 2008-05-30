// Copyright 2008 Julian Blake Kongslie
// Based on code by
//  Jamey Sharp
//  Josh Triplett
// Licensed under the GNU GPL version 2.

#pragma once

#include "ziggurat/random.h"

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

  double fuel;

};

static inline void vec_add( struct vec *a, struct vec *b ) {
  a->z += b->z;
};

static inline void vec_sub( struct vec *a, struct vec *b ) {
  a->z -= b->z;
};

static inline void vec_mul( struct vec *a, double b ) {
  a->z *= b;
};

static inline void permute_vec( struct vec *a, double sigma ) {
  a->z += gaussian( sigma );
};

void update_rocket( double delta_t, struct rocket *rocket );

void permute_rocket( double delta_t, struct rocket *rocket );

const char * state_names[STATE_COUNT];

void print_rocket( struct rocket *rocket );
