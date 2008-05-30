// Copyright 2008 Julian Blake Kongslie
// Licensed under the GNU GPL version 2.

#include <stdbool.h>
#include <math.h>

#include "query.h"
#include "rocket.h"

bool trigger_drogue_chute( struct rocket *state ) {
  return  (   state->beeninair
          &&  state->state == STATE_COAST
          &&  state->velocity.z <= 5
          );
};

bool trigger_main_chute( struct rocket *state ) {
  return  (   state->beeninair
          &&  (   state->state == STATE_COAST
              ||  state->state == STATE_DROGUECHUTE
              )
          &&  state->position.z <= 500
          &&  abs( state->velocity.z ) <= 30
          &&  state->velocity.z <= 0
          );
};
