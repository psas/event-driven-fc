// Copyright 2008 Julian Blake Kongslie
// Licensed under the GNU GPL version 2.

#include <stdbool.h>
#include <math.h>

#include "query.h"
#include "rocket.h"

bool trigger_drogue_chute( struct rocket *state ) {
  return  (   state->beeninair
          &&  !state->engine_burn
          &&  !state->drogue_chute_deployed
          &&  !state->main_chute_deployed
          &&  state->velocity.z <= 5
          );
};

bool trigger_main_chute( struct rocket *state ) {
  return  (   state->beeninair
          &&  !state->engine_burn
          &&  !state->main_chute_deployed
          &&  state->position.z <= 500
          &&  abs( state->velocity.z ) <= 30
          &&  state->velocity.z <= 0
          );
};
