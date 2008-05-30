// Copyright 2008 Julian Blake Kongslie
// Licensed under the GNU GPL version 2.

#include <stdbool.h>

#include "query.h"
#include "rocket.h"

bool detect_apogee_in_coast( struct rocket *state ) {
  return  (   state->beeninair
          &&  state->state == STATE_COAST
          &&  state->position.z > 500
          &&  state->velocity.z <= 5
          );
};

bool detect_500m_in_fall( struct rocket *state ) {
  return  (   state->beeninair
          &&  (   state->state == STATE_COAST
              ||  state->state == STATE_DROGUECHUTE
              )
          &&  state->position.z <= 500
          &&  state->velocity.z <= 0
          );
};
