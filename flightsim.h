// Copyright 2008 Julian Blake Kongslie
// Licensed under the GNU GPL version 2.

#pragma once

#include <stdbool.h>

#include "rocket.h"

struct flightsim_state {
  double        time;
  bool          beeninair;
  struct rocket rocket;
};

static const struct flightsim_state initial_sim =
  { .time       = 0
  , .beeninair  = false
  , .rocket     =
    { .state    = STATE_WAITING
    , .position =
      { .z      = 0
      }
    , .velocity =
      { .z      = 0
      }
    , .accel    =
      { .z      = 0
      }
    , .fuel     = FUEL_MASS
    }
  };

bool flightsim_tick( double delta_t, struct flightsim_state *sim );

void start_burn( struct flightsim_state *sim );
void release_drogue_chute( struct flightsim_state *sim );
void release_main_chute( struct flightsim_state *sim );
