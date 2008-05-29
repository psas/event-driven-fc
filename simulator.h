// Copyright 2008 Julian Blake Kongslie
// Licensed under the GNU GPL version 2.

#pragma once

#include <stdbool.h>

#include "rocket.h"

struct simulator_state {
  double        time;
  bool          beeninair
  struct rocket rocket;
};

static const struct simulator_state initial_sim =
  { .time       = 0
  , .beeninair  = false
  , .rocket     = initial_rocket
  };

static const struct rocket initial_rocket =
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
  };

static bool simulator_tick( double delta_t, struct simulator_state *sim );
