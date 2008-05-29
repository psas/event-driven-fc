// Copyright 2008 Julian Blake Kongslie
// Licensed under the GNU GPL version 2.

#pragma once

#include "rocket.h"

struct simulator_state {
  double        time;
  struct rocket rocket;
};

static const struct simulator_state initial_sim =
  { .time   = 0
  , .rocket = initial_rocket
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

static void simulator_tick( double delta_t, struct simulator_state *sim );
