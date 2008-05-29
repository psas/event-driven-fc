// Copyright 2008 Julian Blake Kongslie
// Licensed under the GNU GPL version 2.

#pragma once

#include <math.h>

static inline unsigned double prob_given( double value, double sensor, double sigma ) {
  return exp( -(sensor - value) * (sensor - value) * sigma );
};
