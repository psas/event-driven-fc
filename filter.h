// Copyright 2008 Julian Blake Kongslie
// Licensed under the GNU GPL version 2.

#pragma once

#include <stdbool.h>

#include "rocket.h"

struct particle {
  double        weight;
  struct rocket state;
};

void resample_particles( double total_weight, unsigned int src_length, struct particle *src, unsigned int dest_length, struct particle *dest );

static inline void update_particles( double delta_t, unsigned int src_length, struct particle *src ) {
  for ( unsigned int i = 0; i < src_length; ++i ) {
    // Permute before update, so the effect of a random state transition is reflected by the new state variables.
    permute_rocket( delta_t, &(src[i].state) );
    update_rocket( delta_t, &(src[i].state) );
  };
};

static inline double test_particles( double (*test_func)( struct rocket *state ), unsigned int src_length, struct particle *src ) {

  double total_weight = 0;

  for ( unsigned int i = 0; i < src_length; ++i )
    total_weight += (src[i].weight *= test_func( &(src[i].state) ));

  return total_weight;

};

static inline bool query_particles( bool (*query_func)( struct rocket *state ), unsigned int src_length, struct particle *src ) {

  double total_weight = 0;
  double confidence = 0;

  for ( unsigned int i = 0; i < src_length; ++i ) {
    total_weight += src[i].weight;
    if ( query_func( &(src[i].state) ) )
      confidence += src[i].weight;
  };

  return (confidence > total_weight / 2);

};
