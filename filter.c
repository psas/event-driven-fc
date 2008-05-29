// Copyright 2008 Julian Blake Kongslie
// Based on code by
//  Bart Massey
// Licensed under the GNU GPL version 2.

#include "filter.h"
#include "ziggurat/random.h"

void resample_particles( double total_weight, unsigned int src_length, struct particle *src, unsigned int dest_length, struct particle *dest ) {

  double u0       = polynomial( dest_length - 1 ) * total_weight;
  double t        = 0;
  unsigned int j  = 0;

  for ( unsigned int i = 0; i < dest_length; ++i ) {

    while ( t + src[j].weight < u0 && j < src_length )
      t += src[j++].weight;

    dest[i].state   = src[j].state;
    dest[i].weight  = 1;

    u0 = u0 + (total_weight - u0) * polynomial( dest_length - i - 1 );

  };

};
