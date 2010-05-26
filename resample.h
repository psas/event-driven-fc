#ifndef _RESAMPLE_H
#define _RESAMPLE_H

#include "particle.h"

/* accepts a total weight, an input sample size, an input sample array,
   an output sample size, an output sample array */
/* returns a highest-weighted particle index */
int resample_optimal(double scale,
                     int m, struct particle *particle,
                     int n, struct particle *newp);

#endif
