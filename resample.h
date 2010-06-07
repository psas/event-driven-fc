#ifndef _RESAMPLE_H
#define _RESAMPLE_H

#include "particle.h"

/* returns a pointer to the highest-weighted particle */
void resample_regular(int m, struct particle *particle,
                      int n, struct particle *newp,
                      int sort);

#endif
