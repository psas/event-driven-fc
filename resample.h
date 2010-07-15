/* Copyright © 2010 Portland State Aerospace Society
 * See version control history for detailed authorship information.
 *
 * This program is licensed under the GPL version 2 or later.  Please see the
 * file COPYING in the source distribution of this software for license terms.
 */
#ifndef _RESAMPLE_H
#define _RESAMPLE_H

#include "particle.h"

/* returns a pointer to the highest-weighted particle */
void resample_regular(int m, struct particle *particle,
                      int n, struct particle *newp,
                      int sort);

#endif
