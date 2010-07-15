/* Copyright © 2010 Portland State Aerospace Society, Bart Massey
 * See version control history for detailed authorship information.
 *
 * This program is licensed under the GPL version 2 or later.  Please see the
 * file COPYING in the source distribution of this software for license terms.
 */
#ifndef GPROB_H
#define GPROB_H

#include <math.h>

static inline double log_gprob(double delta, double variance)
{
    return -delta * delta / (2 * variance);
}

#endif /* GPROB_H */
