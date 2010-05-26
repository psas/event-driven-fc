/*
 * Copyright (C) 2007 Bart Massey
 * ALL RIGHTS RESERVED
 *
 * This program is licensed under the GPL version 2 or GPL
 * version 3 or later.  Please see the file COPYING in the
 * source distribution of this software for license terms.
 */
#ifndef GPROB_H
#define GPROB_H

#include <math.h>

static inline double gprob(double delta, double variance)
{
    return exp(-delta * delta / (2 * variance));
}

#endif /* GPROB_H */
