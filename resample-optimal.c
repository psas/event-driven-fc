/*
 * Derived from resample/optimal.c in bmpf:
 * Copyright (C) 2007 Bart Massey
 * ALL RIGHTS RESERVED
 * 
 * This program is licensed under the GPL version 2 or GPL
 * version 3 or later.  Please see the file COPYING in the
 * source distribution of this software for license terms.
 */

#include "particle.h"
#include "resample.h"
#include "ziggurat/random.h"

int resample_optimal(double scale,
		     int m, struct particle *particle,
		     int n, struct particle *newp)
{
    double invscale = 1.0 / scale;
    double u0 = polynomial(n - 1) * scale;
    int i, j = 0;
    double t = 0;
    double best_w = 0;
    int best_i = 0;
    for (i = 0; i < n; i++) {
        while (t + particle[j].weight < u0 && j < m)
	    t += particle[j++].weight;
	newp[i] = particle[j];
	newp[i].weight *= invscale;
        if (newp[i].weight > best_w) {
	    best_w = newp[i].weight;
	    best_i = i;
	}
	u0 = u0 + (scale - u0) * polynomial(n - i - 1);
	newp[i].weight = 1;
    }
    return best_i;
}
