/*
 * Derived from resample/regular.c in bmpf:
 * Copyright (C) 2007 Bart Massey
 * ALL RIGHTS RESERVED
 *
 * This program is licensed under the GPL version 2 or GPL
 * version 3 or later.  Please see the file COPYING in the
 * source distribution of this software for license terms.
 */

#include <stdio.h>

#include "particle.h"
#include "resample.h"
#include "ziggurat/zrandom.h"

void resample_regular(int m, struct particle *particle,
                      int n, struct particle *newp,
                      int sort)
{
	int i, j;
	double u0, t = 0;
	if (sort)
	{
		/* shuffle */
		for (i = 0; i < m - 1; i++)
		{
			j = rand32() % (m - i) + i;
			struct particle ptmp = particle[j];
			particle[j] = particle[i];
			particle[i] = ptmp;
		}
	}
	/* merge */
	j = 0;
	u0 = uniform() * 1.0 / (n + 1);
	for (i = 0; i < n; i++ )
	{
		for (;j < m; j++)
		{
			double w = exp(particle[j].weight);
			if (t + w >= u0)
				break;
			t += w;
		}
		if (j >= m)
		{
			fprintf(stderr, "fell off end t=%.14g u0=%.14g\n", t, u0);
			abort();
		}
		newp[i] = particle[j];
		newp[i].weight = -log(n);
		u0 += 1.0 / (n + 1);
	}
}
