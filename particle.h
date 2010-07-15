/* Copyright © 2010 Portland State Aerospace Society
 * See version control history for detailed authorship information.
 *
 * This program is licensed under the GPL version 2 or later.  Please see the
 * file COPYING in the source distribution of this software for license terms.
 */
#ifndef PARTICLE_H
#define PARTICLE_H

#include "physics.h"

struct particle
{
	double weight;
	struct rocket_state s;
};

#endif /* PARTICLE_H */
