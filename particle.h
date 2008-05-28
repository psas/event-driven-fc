#ifndef PARTICLE_H
#define PARTICLE_H

#include "physics.h"

struct particle
{
	double weight;
	struct rocket_state s;
};

#endif /* PARTICLE_H */
