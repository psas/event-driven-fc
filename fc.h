#include <stdint.h>

enum state {
	STATE_PREFLIGHT,      // until both given command to arm, and ready to arm
	STATE_ARMED,          // until launch command given and upward thrust detected
	STATE_BOOST,          // until burnout detected
	STATE_COAST,          // until apogee detected and drogue deployed
	STATE_DROGUE_DESCENT, // until z < 500m
	STATE_MAIN_DESCENT,   // until touchdown
	STATE_RECOVERY        // forever
};

enum { X, Y, Z };

typedef double vec3[3];
typedef uint64_t microseconds;

void arm(void);
void launch(void);
void omniscience_9000(microseconds timestamp,
                      vec3 pos, vec3 vel, vec3 acc,
                      vec3 rotpos, vec3 rotvel);
