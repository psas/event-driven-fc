#ifndef FC_H
#define FC_H

enum state
{
	STATE_PREFLIGHT,      // until both given command to arm, and ready to arm
	STATE_ARMED,          // until launch command given and upward thrust detected
	STATE_BOOST,          // until burnout detected
	STATE_COAST,          // until apogee detected and drogue deployed
	STATE_DROGUE_DESCENT, // until z < 500m
	STATE_MAIN_DESCENT,   // until touchdown
	STATE_RECOVERY        // forever
};

void init(void);
void tick(double delta_t);
void arm(void);
void launch(void);
void z_accelerometer(double z_acc);
void pressure_sensor(double pressure);

#endif /* FC_H */
