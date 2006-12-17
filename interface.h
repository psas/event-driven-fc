#include <stdbool.h>

void report_state(enum state state);
void ignite(bool go);
void drogue_chute(bool go);
void main_chute(bool go);
void enqueue_error(char *msg);
