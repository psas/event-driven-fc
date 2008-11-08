#ifndef INTERFACE_H
#define INTERFACE_H

#include <stdbool.h>

void report_state(enum state state);
void ignite(bool go);
void drogue_chute(bool go);
void main_chute(bool go);
void enqueue_error(const char *msg);

#endif /* INTERFACE_H */
