#ifndef SIM_COMMON_H
#define SIM_COMMON_H

#include <stdbool.h>
#include "compiler.h"

extern geodetic initial_geodetic;
void parse_trace_args(int argc, const char *const argv[]);

enum state last_reported_state(void);
void trace_printf(const char *fmt, ...) ATTR_FORMAT(printf,1,2);

double current_timestamp(void);

#endif /* SIM_COMMON_H */
