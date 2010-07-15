/* Copyright © 2010 Portland State Aerospace Society
 * See version control history for detailed authorship information.
 *
 * This program is licensed under the GPL version 2 or later.  Please see the
 * file COPYING in the source distribution of this software for license terms.
 */
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
