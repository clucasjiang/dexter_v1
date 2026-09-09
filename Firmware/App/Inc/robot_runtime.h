#ifndef ROBOT_RUNTIME_H
#define ROBOT_RUNTIME_H

#include <stdint.h>

typedef struct {
    uint32_t measured_hz;       /* Completed updates per second, last window. */
    uint32_t missed_periods;    /* Skipped 10 ms releases since startup. */
    uint32_t max_update_ms;     /* Includes motor exchanges and console work. */
} robot_runtime_stats_t;

/* Main-context only. Initialize after robot_app_init(), then poll continuously.
 * Late releases are skipped; blocking I/O can still exceed the target period.
 * Only a completed application update refreshes the hardware watchdog.
 */
void robot_runtime_init(void);
void robot_runtime_update(void);
const robot_runtime_stats_t *robot_runtime_stats(void);

#endif
