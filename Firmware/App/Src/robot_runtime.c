#include "robot_runtime.h"
#include "robot_app.h"
#include "robot_config.h"
#include "board.h"

_Static_assert(CONTROL_LOOP_HZ > 0U && CONTROL_LOOP_HZ <= 1000U &&
               1000U % CONTROL_LOOP_HZ == 0U, "Loop period must be whole milliseconds");

static robot_runtime_stats_t stats;
static uint32_t last_release_ms;
static uint32_t window_start_ms;
static uint32_t window_updates;

void robot_runtime_init(void)
{
    stats = (robot_runtime_stats_t){0};
    last_release_ms = board_millis();
    window_start_ms = last_release_ms;
    window_updates = 0U;
}

void robot_runtime_update(void)
{
    uint32_t start_ms = board_millis();
    uint32_t elapsed_ms = start_ms - last_release_ms;
    if (elapsed_ms < CONTROL_LOOP_PERIOD_MS) return;

    /* Keep the original phase, but never replay old control updates. Unsigned
     * subtraction also handles the HAL millisecond counter wrapping around.
     */
    uint32_t periods = elapsed_ms / CONTROL_LOOP_PERIOD_MS;
    stats.missed_periods += periods - 1U;
    last_release_ms += periods * CONTROL_LOOP_PERIOD_MS;

    robot_app_update();

    uint32_t end_ms = board_millis();
    uint32_t update_ms = end_ms - start_ms;
    if (update_ms > stats.max_update_ms) stats.max_update_ms = update_ms;
    ++window_updates;
    uint32_t window_ms = end_ms - window_start_ms;
    if (window_ms >= 1000U) {
        stats.measured_hz = (uint32_t)(((uint64_t)window_updates * 1000U +
                                      window_ms / 2U) / window_ms);
        window_start_ms = end_ms;
        window_updates = 0U;
    }

    /* Never feed from an interrupt, an I/O wait, or the idle polling path.
     * A stuck application or a stopped scheduling clock must time out.
     */
    board_watchdog_refresh();
}

const robot_runtime_stats_t *robot_runtime_stats(void)
{
    return &stats;
}
