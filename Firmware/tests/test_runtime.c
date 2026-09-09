#include "robot_runtime.h"
#include "robot_app.h"
#include "board.h"

#include <assert.h>
#include <stdbool.h>
#include <stdio.h>

static uint32_t now_ms;
static uint32_t work_ms;
static unsigned updates;
static unsigned feeds;
static bool updating;

uint32_t board_millis(void)
{
    return now_ms;
}

void robot_app_update(void)
{
    updating = true;
    assert(feeds == updates); /* No feed before this update completes. */
    ++updates;
    now_ms += work_ms;
    updating = false;
}

void board_watchdog_refresh(void)
{
    assert(!updating);
    assert(updates == feeds + 1U);
    ++feeds;
}

static void reset(uint32_t start_ms)
{
    now_ms = start_ms;
    work_ms = 0U;
    updates = feeds = 0U;
    updating = false;
    robot_runtime_init();
}

static void test_100hz_and_idle_watchdog(void)
{
    reset(0U);
    for (now_ms = 0U; now_ms <= 1000U; ++now_ms) {
        robot_runtime_update();
        assert(updates == now_ms / 10U);
        assert(feeds == updates);
    }
    assert(robot_runtime_stats()->measured_hz == 100U);
    assert(robot_runtime_stats()->missed_periods == 0U);
    /* A stopped HAL tick must not keep the watchdog alive. */
    for (unsigned n = 0U; n < 10000U; ++n) robot_runtime_update();
    assert(feeds == 100U);
}

static void test_work_does_not_add_to_period(void)
{
    reset(0U);
    work_ms = 4U;
    now_ms = 10U;
    robot_runtime_update();
    assert(now_ms == 14U && updates == 1U);
    now_ms = 19U;
    robot_runtime_update();
    assert(updates == 1U);
    now_ms = 20U;
    robot_runtime_update();
    assert(updates == 2U);
    assert(robot_runtime_stats()->max_update_ms == 4U);
}

static void test_late_update_skips_old_releases(void)
{
    reset(100U);
    now_ms = 110U;
    work_ms = 35U;
    robot_runtime_update();
    assert(now_ms == 145U && feeds == 1U);
    work_ms = 0U;
    robot_runtime_update();
    assert(updates == 2U);
    assert(robot_runtime_stats()->missed_periods == 2U);
    assert(robot_runtime_stats()->max_update_ms == 35U);
    robot_runtime_update();
    assert(updates == 2U); /* No catch-up burst or duplicate watchdog feed. */
    now_ms = 150U;
    robot_runtime_update();
    assert(updates == 3U);
}

static void test_slow_bus_rate_is_measured(void)
{
    reset(0U);
    now_ms = 10U;
    work_ms = 25U;
    for (unsigned n = 0U; n < 40U; ++n) robot_runtime_update();
    assert(robot_runtime_stats()->measured_hz == 40U);
    assert(robot_runtime_stats()->missed_periods > 0U);
    assert(feeds == 40U);
}

static void test_clock_wrap(void)
{
    reset(UINT32_MAX - 4U);
    now_ms = 4U;
    robot_runtime_update();
    assert(updates == 0U);
    now_ms = 5U;
    work_ms = 3U;
    robot_runtime_update();
    assert(updates == 1U && now_ms == 8U);
    robot_runtime_update();
    assert(updates == 1U);
    now_ms = 15U;
    robot_runtime_update();
    assert(updates == 2U);
    assert(robot_runtime_stats()->missed_periods == 0U);
    assert(robot_runtime_stats()->max_update_ms == 3U);
}

int main(void)
{
    test_100hz_and_idle_watchdog();
    test_work_does_not_add_to_period();
    test_late_update_skips_old_releases();
    test_slow_bus_rate_is_measured();
    test_clock_wrap();
    puts("runtime tests passed");
    return 0;
}
