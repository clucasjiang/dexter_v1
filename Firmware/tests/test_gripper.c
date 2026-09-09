#include "gripper.h"

#include <assert.h>
#include <stdio.h>

static void test_close_and_open(void)
{
    gripper_t claw;
    gripper_config_t config = gripper_default_config();
    float step;
    float movement = 0.0f;

    gripper_init(&claw, &config);
    assert(gripper_start(&claw, 50.0f, 0));
    gripper_update(&claw, 0.0f, 0, 0, 30.0f, false, &step);

    // Low current should make the claw close a little.
    for (uint32_t time = 20; time <= 500; time += 20) {
        gripper_update(&claw, 0.0f, time, time, 30.0f, false, &step);
        assert(claw.state == GRIPPER_ACTIVE);
        assert(step >= 0.0f && step <= config.max_step_deg);
        movement += step;
    }
    assert(movement > 0.0f);

    // Current above the target should make it back off.
    movement = 0.0f;
    for (uint32_t time = 520; time <= 800; time += 20) {
        gripper_update(&claw, 120.0f, time, time, 30.0f, false, &step);
        assert(claw.state == GRIPPER_ACTIVE);
        assert(step <= 0.0f && step >= -config.max_step_deg);
        movement += step;
    }
    assert(movement < 0.0f);
}

static void test_current_limit(void)
{
    gripper_t claw;
    gripper_config_t config = gripper_default_config();
    float step;

    gripper_init(&claw, &config);
    assert(gripper_start(&claw, 50.0f, 0));

    // The cutoff should work even if the motor is still moving.
    gripper_update(&claw, config.hard_current_limit_ma, 20, 20,
                   30.0f, true, &step);
    assert(claw.state == GRIPPER_FAULT);
    assert(claw.fault == GRIPPER_FAULT_CURRENT_LIMIT);
    assert(step == 0.0f);
}

static void test_old_feedback(void)
{
    gripper_t claw;
    gripper_config_t config = gripper_default_config();
    float step;

    gripper_init(&claw, &config);
    assert(gripper_start(&claw, 50.0f, 0));
    gripper_update(&claw, 0.0f, 0, 0, 30.0f, false, &step);

    // Stop if no new motor reading arrives before the timeout.
    gripper_update(&claw, 0.0f, 0, config.feedback_timeout_ms + 1,
                   30.0f, false, &step);
    assert(claw.state == GRIPPER_FAULT);
    assert(claw.fault == GRIPPER_FAULT_CURRENT_STALE);
    assert(step == 0.0f);
}

int main(void)
{
    test_close_and_open();
    test_current_limit();
    test_old_feedback();
    puts("Claw tests passed");
    return 0;
}
