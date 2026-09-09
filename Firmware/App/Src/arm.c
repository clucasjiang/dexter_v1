#include "arm.h"
#include <math.h>
#include <string.h>

/* J2/J3/J5 are the gravity-loaded roll axes. */
const joint_config_t joint_config[ARM_JOINT_COUNT] = {
    {1, 42, 0, 50.0f, 1.0f, -160.0f, 160.0f, 12.0f, 1.0f},
    {2, 42, 1, 50.0f, 1.0f,  -70.0f,  70.0f,  8.0f, 1.0f},
    {3, 42, 1, 50.0f, 1.0f,  -90.0f,  90.0f,  8.0f, 1.0f},
    {4, 35, 0, 30.0f, 1.0f, -160.0f, 160.0f, 12.0f, 1.5f},
    {5, 35, 1, 30.0f, 1.0f,  -90.0f,  90.0f,  8.0f, 1.5f},
    {6, 35, 0, 30.0f, 1.0f, -160.0f, 160.0f, 12.0f, 1.5f}
};

static bool driver_ok(const zdt_status_t *s)
{
    return s->encoder_ready && s->calibrated && !s->stalled &&
           !s->stall_protected && !s->overtemperature && !s->overcurrent;
}

void arm_init(arm_t *arm, zdt_bus_t *bus)
{
    memset(arm, 0, sizeof(*arm));
    arm->bus = bus;
}

bool arm_is_idle(const arm_t *arm)
{
    for (unsigned i = 0; i < ARM_JOINT_COUNT; ++i) {
        if (arm->joint[i].moving) return false;
    }
    return true;
}

bool arm_stop_joint(arm_t *arm, uint8_t joint)
{
    if (joint < 1 || joint > ARM_JOINT_COUNT) return false;
    bool ok = zdt_stop(arm->bus, joint_config[joint - 1].address);
    arm->joint[joint - 1].moving = false;
    arm->joint[joint - 1].error_pending = false;
    if (!ok) arm->stop_unconfirmed = true;
    return ok;
}

bool arm_stop_all(arm_t *arm)
{
    bool ok = true;
    for (uint8_t j = 1; j <= ARM_JOINT_COUNT; ++j) {
        if (!arm_stop_joint(arm, j)) ok = false;
    }
    if (!zdt_stop(arm->bus, CLAW_ADDRESS)) ok = false;
    if (!ok) arm->stop_unconfirmed = true;
    /* I leave holding torque on: disabling J2/J3/J5 can drop the arm. */
    return ok;
}

void arm_fault_stop(arm_t *arm, arm_fault_t fault, uint8_t address)
{
    if (arm->fault == ARM_FAULT_NONE) {
        arm->fault = fault;
        arm->fault_address = address;
        arm->armed = false;
        arm->referenced = false;
        (void)arm_stop_all(arm);
    }
}

bool arm_clear_fault(arm_t *arm)
{
    /* Clearing the software latch never clears protection inside a motor. */
    if (!arm_stop_all(arm)) return false;
    for (uint8_t id = 1; id <= ROBOT_MOTOR_COUNT; ++id) {
        zdt_status_t status;
        if (!zdt_read_status(arm->bus, id, &status) || !driver_ok(&status)) return false;
    }
    arm->fault = ARM_FAULT_NONE;
    arm->fault_address = 0;
    arm->stop_unconfirmed = false;
    arm->referenced = false;
    arm->armed = false;
    return true;
}

bool arm_reference_here(arm_t *arm, uint32_t now_ms)
{
    if (arm->armed || arm->fault != ARM_FAULT_NONE || !arm_is_idle(arm)) return false;
    arm->referenced = false;
    if (!arm_stop_all(arm)) {
        arm_fault_stop(arm, ARM_FAULT_BUS, 0);
        return false;
    }
    for (unsigned i = 0; i < ARM_JOINT_COUNT; ++i) {
        joint_state_t *j = &arm->joint[i];
        uint8_t id = joint_config[i].address;
        if (!zdt_check_configuration(arm->bus, id) ||
            !zdt_read_status(arm->bus, id, &j->feedback) || !driver_ok(&j->feedback)) {
            arm_fault_stop(arm, ARM_FAULT_DRIVER, id);
            return false;
        }
        j->origin_motor_deg = j->feedback.position_deg;
        j->angle_deg = j->target_deg = j->progress_deg = 0.0f;
        j->feedback_ms = now_ms;
        j->valid = true;
    }
    arm->referenced = true;
    return true;
}

bool arm_enable(arm_t *arm, uint32_t now_ms)
{
    if (!arm->referenced || arm->fault != ARM_FAULT_NONE || arm->armed) return false;
    for (unsigned i = 0; i < ARM_JOINT_COUNT; ++i) {
        uint8_t id = joint_config[i].address;
        if (!zdt_stop(arm->bus, id) || !zdt_enable(arm->bus, id, true) ||
            !zdt_read_status(arm->bus, id, &arm->joint[i].feedback) ||
            !driver_ok(&arm->joint[i].feedback) || !arm->joint[i].feedback.enabled) {
            arm_fault_stop(arm, ARM_FAULT_DRIVER, id);
            return false;
        }
        arm->joint[i].feedback_ms = now_ms;
    }
    arm->armed = true;
    return true;
}

bool arm_move_joint(arm_t *arm, uint8_t joint, float degrees,
                    float speed_deg_s, uint32_t now_ms)
{
    if (joint < 1 || joint > ARM_JOINT_COUNT || !arm->armed ||
        arm->fault != ARM_FAULT_NONE || !arm->referenced ||
        !isfinite(degrees) || !isfinite(speed_deg_s)) return false;
    unsigned i = joint - 1;
    const joint_config_t *c = &joint_config[i];
    joint_state_t *j = &arm->joint[i];
    if (degrees < c->min_deg || degrees > c->max_deg || speed_deg_s < 0.2f ||
        speed_deg_s > c->max_speed_deg_s || j->moving || !j->valid ||
        (uint32_t)(now_ms - j->feedback_ms) > FEEDBACK_MAX_AGE_MS) return false;
    float motor_deg = j->origin_motor_deg + c->direction * degrees * c->reduction;
    float rpm = speed_deg_s * c->reduction / 6.0f;
    if (!zdt_move(arm->bus, c->address, motor_deg, rpm, MOTOR_ACCEL_RPM_S, true)) {
        arm_fault_stop(arm, ARM_FAULT_BUS, c->address);
        return false;
    }
    j->target_deg = degrees;
    j->started_ms = j->progress_ms = now_ms;
    j->progress_deg = j->angle_deg;
    j->error_pending = false;
    j->deadline_ms = (uint32_t)(fabsf(degrees - j->angle_deg) / speed_deg_s * 2000.0f) + 3000U;
    j->moving = true;
    return true;
}

void arm_poll(arm_t *arm, uint32_t now_ms, bool estop)
{
    if (estop) {
        arm_fault_stop(arm, ARM_FAULT_ESTOP, 0);
        return;
    }
    if (arm->fault != ARM_FAULT_NONE) return;
    unsigned i = arm->poll_joint;
    arm->poll_joint = (uint8_t)((i + 1U) % ARM_JOINT_COUNT);
    joint_state_t *j = &arm->joint[i];
    const joint_config_t *c = &joint_config[i];
    if (!zdt_read_status(arm->bus, c->address, &j->feedback)) {
        j->valid = false;
        if (arm->armed) arm_fault_stop(arm, ARM_FAULT_BUS, c->address);
        return;
    }
    j->valid = true;
    j->feedback_ms = now_ms;
    if (!arm->referenced) return;
    j->angle_deg = (j->feedback.position_deg - j->origin_motor_deg) /
                   (c->direction * c->reduction);
    if (!arm->armed) return;
    if (!driver_ok(&j->feedback) || !j->feedback.enabled) {
        arm_fault_stop(arm, ARM_FAULT_DRIVER, c->address);
        return;
    }
    if (j->angle_deg < c->min_deg - 0.5f || j->angle_deg > c->max_deg + 0.5f) {
        arm_fault_stop(arm, ARM_FAULT_LIMIT, c->address);
        return;
    }
    for (unsigned n = 0; n < ARM_JOINT_COUNT; ++n) {
        if ((uint32_t)(now_ms - arm->joint[n].feedback_ms) > FEEDBACK_MAX_AGE_MS) {
            arm_fault_stop(arm, ARM_FAULT_STALE, joint_config[n].address);
            return;
        }
    }
    /* The drive reports its trajectory error, not distance to the final target. */
    if (fabsf(j->feedback.error_deg) / c->reduction > c->max_following_error_deg) {
        if (!j->error_pending) { j->error_pending = true; j->error_ms = now_ms; }
        if ((uint32_t)(now_ms - j->error_ms) >= FOLLOWING_ERROR_TIME_MS) {
            arm_fault_stop(arm, ARM_FAULT_BLOCKED, c->address);
            return;
        }
    } else j->error_pending = false;
    if (!j->moving) return;
    if (fabsf(j->angle_deg - j->target_deg) < 0.2f && fabsf(j->feedback.rpm) < 0.5f) {
        j->moving = false;
        return;
    }
    if (fabsf(j->angle_deg - j->progress_deg) >= 0.05f) {
        j->progress_deg = j->angle_deg;
        j->progress_ms = now_ms;
    }
    if ((uint32_t)(now_ms - j->progress_ms) > NO_PROGRESS_TIME_MS ||
        (uint32_t)(now_ms - j->started_ms) > j->deadline_ms) {
        arm_fault_stop(arm, ARM_FAULT_BLOCKED, c->address);
    }
}

const char *arm_fault_name(arm_fault_t fault)
{
    switch (fault) {
    case ARM_FAULT_NONE: return "none";
    case ARM_FAULT_ESTOP: return "estop";
    case ARM_FAULT_BUS: return "bus";
    case ARM_FAULT_DRIVER: return "driver/config";
    case ARM_FAULT_BLOCKED: return "blocked";
    case ARM_FAULT_LIMIT: return "joint limit";
    case ARM_FAULT_STALE: return "stale feedback";
    case ARM_FAULT_GRIPPER: return "gripper";
    default: return "unknown";
    }
}
