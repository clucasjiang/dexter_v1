#include "robot_app.h"
#include "robot_runtime.h"
#include "arm.h"
#include "board.h"
#include "gripper.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static zdt_bus_t bus = { .exchange = board_motor_exchange, .timeout_ms = MOTOR_TIMEOUT_MS };
static arm_t arm;
static gripper_t claw;
static zdt_status_t claw_feedback;
static float claw_origin, claw_position, claw_target;
static bool claw_referenced, claw_pending;
static uint32_t claw_feedback_ms, claw_command_ms, claw_poll_ms;
static bool claw_feedback_valid;
static char line[96];
static unsigned line_length;
static bool discard_line;
static int demo_step = -1;

static void stop_claw(void)
{
    gripper_stop(&claw);
    claw_pending = false;
    if (!zdt_stop(&bus, CLAW_ADDRESS)) arm_fault_stop(&arm, ARM_FAULT_BUS, CLAW_ADDRESS);
}

static void fail_claw(void)
{
    gripper_stop(&claw);
    claw_pending = false;
    arm_fault_stop(&arm, ARM_FAULT_GRIPPER, CLAW_ADDRESS);
}

static bool read_claw(void)
{
    claw_feedback_valid = false;
    if (!zdt_read_status(&bus, CLAW_ADDRESS, &claw_feedback)) return false;
    /* Timestamp only a newly received reply; cached reads never refresh it. */
    claw_feedback_ms = board_millis();
    if (claw_referenced) {
        claw_position = (claw_feedback.position_deg - claw_origin) / CLAW_DIRECTION;
    }
    claw_feedback_valid = claw_feedback.encoder_ready && claw_feedback.calibrated &&
           !claw_feedback.stalled && !claw_feedback.stall_protected &&
           !claw_feedback.overcurrent && !claw_feedback.overtemperature &&
           !claw_feedback.homing && !claw_feedback.homing_failed &&
           (!claw_referenced || claw_feedback.power_marker);
    return claw_feedback_valid;
}

static bool claw_move(float target, uint32_t now)
{
    if (!arm.armed || !claw_referenced || !isfinite(target) || target < 0.0f ||
        target > CLAW_MAX_MOTOR_DEG || claw_pending || !claw_feedback_valid ||
        !claw_feedback.enabled || arm.fault != ARM_FAULT_NONE ||
        (uint32_t)(now - claw_feedback_ms) > CLAW_FEEDBACK_TIMEOUT_MS) return false;
    /* Absolute target avoids adding up small rounding errors in repeated moves. */
    if (!zdt_move_limited(&bus, CLAW_ADDRESS, claw_origin + CLAW_DIRECTION * target,
                         CLAW_MOVE_RPM, MOTOR_ACCEL_RPM_S, true, CLAW_CURRENT_LIMIT_MA)) {
        fail_claw();
        return false;
    }
    claw_target = target;
    claw_command_ms = now;
    claw_pending = true;
    return true;
}

static void update_claw(uint32_t now)
{
    if (!arm.armed || arm.fault != ARM_FAULT_NONE) return;
    if ((uint32_t)(now - claw_poll_ms) >= CLAW_FEEDBACK_PERIOD_MS) {
        claw_poll_ms = now;
        if (!read_claw() || !claw_feedback.enabled) { fail_claw(); return; }
        if (claw_position < -0.2f || claw_position > CLAW_MAX_MOTOR_DEG + 0.2f) {
            fail_claw(); return;
        }
        if (claw_pending && claw_feedback.reached && fabsf(claw_position - claw_target) < 0.06f &&
            fabsf(claw_feedback.rpm) < 0.5f) claw_pending = false;
    }
    now = board_millis();
    /* At contact a closing step may build the requested load without reaching
     * its endpoint. Cancel the remaining step once stationary at that load so
     * the next fresh sample can hold or release, instead of timing out or
     * waiting behind the contact indefinitely. Never cancel an OPEN this way.
     */
    if (claw_pending && claw.state == GRIPPER_ACTIVE && claw.have_sample &&
        claw_target > claw_position && claw.setpoint_ma > claw.config.current_deadband_ma &&
        claw_feedback_valid && (uint32_t)(now - claw_feedback_ms) <= CLAW_FEEDBACK_TIMEOUT_MS &&
        claw_feedback.phase_current_ma >= claw.setpoint_ma &&
        claw_feedback.phase_current_ma < claw.config.hard_current_limit_ma &&
        fabsf(claw_feedback.rpm) < 0.5f &&
        (uint32_t)(now - claw_command_ms) >= 2U * CLAW_FEEDBACK_PERIOD_MS) {
        if (!zdt_stop(&bus, CLAW_ADDRESS)) { fail_claw(); return; }
        claw_pending = false;
        /* The pre-stop sample cannot authorize another command. */
        if (!read_claw() || !claw_feedback.enabled) { fail_claw(); return; }
        now = board_millis();
    }
    if (claw_pending) {
        /* Opening may cover the full travel. PID steps are at most 0.2 degrees. */
        uint32_t timeout = claw.state == GRIPPER_ACTIVE ? 500U : 12000U;
        if ((uint32_t)(now - claw_command_ms) > timeout) { fail_claw(); return; }
    }
    if (claw.state != GRIPPER_ACTIVE) return;
    if (!claw_feedback_valid ||
        (uint32_t)(now - claw_feedback_ms) > CLAW_FEEDBACK_TIMEOUT_MS) { fail_claw(); return; }
    float delta = 0.0f;
    gripper_state_t state = gripper_update(&claw, (float)claw_feedback.phase_current_ma,
                                          claw_feedback_ms, now, claw_position,
                                          claw_pending || fabsf(claw_feedback.rpm) > 0.5f, &delta);
    if (state == GRIPPER_FAULT) { fail_claw(); return; }
    if (delta != 0.0f && !claw_move(claw_position + delta, now)) fail_claw();
}

static void print_status(void)
{
    char text[180];
    snprintf(text, sizeof(text), "armed=%u zero=%u fault=%s id=%u stop_unconfirmed=%u\r\n",
             arm.armed, arm.referenced, arm_fault_name(arm.fault), arm.fault_address, arm.stop_unconfirmed);
    board_console_write(text);
    const robot_runtime_stats_t *timing = robot_runtime_stats();
    snprintf(text, sizeof(text), "loop_target=%uHz loop_actual=%luHz missed=%lu max_update=%lums watchdog=%ums watchdog_reset=%u\r\n",
             CONTROL_LOOP_HZ, (unsigned long)timing->measured_hz,
             (unsigned long)timing->missed_periods, (unsigned long)timing->max_update_ms,
             BOARD_WATCHDOG_TIMEOUT_MS, board_watchdog_was_reset());
    board_console_write(text);
    for (unsigned i = 0; i < ARM_JOINT_COUNT; ++i) {
        const joint_state_t *j = &arm.joint[i];
        snprintf(text, sizeof(text), "J%u angle=%.2f target=%.2f moving=%u valid=%u current=%umA\r\n",
                 i + 1U, (double)j->angle_deg, (double)j->target_deg, j->moving, j->valid,
                 j->feedback.phase_current_ma);
        board_console_write(text);
    }
    snprintf(text, sizeof(text), "claw=%.2f current=%umA target_current=%.1fmA feedback_valid=%u grip=%u fault=%u feedback_age=%lums\r\n",
             (double)claw_position, claw_feedback.phase_current_ma,
             (double)claw.target_current_ma, claw_feedback_valid,
             (unsigned)claw.state, (unsigned)claw.fault,
             (unsigned long)(board_millis() - claw_feedback_ms));
    board_console_write(text);
}

static bool number(const char *text, float *value)
{
    char *end;
    if (text == NULL || *text == '\0') return false;
    *value = strtof(text, &end);
    return *end == '\0' && isfinite(*value);
}

static void command(char *text)
{
    char *argv[5];
    unsigned argc = 0;
    for (char *part = strtok(text, " \t"); part != NULL; part = strtok(NULL, " \t")) {
        if (argc == 5) { board_console_write("ERR too many arguments\r\n"); return; }
        argv[argc++] = part;
    }
    if (argc == 0) return;
    uint32_t now = board_millis();
    bool ok = false;
    float a, b, c;
    if (strcmp(argv[0], "HELP") == 0 && argc == 1) {
        board_console_write("STATUS | ZERO | ARM | MOVE joint degrees deg/s | STOP [joint]\r\n"
                            "DISARM | CLEAR | GRIP milliamps | OPEN | DEMO\r\n"
                            "ZERO: put all joints at known zero and claw fully open first.\r\n"
                            "GRIP uses motor phase current in mA. Zero is lost on reset.\r\n");
        return;
    }
    if (strcmp(argv[0], "STATUS") == 0 && argc == 1) { print_status(); return; }
    if (strcmp(argv[0], "STOP") == 0 && argc == 1) {
        demo_step = -1;
        gripper_stop(&claw);
        claw_pending = false;
        ok = arm_stop_all(&arm);
        if (!ok) arm_fault_stop(&arm, ARM_FAULT_BUS, 0);
    } else if (strcmp(argv[0], "STOP") == 0 && argc == 2 && number(argv[1], &a) &&
               a >= 1.0f && a <= 7.0f && a == floorf(a)) {
        demo_step = -1;
        if (a == 7.0f) { stop_claw(); ok = arm.fault == ARM_FAULT_NONE; }
        else { ok = arm_stop_joint(&arm, (uint8_t)a); if (!ok) arm_fault_stop(&arm, ARM_FAULT_BUS, (uint8_t)a); }
    } else if (board_estop_active()) {
        arm_fault_stop(&arm, ARM_FAULT_ESTOP, 0);
    } else if (strcmp(argv[0], "DISARM") == 0 && argc == 1) {
        demo_step = -1;
        gripper_stop(&claw);
        claw_pending = false;
        ok = arm_stop_all(&arm);
        arm.armed = false;
        if (!ok) arm_fault_stop(&arm, ARM_FAULT_BUS, 0);
    } else if (strcmp(argv[0], "CLEAR") == 0 && argc == 1 && !arm.armed) {
        ok = arm_clear_fault(&arm);
        if (ok) {
            gripper_config_t config = gripper_default_config();
            gripper_init(&claw, &config);
            claw_pending = claw_referenced = claw_feedback_valid = false;
        }
    } else if (strcmp(argv[0], "ZERO") == 0 && argc == 1) {
        ok = arm_reference_here(&arm, now);
        if (ok) {
            claw_referenced = false;
            ok = zdt_check_configuration(&bus, CLAW_ADDRESS) &&
                 zdt_set_power_marker(&bus, CLAW_ADDRESS) && read_claw() &&
                 claw_feedback.power_marker;
            if (ok) {
                claw_origin = claw_feedback.position_deg;
                claw_position = 0.0f;
                claw_referenced = true;
            } else arm_fault_stop(&arm, ARM_FAULT_DRIVER, CLAW_ADDRESS);
        }
    } else if (strcmp(argv[0], "ARM") == 0 && argc == 1 && claw_referenced) {
        ok = arm_enable(&arm, now);
        if (ok) {
            ok = zdt_stop(&bus, CLAW_ADDRESS) && zdt_enable(&bus, CLAW_ADDRESS, true) &&
                 read_claw() && claw_feedback.enabled;
            if (!ok) fail_claw();
        }
    } else if (strcmp(argv[0], "MOVE") == 0 && argc == 4 && demo_step < 0 &&
               number(argv[1], &a) && number(argv[2], &b) && number(argv[3], &c) &&
               a >= 1.0f && a <= 6.0f && a == floorf(a)) {
        ok = arm_move_joint(&arm, (uint8_t)a, b, c, now);
    } else if (strcmp(argv[0], "GRIP") == 0 && argc == 2 && number(argv[1], &a) &&
               arm.armed && claw_referenced && !claw_pending && claw_feedback_valid &&
               (uint32_t)(now - claw_feedback_ms) <= CLAW_FEEDBACK_TIMEOUT_MS) {
        ok = gripper_start(&claw, a, now);
        if (!ok && claw.state == GRIPPER_FAULT) fail_claw();
    } else if (strcmp(argv[0], "OPEN") == 0 && argc == 1 && arm.armed) {
        stop_claw();
        if (arm.fault == ARM_FAULT_NONE && read_claw()) ok = claw_move(0.0f, board_millis());
    } else if (strcmp(argv[0], "DEMO") == 0 && argc == 1 && arm.armed && arm_is_idle(&arm) &&
               claw.state == GRIPPER_IDLE && !claw_pending) {
        ok = true;
        for (unsigned i = 0; i < ARM_JOINT_COUNT; ++i) {
            if (fabsf(arm.joint[i].angle_deg) > 0.5f) ok = false;
        }
        if (ok) demo_step = 0;
    }
    board_console_write(ok ? "OK\r\n" : "ERR rejected; check HELP and STATUS\r\n");
}

static void console_update(void)
{
    char ch;
    /* At most one line per loop, so a pasted script cannot starve monitoring. */
    for (unsigned n = 0; n < sizeof(line) && board_console_read(&ch); ++n) {
        if (ch == 0x18) { discard_line = true; line_length = 0; continue; }
        if (ch == '\n' || ch == '\r') {
            if (!discard_line) { line[line_length] = '\0'; command(line); }
            else board_console_write("ERR damaged or long line discarded\r\n");
            line_length = 0;
            discard_line = false;
            return;
        }
        if (discard_line) continue;
        if (ch < 32 || ch > 126 || line_length >= sizeof(line) - 1U) {
            line_length = 0;
            discard_line = true;
        } else line[line_length++] = ch;
    }
}

void robot_app_init(void)
{
    claw_referenced = claw_pending = claw_feedback_valid = false;
    claw_origin = claw_position = claw_target = 0.0f;
    claw_feedback_ms = claw_command_ms = claw_poll_ms = 0U;
    memset(&claw_feedback, 0, sizeof(claw_feedback));
    line_length = 0U;
    discard_line = false;
    demo_step = -1;
    arm_init(&arm, &bus);
    gripper_config_t config = gripper_default_config();
    gripper_init(&claw, &config);
    /* Reset never starts a demo or assumes the encoder's startup zero is home. */
    if (!arm_stop_all(&arm)) arm_fault_stop(&arm, ARM_FAULT_BUS, 0);
    if (board_watchdog_was_reset()) {
        board_console_write("Watchdog reset detected; reference and arming required.\r\n");
    }
    board_console_write("Dexter ready, motion disarmed. Type HELP.\r\n");
}

void robot_app_update(void)
{
    if (board_estop_active()) arm_fault_stop(&arm, ARM_FAULT_ESTOP, 0);
    update_claw(board_millis());
    arm_poll(&arm, board_millis(), board_estop_active());
    if (arm.fault != ARM_FAULT_NONE) {
        demo_step = -1;
        claw_referenced = false;
        claw_pending = false;
        claw_feedback_valid = false;
        gripper_stop(&claw);
    }
    console_update();
    if (demo_step >= 0 && arm.armed && arm_is_idle(&arm)) {
        if (demo_step >= 12) {
            demo_step = -1;
            board_console_write("OK demo complete\r\n");
        } else {
            uint8_t joint = (uint8_t)(demo_step / 2 + 1);
            float target = (demo_step % 2 == 0) ? 5.0f : 0.0f;
            if (arm_move_joint(&arm, joint, target, 2.0f, board_millis())) ++demo_step;
            else { demo_step = -1; board_console_write("ERR demo move rejected\r\n"); }
        }
    }
}
