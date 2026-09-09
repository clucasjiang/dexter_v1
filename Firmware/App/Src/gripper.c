#include "gripper.h"
#include "robot_config.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static float clamp(float value, float low, float high)
{
    if (value < low) {
        return low;
    }
    if (value > high) {
        return high;
    }
    return value;
}

static void latch_fault(gripper_t *gripper, gripper_fault_t fault)
{
    gripper->state = GRIPPER_FAULT;
    gripper->fault = fault;
    gripper->integral_ma_s = 0.0f;
    gripper->fractional_step_deg = 0.0f;
}

static bool valid_config(const gripper_config_t *c)
{
    return isfinite(c->kp) && c->kp >= 0.0f &&
           isfinite(c->ki) && c->ki >= 0.0f &&
           isfinite(c->kd) && c->kd >= 0.0f &&
           isfinite(c->current_filter_s) && c->current_filter_s >= 0.0f &&
           isfinite(c->derivative_filter_s) && c->derivative_filter_s >= 0.0f &&
           isfinite(c->integral_limit_ma_s) && c->integral_limit_ma_s > 0.0f &&
           isfinite(c->current_ramp_ma_s) && c->current_ramp_ma_s > 0.0f &&
           isfinite(c->current_deadband_ma) && c->current_deadband_ma >= 0.0f &&
           isfinite(c->max_target_current_ma) && c->max_target_current_ma > 0.0f &&
           isfinite(c->hard_current_limit_ma) &&
           c->hard_current_limit_ma > c->max_target_current_ma &&
           isfinite(c->max_close_deg_s) && c->max_close_deg_s > 0.0f &&
           isfinite(c->max_open_deg_s) && c->max_open_deg_s > 0.0f &&
           isfinite(c->min_step_deg) && c->min_step_deg > 0.0f &&
           isfinite(c->max_step_deg) && c->max_step_deg >= c->min_step_deg &&
           isfinite(c->min_motor_deg) && isfinite(c->max_motor_deg) &&
           c->min_motor_deg < c->max_motor_deg &&
           c->command_period_ms > 0U &&
           c->feedback_timeout_ms >= c->command_period_ms &&
           c->feedback_timeout_ms <= 1000U;
}

gripper_config_t gripper_default_config(void)
{
    gripper_config_t config = {
        .kp = 0.1f,
        .ki = 0.025f,
        .kd = 0.001f,
        .current_filter_s = 0.025f,
        .derivative_filter_s = 0.050f,
        .integral_limit_ma_s = 40.0f,
        .current_ramp_ma_s = 40.0f,
        .current_deadband_ma = 3.0f,
        .max_target_current_ma = CLAW_MAX_TARGET_CURRENT_MA,
        .hard_current_limit_ma = CLAW_CURRENT_LIMIT_MA,
        .max_close_deg_s = 5.0f,
        .max_open_deg_s = 5.0f,
        .min_step_deg = 0.1f,
        .max_step_deg = 0.2f,
        .min_motor_deg = 0.0f,
        .max_motor_deg = CLAW_MAX_MOTOR_DEG,
        .command_period_ms = CLAW_FEEDBACK_PERIOD_MS,
        .feedback_timeout_ms = CLAW_FEEDBACK_TIMEOUT_MS
    };
    return config;
}

void gripper_init(gripper_t *gripper, const gripper_config_t *config)
{
    if (gripper == NULL) {
        return;
    }
    memset(gripper, 0, sizeof(*gripper));
    if (config == NULL || !valid_config(config)) {
        latch_fault(gripper, GRIPPER_FAULT_CONFIG);
        return;
    }
    gripper->config = *config;
    gripper->state = GRIPPER_IDLE;
}

bool gripper_start(gripper_t *gripper, float target_current_ma, uint32_t now_ms)
{
    if (gripper == NULL || gripper->state == GRIPPER_FAULT) {
        return false;
    }
    if (!isfinite(target_current_ma) || target_current_ma <= 0.0f ||
        target_current_ma > gripper->config.max_target_current_ma) {
        latch_fault(gripper, GRIPPER_FAULT_TARGET);
        return false;
    }
    gripper->target_current_ma = target_current_ma;
    gripper->setpoint_ma = 0.0f;
    gripper->filtered_current_ma = 0.0f;
    gripper->filtered_derivative_ma_s = 0.0f;
    gripper->integral_ma_s = 0.0f;
    gripper->fractional_step_deg = 0.0f;
    gripper->start_ms = now_ms;
    gripper->last_control_ms = now_ms;
    gripper->have_sample = false;
    gripper->state = GRIPPER_ACTIVE;
    return true;
}

void gripper_stop(gripper_t *gripper)
{
    if (gripper == NULL) {
        return;
    }
    if (gripper->state != GRIPPER_FAULT) {
        gripper->state = GRIPPER_IDLE;
    }
    gripper->integral_ma_s = 0.0f;
    gripper->fractional_step_deg = 0.0f;
    gripper->have_sample = false;
}

gripper_state_t gripper_update(gripper_t *gripper, float measured_current_ma,
                               uint32_t sample_ms, uint32_t now_ms,
                               float motor_deg, bool motor_moving,
                               float *delta_deg)
{
    const gripper_config_t *c;
    float dt_s;
    float old_current_ma;
    float derivative_ma_s;
    float error_ma;
    float trial_integral_ma_s;
    float speed_deg_s;
    float command_dt_s;
    float close_limit_deg_s;
    float open_limit_deg_s;
    float remaining_deg;
    float available_deg;
    float whole_steps;
    float delta;
    uint32_t sample_age_ms;
    uint32_t sample_dt_ms;

    if (delta_deg != NULL) {
        *delta_deg = 0.0f;
    }
    if (gripper == NULL) {
        return GRIPPER_FAULT;
    }
    if (gripper->state != GRIPPER_ACTIVE) {
        return gripper->state;
    }
    c = &gripper->config;

    /* These checks still run while a previously issued move is in progress. */
    if (!isfinite(measured_current_ma) || measured_current_ma < 0.0f) {
        latch_fault(gripper, GRIPPER_FAULT_CURRENT_VALUE);
        return gripper->state;
    }
    /* I check the unfiltered sample so the low-pass filter cannot hide a spike. */
    if (measured_current_ma >= c->hard_current_limit_ma) {
        latch_fault(gripper, GRIPPER_FAULT_CURRENT_LIMIT);
        return gripper->state;
    }
    sample_age_ms = now_ms - sample_ms;
    if (sample_age_ms > c->feedback_timeout_ms) {
        latch_fault(gripper, GRIPPER_FAULT_CURRENT_STALE);
        return gripper->state;
    }
    if (!isfinite(motor_deg)) {
        latch_fault(gripper, GRIPPER_FAULT_MOTOR_FEEDBACK);
        return gripper->state;
    }
    if (motor_deg < c->min_motor_deg || motor_deg > c->max_motor_deg) {
        latch_fault(gripper, GRIPPER_FAULT_TRAVEL_LIMIT);
        return gripper->state;
    }

    if (!gripper->have_sample) {
        /* Wait for a sample acquired since start, instead of using an old one. */
        if (sample_age_ms > now_ms - gripper->start_ms) {
            return gripper->state;
        }
        gripper->filtered_current_ma = measured_current_ma;
        gripper->setpoint_ma = clamp(measured_current_ma, 0.0f,
                                    gripper->target_current_ma);
        gripper->last_sample_ms = sample_ms;
        gripper->have_sample = true;
        return gripper->state;
    }

    sample_dt_ms = sample_ms - gripper->last_sample_ms;
    if (sample_dt_ms == 0U) {
        return gripper->state;
    }
    if (sample_dt_ms > c->feedback_timeout_ms) {
        latch_fault(gripper, GRIPPER_FAULT_CURRENT_STALE);
        return gripper->state;
    }
    gripper->last_sample_ms = sample_ms;
    dt_s = (float)sample_dt_ms * 0.001f;
    old_current_ma = gripper->filtered_current_ma;
    gripper->filtered_current_ma += dt_s / (c->current_filter_s + dt_s) *
                                 (measured_current_ma - old_current_ma);
    derivative_ma_s = (gripper->filtered_current_ma - old_current_ma) / dt_s;
    gripper->filtered_derivative_ma_s +=
        dt_s / (c->derivative_filter_s + dt_s) *
        (derivative_ma_s - gripper->filtered_derivative_ma_s);
    gripper->setpoint_ma = clamp(gripper->setpoint_ma + c->current_ramp_ma_s * dt_s,
                                0.0f, gripper->target_current_ma);

    /* Do not wind up the integral or queue more moves behind a busy motor. */
    if (motor_moving || delta_deg == NULL ||
        now_ms - gripper->last_control_ms < c->command_period_ms) {
        return gripper->state;
    }
    gripper->last_control_ms = now_ms;
    error_ma = gripper->setpoint_ma - gripper->filtered_current_ma;
    if (fabsf(error_ma) <= c->current_deadband_ma) {
        gripper->integral_ma_s = 0.0f;
        gripper->fractional_step_deg = 0.0f;
        return gripper->state;
    }

    command_dt_s = (float)c->command_period_ms * 0.001f;
    close_limit_deg_s = fminf(c->max_close_deg_s, c->max_step_deg / command_dt_s);
    open_limit_deg_s = fminf(c->max_open_deg_s, c->max_step_deg / command_dt_s);
    trial_integral_ma_s = clamp(gripper->integral_ma_s + error_ma * command_dt_s,
                               -c->integral_limit_ma_s, c->integral_limit_ma_s);
    /* Derivative on measured current avoids a kick when the target ramps up. */
    speed_deg_s = c->kp * error_ma + c->ki * trial_integral_ma_s -
                  c->kd * gripper->filtered_derivative_ma_s;
    if (!isfinite(speed_deg_s)) {
        latch_fault(gripper, GRIPPER_FAULT_CONFIG);
        return gripper->state;
    }
    /* Only integrate when unsaturated, or when the error unwinds saturation. */
    if ((speed_deg_s <= close_limit_deg_s || error_ma < 0.0f) &&
        (speed_deg_s >= -open_limit_deg_s || error_ma > 0.0f)) {
        gripper->integral_ma_s = trial_integral_ma_s;
    }
    speed_deg_s = c->kp * error_ma + c->ki * gripper->integral_ma_s -
                  c->kd * gripper->filtered_derivative_ma_s;
    speed_deg_s = clamp(speed_deg_s, -open_limit_deg_s, close_limit_deg_s);

    /* Always use one control period: a slow loop must never create a big step. */
    delta = clamp(speed_deg_s * command_dt_s, -c->max_step_deg, c->max_step_deg);
    if (delta == 0.0f) {
        return gripper->state;
    }
    remaining_deg = delta > 0.0f ? c->max_motor_deg - motor_deg :
                                     motor_deg - c->min_motor_deg;
    if (remaining_deg < c->min_step_deg) {
        latch_fault(gripper, GRIPPER_FAULT_TRAVEL_LIMIT);
        return gripper->state;
    }

    /* Small PID outputs take several cycles to earn one drive step. Rounding
       each tiny request up would turn a gentle approach into full speed. */
    if ((delta > 0.0f && gripper->fractional_step_deg < 0.0f) ||
        (delta < 0.0f && gripper->fractional_step_deg > 0.0f)) {
        gripper->fractional_step_deg = 0.0f;
    }
    gripper->fractional_step_deg += delta;
    available_deg = clamp(fabsf(gripper->fractional_step_deg), 0.0f,
                           fminf(c->max_step_deg, remaining_deg));
    whole_steps = floorf(available_deg / c->min_step_deg);
    *delta_deg = (delta > 0.0f ? 1.0f : -1.0f) * whole_steps * c->min_step_deg;
    gripper->fractional_step_deg -= *delta_deg;
    gripper->fractional_step_deg = clamp(gripper->fractional_step_deg,
                                         -c->min_step_deg, c->min_step_deg);
    return gripper->state;
}
