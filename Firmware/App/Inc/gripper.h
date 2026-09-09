#ifndef GRIPPER_H
#define GRIPPER_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    GRIPPER_IDLE = 0,
    GRIPPER_ACTIVE,
    GRIPPER_FAULT
} gripper_state_t;

typedef enum {
    GRIPPER_FAULT_NONE = 0,
    GRIPPER_FAULT_CONFIG,
    GRIPPER_FAULT_TARGET,
    GRIPPER_FAULT_CURRENT_STALE,
    GRIPPER_FAULT_CURRENT_VALUE,
    GRIPPER_FAULT_CURRENT_LIMIT,
    GRIPPER_FAULT_MOTOR_FEEDBACK,
    GRIPPER_FAULT_TRAVEL_LIMIT
} gripper_fault_t;

typedef struct {
    /* PID output is motor degrees/second. These gains need bench tuning. */
    float kp;                       /* (degrees/second) / mA */
    float ki;                       /* (degrees/second) / (mA * second) */
    float kd;                       /* degrees / mA */
    float current_filter_s;
    float derivative_filter_s;
    float integral_limit_ma_s;
    float current_ramp_ma_s;
    float current_deadband_ma;
    float max_target_current_ma;
    float hard_current_limit_ma;
    float max_close_deg_s;
    float max_open_deg_s;
    float min_step_deg;              /* Drive angle resolution: 0.1 degree. */
    float max_step_deg;
    /* Motor coordinates increase as the claw closes, after applying polarity. */
    float min_motor_deg;
    float max_motor_deg;
    uint32_t command_period_ms;
    uint32_t feedback_timeout_ms;
} gripper_config_t;

typedef struct {
    gripper_config_t config;
    gripper_state_t state;
    gripper_fault_t fault;
    float target_current_ma;
    float setpoint_ma;
    float filtered_current_ma;
    float filtered_derivative_ma_s;
    float integral_ma_s;
    float fractional_step_deg;
    uint32_t start_ms;
    uint32_t last_sample_ms;
    uint32_t last_control_ms;
    bool have_sample;
} gripper_t;

/* Untuned bench defaults: phase current is a grip-load proxy, not newtons. */
gripper_config_t gripper_default_config(void);
void gripper_init(gripper_t *gripper, const gripper_config_t *config);
bool gripper_start(gripper_t *gripper, float target_current_ma, uint32_t now_ms);
/* Stop clears the controller but preserves a latched fault. Reinitialize to reset. */
void gripper_stop(gripper_t *gripper);

/*
 * measured_current_ma is motor 7's nonnegative phase-current telemetry in mA.
 * sample_ms timestamps a successful status reply, never a cached reread.
 * The drive provides no internal sample timestamp; bus freshness is all we know.
 * motor_moving MUST include queued commands and unconfirmed motor completion.
 * delta_deg is a bounded RELATIVE motor move, positive to close, negative to open.
 * A zero delta means no new command. Dispatch failures require an immediate stop.
 * On GRIPPER_FAULT the caller must issue the motor stop command immediately.
 * This controller does not replace the drive's own stall/current protections.
 */
gripper_state_t gripper_update(gripper_t *gripper, float measured_current_ma,
                               uint32_t sample_ms, uint32_t now_ms,
                               float motor_deg, bool motor_moving,
                               float *delta_deg);

#endif
