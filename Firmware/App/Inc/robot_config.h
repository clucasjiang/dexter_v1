#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include <stdint.h>

#define ARM_JOINT_COUNT 6U
#define ROBOT_MOTOR_COUNT 7U
#define CONTROL_LOOP_HZ 100U
#define CONTROL_LOOP_PERIOD_MS (1000U / CONTROL_LOOP_HZ)
#define CLAW_ADDRESS 7U
#define MOTOR_TIMEOUT_MS 10U
#define FEEDBACK_MAX_AGE_MS 150U
#define FOLLOWING_ERROR_TIME_MS 120U
#define NO_PROGRESS_TIME_MS 700U
#define MOTOR_ACCEL_RPM_S 600U
/* Tested limits for the current stepper-driven arm and egg-grasp demo. */
#define CLAW_CURRENT_LIMIT_MA 150U
#define CLAW_MAX_TARGET_CURRENT_MA 100.0f
#define CLAW_FEEDBACK_PERIOD_MS 20U
#define CLAW_FEEDBACK_TIMEOUT_MS 100U
#define CLAW_MOVE_RPM 3.0f
#define CLAW_DIRECTION 1.0f
#define CLAW_MAX_MOTOR_DEG 180.0f

/* Installed mechanism configuration. */
typedef struct {
    uint8_t address;
    uint8_t motor_size_mm;
    uint8_t carries_gravity;
    float reduction;
    float direction;
    float min_deg;
    float max_deg;
    float max_speed_deg_s;
    float max_following_error_deg;
} joint_config_t;

extern const joint_config_t joint_config[ARM_JOINT_COUNT];

#endif
