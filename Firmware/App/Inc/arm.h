#ifndef ARM_H
#define ARM_H

#include <stdbool.h>
#include <stdint.h>
#include "robot_config.h"
#include "zdt.h"

typedef enum {
    ARM_FAULT_NONE, ARM_FAULT_ESTOP, ARM_FAULT_BUS, ARM_FAULT_DRIVER,
    ARM_FAULT_BLOCKED, ARM_FAULT_LIMIT, ARM_FAULT_STALE, ARM_FAULT_GRIPPER
} arm_fault_t;

typedef struct {
    zdt_status_t feedback;
    float origin_motor_deg;
    float angle_deg;
    float target_deg;
    float progress_deg;
    uint32_t feedback_ms;
    uint32_t started_ms;
    uint32_t progress_ms;
    uint32_t error_ms;
    uint32_t deadline_ms;
    bool valid;
    bool moving;
    bool error_pending;
} joint_state_t;

typedef struct {
    zdt_bus_t *bus;
    joint_state_t joint[ARM_JOINT_COUNT];
    arm_fault_t fault;
    uint8_t fault_address;
    uint8_t poll_joint;
    bool armed;
    bool referenced;
    bool stop_unconfirmed;
} arm_t;

void arm_init(arm_t *arm, zdt_bus_t *bus);
/* Zero means a manually supported, known pose. It does not search for end stops. */
bool arm_reference_here(arm_t *arm, uint32_t now_ms);
bool arm_enable(arm_t *arm, uint32_t now_ms);
bool arm_move_joint(arm_t *arm, uint8_t joint, float degrees,
                    float speed_deg_s, uint32_t now_ms);
bool arm_stop_joint(arm_t *arm, uint8_t joint);
bool arm_stop_all(arm_t *arm);
void arm_fault_stop(arm_t *arm, arm_fault_t fault, uint8_t address);
bool arm_clear_fault(arm_t *arm);
void arm_poll(arm_t *arm, uint32_t now_ms, bool estop);
bool arm_is_idle(const arm_t *arm);
const char *arm_fault_name(arm_fault_t fault);

#endif
