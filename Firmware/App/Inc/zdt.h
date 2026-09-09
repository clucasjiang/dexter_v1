#ifndef ZDT_H
#define ZDT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* This driver is for the X firmware in the supplied ZDT manual, not Emm5.0.
 * Set Response=Receive, Check=0x6B, S_PosTDP=Disable and closed-loop mode.
 * Addresses 1..255 are allowed. I avoid broadcasts so each command is checked.
 */
typedef enum {
    ZDT_OK = 0,
    ZDT_BAD_ARGUMENT,
    ZDT_TRANSPORT_ERROR,
    ZDT_BAD_REPLY,
    ZDT_COMMAND_REJECTED,
    ZDT_WRONG_CONFIGURATION
} zdt_error_t;

typedef struct {
    /* The board code owns DE/RE and the UART. Return true only after receiving
     * exactly rx_len bytes within the total timeout, with no UART errors.
     * Drain stale bytes before TX; do not retry motion commands automatically.
     */
    bool (*exchange)(const uint8_t *tx, size_t tx_len, uint8_t *rx,
                     size_t rx_len, uint32_t timeout_ms);
    uint32_t timeout_ms;
    zdt_error_t last_error;
    uint8_t last_reply_code;
} zdt_bus_t;

typedef struct {
    float position_deg;      /* Motor shaft degrees, before the reducer. */
    float target_deg;
    float error_deg;
    float rpm;
    int16_t temperature_c;
    uint16_t phase_current_ma;
    uint16_t bus_current_ma;
    uint16_t bus_voltage_mv;
    uint8_t motor_flags;
    uint8_t driver_flags;
    bool enabled;
    bool reached;
    bool stalled;
    bool stall_protected;
    bool overtemperature;
    bool overcurrent;
    bool encoder_ready;
    bool calibrated;
    bool homing;
    bool homing_failed;
    bool power_marker;       /* Clears if the motor driver loses power/reboots. */
} zdt_status_t;

/* All functions make one bounded bus transaction, or none on invalid input.
 * Failed reads leave the output unchanged. A successful command means accepted,
 * not motion completed. Poll reached AND position before starting the next move.
 */
bool zdt_check_configuration(zdt_bus_t *bus, uint8_t id);
/* Set this after capturing a reference, then require power_marker in telemetry.
 * It is volatile: clearing it invalidates any captured motor position origin.
 */
bool zdt_set_power_marker(zdt_bus_t *bus, uint8_t id);
bool zdt_enable(zdt_bus_t *bus, uint8_t id, bool enabled);
bool zdt_stop(zdt_bus_t *bus, uint8_t id);

/* Negative motor_deg means CCW. absolute=false is relative to the measured
 * position (mode 02), rather than relative to the previous target (mode 00).
 * rpm is a positive maximum speed; accel applies to acceleration/deceleration.
 */
bool zdt_move(zdt_bus_t *bus, uint8_t id, float motor_deg, float rpm,
              uint16_t accel, bool absolute);
bool zdt_move_limited(zdt_bus_t *bus, uint8_t id, float motor_deg, float rpm,
                      uint16_t accel, bool absolute, uint16_t current_limit_ma);

/* Velocity has a speed bound but no driver-side time limit. For the claw I use
 * short zdt_move_limited() moves instead, so a broken cable cannot leave it
 * closing forever. signed_rpm=0 requests a ramped stop; zdt_stop is immediate.
 */
bool zdt_velocity(zdt_bus_t *bus, uint8_t id, float signed_rpm, uint16_t accel);
bool zdt_velocity_limited(zdt_bus_t *bus, uint8_t id, float signed_rpm,
                          uint16_t accel, uint16_t current_limit_ma);
bool zdt_read_position(zdt_bus_t *bus, uint8_t id, float *motor_deg);
bool zdt_read_error(zdt_bus_t *bus, uint8_t id, float *motor_deg);
bool zdt_read_current(zdt_bus_t *bus, uint8_t id, uint16_t *current_ma);
bool zdt_read_speed(zdt_bus_t *bus, uint8_t id, float *rpm);
bool zdt_read_status(zdt_bus_t *bus, uint8_t id, zdt_status_t *status);

#endif
