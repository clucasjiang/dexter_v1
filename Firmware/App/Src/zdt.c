#include "zdt.h"

#include <math.h>

#define ZDT_TRAILER 0x6BU
#define ZDT_ACK 0x02U
#define ZDT_MAX_RPM 3000.0f
#define ZDT_MAX_CURRENT_MA 5000U

static bool fail(zdt_bus_t *bus, zdt_error_t error)
{
    if (bus != NULL) {
        bus->last_error = error;
    }
    return false;
}

static void put_u16(uint8_t *out, uint16_t value)
{
    out[0] = (uint8_t)(value >> 8);
    out[1] = (uint8_t)value;
}

static void put_u32(uint8_t *out, uint32_t value)
{
    out[0] = (uint8_t)(value >> 24);
    out[1] = (uint8_t)(value >> 16);
    out[2] = (uint8_t)(value >> 8);
    out[3] = (uint8_t)value;
}

static uint16_t get_u16(const uint8_t *in)
{
    return (uint16_t)(((uint16_t)in[0] << 8) | in[1]);
}

static uint32_t get_u32(const uint8_t *in)
{
    return ((uint32_t)in[0] << 24) | ((uint32_t)in[1] << 16)
           | ((uint32_t)in[2] << 8) | in[3];
}

static float signed_position(const uint8_t *in, float units_per_degree)
{
    /* The sign is a separate byte. This is not a two's complement integer. */
    float value = (float)get_u32(&in[1]) / units_per_degree;
    return in[0] == 0U ? value : -value;
}

static bool exchange(zdt_bus_t *bus, const uint8_t *tx, size_t tx_len,
                     uint8_t *rx, size_t rx_len)
{
    if (bus == NULL || bus->exchange == NULL || bus->timeout_ms == 0U
        || tx[0] == 0U) {
        return fail(bus, ZDT_BAD_ARGUMENT);
    }

    bus->last_error = ZDT_OK;
    bus->last_reply_code = 0U;
    if (!bus->exchange(tx, tx_len, rx, rx_len, bus->timeout_ms)) {
        return fail(bus, ZDT_TRANSPORT_ERROR);
    }
    if (rx[0] != tx[0] || rx[1] != tx[1] || rx[rx_len - 1U] != ZDT_TRAILER) {
        return fail(bus, ZDT_BAD_REPLY);
    }
    return true;
}

static bool command(zdt_bus_t *bus, const uint8_t *tx, size_t tx_len)
{
    uint8_t rx[4] = {0};
    if (!exchange(bus, tx, tx_len, rx, sizeof(rx))) {
        return false;
    }
    bus->last_reply_code = rx[2];
    /* E2/EE are errors. 9F is a delayed completion message, not our ACK.
     * 12/22 also do not mean that the requested move was accepted (p.40).
     */
    if (rx[2] != ZDT_ACK) {
        return fail(bus, ZDT_COMMAND_REJECTED);
    }
    return true;
}

bool zdt_check_configuration(zdt_bus_t *bus, uint8_t id)
{
    const uint8_t tx[] = {id, 0x1A, ZDT_TRAILER};
    uint8_t rx[4] = {0};
    if (!exchange(bus, tx, sizeof(tx), rx, sizeof(rx))) {
        return false;
    }
    /* Manual p.77: bit1=Emm, bit2=closed loop, bit7=scaled input.
     * Require X firmware, closed loop and the default angle scale.
     */
    if ((rx[2] & 0x86U) != 0x04U) {
        return fail(bus, ZDT_WRONG_CONFIGURATION);
    }
    return true;
}

bool zdt_enable(zdt_bus_t *bus, uint8_t id, bool enabled)
{
    const uint8_t tx[] = {id, 0xF3, 0xAB, enabled ? 1U : 0U, 0, ZDT_TRAILER};
    return command(bus, tx, sizeof(tx));
}

bool zdt_set_power_marker(zdt_bus_t *bus, uint8_t id)
{
    /* Manual p.77: the driver restores this flag to zero after a power restart. */
    const uint8_t tx[] = {id, 0x50, 1, ZDT_TRAILER};
    return command(bus, tx, sizeof(tx));
}

bool zdt_stop(zdt_bus_t *bus, uint8_t id)
{
    const uint8_t tx[] = {id, 0xFE, 0x98, 0, ZDT_TRAILER};
    /* Keep the driver enabled: disabling a gravity-loaded joint can drop it. */
    return command(bus, tx, sizeof(tx));
}

static bool move(zdt_bus_t *bus, uint8_t id, float motor_deg, float rpm,
                 uint16_t accel, bool absolute, uint16_t current_limit_ma,
                 bool limit_current)
{
    uint8_t tx[18] = {id, limit_current ? 0xCD : 0xFD, 0};
    double angle_units;
    uint16_t speed_units;
    size_t tx_len = limit_current ? 18U : 16U;

    if (!isfinite(motor_deg) || !isfinite(rpm) || rpm < 0.1f
        || rpm > ZDT_MAX_RPM || accel == 0U
        || (limit_current && (current_limit_ma == 0U
                              || current_limit_ma > ZDT_MAX_CURRENT_MA))) {
        return fail(bus, ZDT_BAD_ARGUMENT);
    }

    /* Check in double before the integer cast so huge/NaN input cannot wrap. */
    angle_units = fabs((double)motor_deg) * 10.0 + 0.5;
    if (angle_units > (double)UINT32_MAX) {
        return fail(bus, ZDT_BAD_ARGUMENT);
    }
    speed_units = (uint16_t)(rpm * 10.0f + 0.5f);
    tx[2] = motor_deg < 0.0f ? 1U : 0U;
    put_u16(&tx[3], accel);
    put_u16(&tx[5], accel);
    put_u16(&tx[7], speed_units);
    put_u32(&tx[9], (uint32_t)angle_units);
    tx[13] = absolute ? 1U : 2U;
    tx[14] = 0U;
    if (limit_current) {
        put_u16(&tx[15], current_limit_ma);
    }
    tx[tx_len - 1U] = ZDT_TRAILER;
    /* Manual pp.55-56: X trapezoid commands use motor angle in 0.1 degrees. */
    return command(bus, tx, tx_len);
}

bool zdt_move(zdt_bus_t *bus, uint8_t id, float motor_deg, float rpm,
              uint16_t accel, bool absolute)
{
    return move(bus, id, motor_deg, rpm, accel, absolute, 0U, false);
}

bool zdt_move_limited(zdt_bus_t *bus, uint8_t id, float motor_deg, float rpm,
                      uint16_t accel, bool absolute, uint16_t current_limit_ma)
{
    return move(bus, id, motor_deg, rpm, accel, absolute, current_limit_ma, true);
}

static bool velocity(zdt_bus_t *bus, uint8_t id, float signed_rpm,
                     uint16_t accel, uint16_t current_limit_ma, bool limit_current)
{
    uint8_t tx[11] = {id, limit_current ? 0xC6 : 0xF6, 0};
    size_t tx_len = limit_current ? 11U : 9U;
    if (!isfinite(signed_rpm) || fabsf(signed_rpm) > ZDT_MAX_RPM
        || accel == 0U
        || (limit_current && (current_limit_ma == 0U
                              || current_limit_ma > ZDT_MAX_CURRENT_MA))) {
        return fail(bus, ZDT_BAD_ARGUMENT);
    }
    tx[2] = signed_rpm < 0.0f ? 1U : 0U;
    put_u16(&tx[3], accel);
    put_u16(&tx[5], (uint16_t)(fabsf(signed_rpm) * 10.0f + 0.5f));
    tx[7] = 0U;
    if (limit_current) {
        put_u16(&tx[8], current_limit_ma);
    }
    tx[tx_len - 1U] = ZDT_TRAILER;
    return command(bus, tx, tx_len);
}

bool zdt_velocity(zdt_bus_t *bus, uint8_t id, float signed_rpm, uint16_t accel)
{
    return velocity(bus, id, signed_rpm, accel, 0U, false);
}

bool zdt_velocity_limited(zdt_bus_t *bus, uint8_t id, float signed_rpm,
                          uint16_t accel, uint16_t current_limit_ma)
{
    return velocity(bus, id, signed_rpm, accel, current_limit_ma, true);
}

static bool read_angle(zdt_bus_t *bus, uint8_t id, uint8_t opcode,
                       float units_per_degree, float *motor_deg)
{
    const uint8_t tx[] = {id, opcode, ZDT_TRAILER};
    uint8_t rx[8] = {0};
    if (motor_deg == NULL) {
        return fail(bus, ZDT_BAD_ARGUMENT);
    }
    if (!exchange(bus, tx, sizeof(tx), rx, sizeof(rx))) {
        return false;
    }
    if (rx[2] > 1U) {
        return fail(bus, ZDT_BAD_REPLY);
    }
    *motor_deg = signed_position(&rx[2], units_per_degree);
    return true;
}

bool zdt_read_position(zdt_bus_t *bus, uint8_t id, float *motor_deg)
{
    return read_angle(bus, id, 0x36, 10.0f, motor_deg);
}

bool zdt_read_error(zdt_bus_t *bus, uint8_t id, float *motor_deg)
{
    /* X position errors have ten times finer resolution than positions (p.73). */
    return read_angle(bus, id, 0x37, 100.0f, motor_deg);
}

bool zdt_read_current(zdt_bus_t *bus, uint8_t id, uint16_t *current_ma)
{
    const uint8_t tx[] = {id, 0x27, ZDT_TRAILER};
    uint8_t rx[5] = {0};
    if (current_ma == NULL) {
        return fail(bus, ZDT_BAD_ARGUMENT);
    }
    if (!exchange(bus, tx, sizeof(tx), rx, sizeof(rx))) {
        return false;
    }
    *current_ma = get_u16(&rx[2]);
    return true;
}

bool zdt_read_speed(zdt_bus_t *bus, uint8_t id, float *rpm)
{
    const uint8_t tx[] = {id, 0x35, ZDT_TRAILER};
    uint8_t rx[6] = {0};
    if (rpm == NULL) {
        return fail(bus, ZDT_BAD_ARGUMENT);
    }
    if (!exchange(bus, tx, sizeof(tx), rx, sizeof(rx))) {
        return false;
    }
    if (rx[2] > 1U) {
        return fail(bus, ZDT_BAD_REPLY);
    }
    *rpm = (float)get_u16(&rx[3]) * (rx[2] == 0U ? 0.1f : -0.1f);
    return true;
}

bool zdt_read_status(zdt_bus_t *bus, uint8_t id, zdt_status_t *status)
{
    const uint8_t tx[] = {id, 0x43, 0x7A, ZDT_TRAILER};
    uint8_t rx[37] = {0};
    zdt_status_t result = {0};
    if (status == NULL) {
        return fail(bus, ZDT_BAD_ARGUMENT);
    }
    if (!exchange(bus, tx, sizeof(tx), rx, sizeof(rx))) {
        return false;
    }
    /* Manual pp.95-96. Emm replies are 31 bytes and use different angle units.
     * Checking both counts avoids quietly interpreting the other firmware.
     */
    if (rx[2] != sizeof(rx) || rx[3] != 12U || rx[14] > 1U || rx[19] > 1U
        || rx[22] > 1U || rx[27] > 1U || rx[32] > 1U) {
        return fail(bus, ZDT_BAD_REPLY);
    }
    result.bus_voltage_mv = get_u16(&rx[4]);
    result.bus_current_ma = get_u16(&rx[6]);
    result.phase_current_ma = get_u16(&rx[8]);
    result.target_deg = signed_position(&rx[14], 10.0f);
    result.rpm = (float)get_u16(&rx[20]) * (rx[19] == 0U ? 0.1f : -0.1f);
    result.position_deg = signed_position(&rx[22], 10.0f);
    result.error_deg = signed_position(&rx[27], 100.0f);
    /* The aggregate reply uses 0=positive (p.96); the separate 39 temperature
     * query has a contradictory sign description on p.72, so I do not use it.
     */
    result.temperature_c = (int16_t)((rx[32] == 0U ? 1 : -1) * rx[33]);
    result.driver_flags = rx[34];
    result.motor_flags = rx[35];
    result.enabled = (rx[35] & 0x01U) != 0U;
    result.reached = (rx[35] & 0x02U) != 0U;
    result.stalled = (rx[35] & 0x04U) != 0U;
    result.stall_protected = (rx[35] & 0x08U) != 0U;
    result.power_marker = (rx[35] & 0x80U) != 0U;
    result.encoder_ready = (rx[34] & 0x01U) != 0U;
    result.calibrated = (rx[34] & 0x02U) != 0U;
    result.homing = (rx[34] & 0x04U) != 0U;
    result.homing_failed = (rx[34] & 0x08U) != 0U;
    result.overtemperature = (rx[34] & 0x10U) != 0U;
    result.overcurrent = (rx[34] & 0x20U) != 0U;
    *status = result;
    return true;
}
