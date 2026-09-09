/* These tests use the manual's wire bytes, without an STM32 or motor attached. */
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "zdt.h"

static uint8_t expected_tx[18];
static size_t expected_tx_len;
static uint8_t prepared_reply[37];
static size_t prepared_reply_len;
static unsigned int exchanges;
static bool transport_works;

static bool fake_exchange(const uint8_t *tx, size_t tx_len, uint8_t *rx,
                          size_t rx_len, uint32_t timeout_ms)
{
    ++exchanges;
    assert(timeout_ms == 20U);
    assert(tx_len == expected_tx_len);
    assert(memcmp(tx, expected_tx, tx_len) == 0);
    if (!transport_works || prepared_reply_len != rx_len) {
        return false;
    }
    memcpy(rx, prepared_reply, rx_len);
    return true;
}

static zdt_bus_t bus = {.exchange = fake_exchange, .timeout_ms = 20U};

static void prepare(const uint8_t *tx, size_t tx_len,
                    const uint8_t *reply, size_t reply_len)
{
    assert(tx_len <= sizeof(expected_tx));
    assert(reply_len <= sizeof(prepared_reply));
    memcpy(expected_tx, tx, tx_len);
    memcpy(prepared_reply, reply, reply_len);
    expected_tx_len = tx_len;
    prepared_reply_len = reply_len;
    transport_works = true;
    exchanges = 0U;
}

static void near(float actual, float expected)
{
    assert(fabsf(actual - expected) < 0.001f);
}

static void test_commands(void)
{
    /* Enable and stop examples are printed on manual pp.50 and 60. */
    const uint8_t enable[] = {1, 0xF3, 0xAB, 1, 0, 0x6B};
    const uint8_t enable_ack[] = {1, 0xF3, 2, 0x6B};
    const uint8_t stop[] = {1, 0xFE, 0x98, 0, 0x6B};
    const uint8_t stop_ack[] = {1, 0xFE, 2, 0x6B};
    const uint8_t marker[] = {1, 0x50, 1, 0x6B};
    const uint8_t marker_ack[] = {1, 0x50, 2, 0x6B};
    const uint8_t move[] = {1, 0xFD, 1, 0x03, 0xE8, 0x03, 0xE8,
                           0x05, 0xE1, 0, 0, 0x8C, 0xA0, 1, 0, 0x6B};
    const uint8_t move_ack[] = {1, 0xFD, 2, 0x6B};
    const uint8_t claw_move[] = {7, 0xCD, 0, 0, 50, 0, 50, 0, 25,
                                0, 0, 0, 25, 2, 0, 0x01, 0x2C, 0x6B};
    const uint8_t claw_ack[] = {7, 0xCD, 2, 0x6B};
    /* Manual p.53 C6 example: CCW, 1000 RPM/s, 2000 RPM, 2000 mA. */
    const uint8_t velocity[] = {1, 0xC6, 1, 0x03, 0xE8, 0x4E, 0x20,
                               0, 0x07, 0xD0, 0x6B};
    const uint8_t velocity_ack[] = {1, 0xC6, 2, 0x6B};

    prepare(enable, sizeof(enable), enable_ack, sizeof(enable_ack));
    assert(zdt_enable(&bus, 1, true));
    assert(exchanges == 1U);
    assert(bus.last_error == ZDT_OK && bus.last_reply_code == 2U);

    prepare(stop, sizeof(stop), stop_ack, sizeof(stop_ack));
    assert(zdt_stop(&bus, 1));

    prepare(marker, sizeof(marker), marker_ack, sizeof(marker_ack));
    assert(zdt_set_power_marker(&bus, 1));
    assert(exchanges == 1U);
    prepared_reply[2] = 0xE2;
    assert(!zdt_set_power_marker(&bus, 1));
    assert(bus.last_error == ZDT_COMMAND_REJECTED);

    prepare(move, sizeof(move), move_ack, sizeof(move_ack));
    assert(zdt_move(&bus, 1, -3600.0f, 150.5f, 1000U, true));
    assert(exchanges == 1U);

    prepare(claw_move, sizeof(claw_move), claw_ack, sizeof(claw_ack));
    assert(zdt_move_limited(&bus, 7, 2.5f, 2.5f, 50U, false, 300U));

    prepare(velocity, sizeof(velocity), velocity_ack, sizeof(velocity_ack));
    assert(zdt_velocity_limited(&bus, 1, -2000.0f, 1000U, 2000U));
}

static void test_rejected_replies(void)
{
    const uint8_t tx[] = {1, 0xFE, 0x98, 0, 0x6B};
    const uint8_t ack[] = {1, 0xFE, 2, 0x6B};
    const uint8_t rejected_codes[] = {0xE2, 0xEE, 0x9F, 0x12, 0x22, 0x00};
    size_t i;
    for (i = 0U; i < sizeof(rejected_codes); ++i) {
        prepare(tx, sizeof(tx), ack, sizeof(ack));
        prepared_reply[2] = rejected_codes[i];
        assert(!zdt_stop(&bus, 1));
        assert(bus.last_error == ZDT_COMMAND_REJECTED);
        assert(bus.last_reply_code == rejected_codes[i]);
        assert(exchanges == 1U); /* A lost/rejected relative move is never retried. */
    }

    prepare(tx, sizeof(tx), ack, sizeof(ack));
    prepared_reply[0] = 2U;
    assert(!zdt_stop(&bus, 1));
    assert(bus.last_error == ZDT_BAD_REPLY);

    prepare(tx, sizeof(tx), ack, sizeof(ack));
    prepared_reply[1] = 0xF3;
    assert(!zdt_stop(&bus, 1));
    assert(bus.last_error == ZDT_BAD_REPLY);

    prepare(tx, sizeof(tx), ack, sizeof(ack));
    prepared_reply[3] = 0U;
    assert(!zdt_stop(&bus, 1));
    assert(bus.last_error == ZDT_BAD_REPLY);

    prepare(tx, sizeof(tx), ack, sizeof(ack) - 1U);
    assert(!zdt_stop(&bus, 1));
    assert(bus.last_error == ZDT_TRANSPORT_ERROR);

    prepare(tx, sizeof(tx), ack, sizeof(ack));
    transport_works = false;
    assert(!zdt_stop(&bus, 1));
    assert(exchanges == 1U);
}

static void test_position_units(void)
{
    const uint8_t position_tx[] = {2, 0x36, 0x6B};
    const uint8_t position_reply[] = {2, 0x36, 1, 0, 0, 0x0E, 0x10, 0x6B};
    const uint8_t error_tx[] = {2, 0x37, 0x6B};
    const uint8_t error_reply[] = {2, 0x37, 1, 0, 0, 0, 8, 0x6B};
    const uint8_t speed_tx[] = {2, 0x35, 0x6B};
    const uint8_t speed_reply[] = {2, 0x35, 1, 0x01, 0x41, 0x6B};
    const uint8_t current_tx[] = {2, 0x27, 0x6B};
    const uint8_t current_reply[] = {2, 0x27, 0x02, 0x58, 0x6B};
    float value = 123.0f;
    uint16_t current = 0U;

    prepare(position_tx, sizeof(position_tx), position_reply, sizeof(position_reply));
    assert(zdt_read_position(&bus, 2, &value));
    near(value, -360.0f);
    prepared_reply[2] = 0U;
    assert(zdt_read_position(&bus, 2, &value));
    near(value, 360.0f);
    prepared_reply[2] = 2U;
    assert(!zdt_read_position(&bus, 2, &value));
    near(value, 360.0f); /* Invalid readings cannot replace a valid measurement. */

    prepare(error_tx, sizeof(error_tx), error_reply, sizeof(error_reply));
    assert(zdt_read_error(&bus, 2, &value));
    near(value, -0.08f); /* Position /10 and error /100 must stay different. */

    prepare(speed_tx, sizeof(speed_tx), speed_reply, sizeof(speed_reply));
    assert(zdt_read_speed(&bus, 2, &value));
    near(value, -32.1f);

    prepare(current_tx, sizeof(current_tx), current_reply, sizeof(current_reply));
    assert(zdt_read_current(&bus, 2, &current));
    assert(current == 600U);
}

static void test_snapshot(void)
{
    const uint8_t tx[] = {1, 0x43, 0x7A, 0x6B};
    /* Exact example from manual p.96, not generated with driver helpers. */
    const uint8_t reply[] = {
        0x01, 0x43, 0x25, 0x0C, 0x5C, 0x67, 0x00, 0x00, 0x00, 0x03,
        0x30, 0xB5, 0x43, 0xEB, 0x01, 0x00, 0x00, 0x0E, 0x10, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x0E, 0x10, 0x01, 0x00, 0x00,
        0x00, 0x02, 0x00, 0x22, 0x03, 0x03, 0x6B
    };
    zdt_status_t status = {0};
    zdt_status_t before;
    const size_t signs[] = {14U, 19U, 22U, 27U, 32U};
    size_t i;

    prepare(tx, sizeof(tx), reply, sizeof(reply));
    assert(zdt_read_status(&bus, 1, &status));
    assert(exchanges == 1U);
    near(status.target_deg, -360.0f);
    near(status.position_deg, 360.0f);
    near(status.error_deg, -0.02f);
    near(status.rpm, 0.0f);
    assert(status.bus_voltage_mv == 23655U);
    assert(status.phase_current_ma == 3U);
    assert(status.temperature_c == 34);
    assert(status.enabled && status.reached);
    assert(status.encoder_ready && status.calibrated);
    assert(!status.stalled && !status.stall_protected);
    assert(!status.overcurrent && !status.overtemperature);
    assert(!status.power_marker);

    prepared_reply[35] = 0x83;
    assert(zdt_read_status(&bus, 1, &status));
    assert(status.power_marker && status.enabled && status.reached);
    prepared_reply[35] = 0x03; /* A power restart clears the marker. */
    assert(zdt_read_status(&bus, 1, &status));
    assert(!status.power_marker && status.enabled && status.reached);

    prepared_reply[34] = 0x3F;
    prepared_reply[35] = 0x0C;
    assert(zdt_read_status(&bus, 1, &status));
    assert(status.stalled && status.stall_protected);
    assert(status.overcurrent && status.overtemperature);
    assert(status.homing && status.homing_failed);
    assert(!status.enabled && !status.reached);

    before = status;
    for (i = 0U; i < sizeof(signs) / sizeof(signs[0]); ++i) {
        prepare(tx, sizeof(tx), reply, sizeof(reply));
        prepared_reply[signs[i]] = 2U;
        assert(!zdt_read_status(&bus, 1, &status));
        assert(memcmp(&before, &status, sizeof(status)) == 0);
    }
    prepare(tx, sizeof(tx), reply, sizeof(reply));
    prepared_reply[2] = 31U;
    assert(!zdt_read_status(&bus, 1, &status));
    prepare(tx, sizeof(tx), reply, sizeof(reply));
    prepared_reply[3] = 9U;
    assert(!zdt_read_status(&bus, 1, &status));
    prepare(tx, sizeof(tx), reply, 31U); /* Emm's shorter response. */
    assert(!zdt_read_status(&bus, 1, &status));
    assert(memcmp(&before, &status, sizeof(status)) == 0);
}

static void test_configuration_gate(void)
{
    const uint8_t tx[] = {1, 0x1A, 0x6B};
    const uint8_t reply[] = {1, 0x1A, 0x04, 0x6B};
    prepare(tx, sizeof(tx), reply, sizeof(reply));
    assert(zdt_check_configuration(&bus, 1));
    prepared_reply[2] = 0x06; /* Factory Emm closed-loop firmware. */
    assert(!zdt_check_configuration(&bus, 1));
    assert(bus.last_error == ZDT_WRONG_CONFIGURATION);
    prepared_reply[2] = 0x84; /* Different input angle scale. */
    assert(!zdt_check_configuration(&bus, 1));
    prepared_reply[2] = 0x00; /* Open loop. */
    assert(!zdt_check_configuration(&bus, 1));
}

static void test_invalid_inputs_send_nothing(void)
{
    unsigned int before = exchanges;
    assert(!zdt_move(&bus, 1, NAN, 10.0f, 100U, true));
    assert(!zdt_move(&bus, 1, INFINITY, 10.0f, 100U, true));
    assert(!zdt_move(&bus, 1, 1.0e20f, 10.0f, 100U, true));
    assert(!zdt_move(&bus, 1, 10.0f, NAN, 100U, true));
    assert(!zdt_move(&bus, 1, 10.0f, 3000.1f, 100U, true));
    assert(!zdt_move(&bus, 1, 10.0f, 0.0f, 100U, true));
    assert(!zdt_move(&bus, 1, 10.0f, -10.0f, 100U, true));
    assert(!zdt_move(&bus, 1, 10.0f, 10.0f, 0U, true));
    assert(!zdt_move_limited(&bus, 1, 1.0f, 10.0f, 100U, false, 0U));
    assert(!zdt_move_limited(&bus, 1, 1.0f, 10.0f, 100U, false, 5001U));
    assert(!zdt_velocity(&bus, 1, NAN, 100U));
    assert(!zdt_velocity(&bus, 1, -3000.1f, 100U));
    assert(!zdt_read_position(&bus, 1, NULL));
    assert(!zdt_read_status(&bus, 1, NULL));
    assert(!zdt_stop(&bus, 0));
    assert(!zdt_stop(NULL, 1));
    bus.timeout_ms = 0U;
    assert(!zdt_stop(&bus, 1));
    bus.timeout_ms = 20U;
    assert(exchanges == before);
}

int main(void)
{
    test_commands();
    test_rejected_replies();
    test_position_units();
    test_snapshot();
    test_configuration_gate();
    test_invalid_inputs_send_nothing();
    puts("ZDT protocol tests passed.");
    return 0;
}
