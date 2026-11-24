/* Host regression for the production controller, command parser and calibration journal. */
#include "board.h"
#include "cal_store.h"
#include "console.h"
#include "motor.h"
#include "work_flash.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
    uint32_t words[32];
    bool programmed[32];
} Sector;
static Sector flash[2];
static int writes_left = -1, io_fault_slot = -1;
static unsigned erase_count, vector = 6, duty;
static char report[64];
static uint32_t tick;

void bridge_off(void) { vector = 6; duty = 0; }
void bridge_drive(unsigned v, unsigned d)
{
    assert(v < 6 && d > 0 && d <= MAX_DUTY_PERMILLE);
    vector = v;
    duty = d;
}
void board_uart_send(char *frame, size_t length)
{
    assert(length < sizeof(report));
    memcpy(report, frame, length);
    report[length] = '\0';
}

FlashRead work_flash_read(unsigned slot, unsigned offset, uint32_t *words, unsigned count)
{
    assert(slot < 2 && offset + count <= 32);
    if ((int)slot == io_fault_slot) return WF_IO_ERROR;
    for (unsigned i = 0; i < count; ++i) {
        if (!flash[slot].programmed[offset + i]) return WF_INVALID;
        words[i] = flash[slot].words[offset + i];
    }
    return WF_OK;
}
bool work_flash_erase(unsigned slot)
{
    assert(vector == 6 && !duty && slot < 2);
    memset(&flash[slot], 0, sizeof(flash[slot]));
    ++erase_count;
    return true;
}
bool work_flash_program(unsigned slot, unsigned offset, uint32_t word)
{
    assert(vector == 6 && !duty && !flash[slot].programmed[offset]);
    if (writes_left == 0) return false;
    if (writes_left > 0) --writes_left;
    flash[slot].programmed[offset] = true;
    flash[slot].words[offset] = word;
    return true;
}

static void expect_cal(Calibration expected)
{
    Calibration actual;
    assert(cal_store_load(&actual) == CAL_VALID);
    assert(actual.direction == expected.direction && actual.zero_electrical == expected.zero_electrical);
}

static void test_flash(void)
{
    Calibration first = {123, 1}, second = {15990, -1}, third = {833, 1};
    memset(flash, 0, sizeof(flash));
    assert(cal_store_load(&third) == CAL_EMPTY);
    writes_left = 2;
    assert(!cal_store_save(&first) && cal_store_load(&third) == CAL_EMPTY);
    writes_left = -1;
    assert(cal_store_save(&first) && cal_store_save(&second));
    io_fault_slot = 1;
    expect_cal(first); /* The other intact copy can still boot the motor into standby. */
    io_fault_slot = -1;
    Sector saved[2];
    memcpy(saved, flash, sizeof(flash));
    /* Includes interruption after erase and after the CRC, before the commit word. */
    for (int cut = 0; cut < 6; ++cut) {
        memcpy(flash, saved, sizeof(flash));
        writes_left = cut;
        assert(!cal_store_save(&third));
        expect_cal(second);
    }
    writes_left = -1;
    assert(cal_store_save(&third));
    expect_cal(third);
    flash[0].words[3] ^= 0x10; /* header is valid, payload CRC no longer matches */
    expect_cal(second);
    flash[0].programmed[0] = false; /* simulate an unreadable/ECC-torn payload */
    expect_cal(second);
    io_fault_slot = 1;
    unsigned before = erase_count;
    assert(!cal_store_save(&third) && erase_count == before);
    io_fault_slot = -1;
    puts("PASS Flash rotation, interrupted writes, CRC, unreadable slots and I/O failure");
}

static uint16_t wrap(int position) { return (uint16_t)((unsigned)position & (ENCODER_COUNTS - 1u)); }
static void sample(int position) { motor_sample(++tick, true, wrap(position)); }
static void command(const char *text)
{
    while (*text) console_input((unsigned char)*text++, tick);
    console_input('\r', tick);
    console_input('\n', tick);
}
static void expect_report_code(int state)
{
    console_report();
    const char *last = strrchr(report, ',');
    assert(last && atoi(last + 1) == state);
}

/* Model rotor following the actual energized vector. The mechanical move is limited
   per sample, so the controller sees settling before it measures each position. */
static int simulate_cal(int origin, int direction, unsigned poles)
{
    int position = origin, field_steps = 0, previous = 0;
    command("cal");
    assert(motor_telemetry().state == MOTOR_CALIBRATING);
    for (unsigned i = 0; i < TICKS_MS(3000) && motor_telemetry().state == MOTOR_CALIBRATING; ++i) {
        if (vector < 6) {
            int change = ((int)vector - previous + 6) % 6;
            assert(change == 0 || change == 1 || change == 5);
            field_steps += change == 5 ? -1 : change;
            previous = (int)vector;
        }
        int target = origin + direction * (field_steps * ENCODER_COUNTS) / (6 * (int)poles);
        int delta = target - position;
        if (delta > 20) delta = 20;
        if (delta < -20) delta = -20;
        position += delta;
        sample(position);
    }
    assert(vector == 6);
    return position;
}

static void test_direction(int direction, int origin)
{
    memset(flash, 0, sizeof(flash));
    motor_init();
    tick = UINT32_MAX - TICKS_MS(500); /* calibration spans tick wrap */
    sample(origin);
    command("run 4");
    assert(!duty && motor_telemetry().state == MOTOR_NEEDS_CAL);
    int position = simulate_cal(origin, direction, MOTOR_POLE_PAIRS);
    assert(motor_telemetry().state == MOTOR_READY);
    Calibration cal;
    assert(cal_store_load(&cal) == CAL_VALID && cal.direction == direction);
    int error = (int)wrap(cal.zero_electrical - direction * MOTOR_POLE_PAIRS * origin + ENCODER_COUNTS / 2)
                - ENCODER_COUNTS / 2;
    assert(abs(error) <= MOTOR_POLE_PAIRS);
    motor_init(); /* power cycle: hold gates off, no second calibration */
    sample(position);
    assert(!duty && motor_telemetry().state == MOTOR_READY);

    command("run 5x");
    command("run 15.1");
    expect_report_code(-6);
    assert(motor_telemetry().state == MOTOR_READY);
    console_input('r', tick);
    console_input(-2, tick); /* UART overrun: ignore rest of damaged line */
    command("un 5");
    assert(motor_telemetry().state == MOTOR_READY);
    command("run 5.0%");
    sample(position);
    assert(motor_telemetry().state == MOTOR_RUNNING && vector < 6);
    const unsigned before = erase_count;
    for (int turn = 0; turn < 64; ++turn) {
        unsigned changes = 0, previous = vector;
        for (int i = 0; i < ENCODER_COUNTS / 16; ++i) {
            position += direction * 16;
            sample(position);
            int rotor = wrap(direction * MOTOR_POLE_PAIRS * (position - origin));
            int lead = wrap((int)(vector * ENCODER_COUNTS / 6u) - rotor);
            assert(lead >= ENCODER_COUNTS / 6 - 10 && lead <= ENCODER_COUNTS / 3 + 10);
            if (vector != previous) {
                assert(vector == (previous + 1) % 6);
                previous = vector;
                ++changes;
            }
        }
        assert(changes == 42 && duty == 50 && motor_telemetry().rpm == 234);
    }
    command("stop");
    assert(!duty && motor_telemetry().state == MOTOR_READY && erase_count == before);

    command("cal");
    sample(position);
    command("stop");
    assert(!duty && erase_count == before);
    expect_cal(cal);
    command("run 4");
    sample(position);
    motor_sample(++tick, false, 0);
    assert(!duty && motor_telemetry().state == MOTOR_SENSOR_FAULT);
    sample(position);
    command("run 4");
    assert(motor_telemetry().state == MOTOR_SENSOR_FAULT);
    command("stop");
    command("run 4");
    for (unsigned i = 0; i <= TICKS_MS(510); ++i) {
        sample(position);
        if (i % TICKS_MS(100) == 0) command("run 4");
    }
    assert(!duty && motor_telemetry().state == MOTOR_STALL);
    command("stop");
    command("run 4");
    sample(position);
    tick += 8;
    sample(position);
    assert(!duty && motor_telemetry().state == MOTOR_TIMING_FAULT);
    command("stop");
    command("cal");
    for (unsigned i = 0; i < TICKS_MS(600); ++i) sample(position); /* locked rotor */
    assert(!duty && motor_telemetry().state == MOTOR_CAL_FAILED && erase_count == before);
    command("stop");
    simulate_cal(position, direction, 8);
    assert(!duty && motor_telemetry().state == MOTOR_CAL_FAILED && erase_count == before);
    expect_cal(cal);
    command("stop");
    writes_left = 2;
    simulate_cal(position, direction, MOTOR_POLE_PAIRS);
    assert(!duty && motor_telemetry().state == MOTOR_FLASH_FAULT);
    expect_cal(cal);
    writes_left = -1;
    command("stop");
    simulate_cal(position, direction, MOTOR_POLE_PAIRS);
    assert(!duty && motor_telemetry().state == MOTOR_READY && erase_count == before + 2);
    printf("PASS encoder direction %+d, wrap, 64 turns, stop, faults and optional recalibration\n", direction);
}

int main(void)
{
    test_flash();
    test_direction(1, 16372);
    test_direction(-1, 23);
    puts("All six-step regression checks passed.");
    return 0;
}
