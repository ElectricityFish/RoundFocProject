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
    long rpm;
    unsigned v, target, whole, tenth;
    int code, end = -1;
    assert(sscanf(report, "%*u.%*u,%ld,%u,%d,%u,%u.%u%n",
                  &rpm, &v, &code, &target, &whole, &tenth, &end) == 6);
    MotorTelemetry t = motor_telemetry();
    assert(end >= 0 && !strcmp(report + end, "\r\n") && code == state);
    assert(rpm == t.rpm && target == t.target_rpm && v == t.vector);
    assert(tenth < 10 && whole * 10u + tenth == duty && t.duty_permille == duty);
}

/* One encoder count per three ticks gives a nominal 8.57-degree step about 293 ms
   to settle, exercising a rotor that takes longer than the old 150 ms dwell. */
static int simulate_cal(int origin, int direction, unsigned poles)
{
    int position = origin, field_steps = 0, previous = 0;
    const unsigned sweep_ticks = CAL_ALIGN_TICKS + 12u * CAL_STEP_TICKS;
    command("cal");
    assert(motor_telemetry().state == MOTOR_CALIBRATING);
    for (unsigned i = 0; i < sweep_ticks && motor_telemetry().state == MOTOR_CALIBRATING; ++i) {
        if (vector < 6) {
            int change = ((int)vector - previous + 6) % 6;
            assert(change == 0 || change == 1 || change == 5);
            field_steps += change == 5 ? -1 : change;
            previous = (int)vector;
        }
        int target = origin + direction * (field_steps * ENCODER_COUNTS) / (6 * (int)poles);
        if (i % 3u == 0 && position != target)
            position += target > position ? 1 : -1;
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
    expect_report_code(-6);
    command("run 70.1");
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
    command("run 70");
    for (unsigned i = 0; i < TICKS_MS(1300); ++i) { /* 5% to 70% at 0.1% per 2 ms */
        position += direction * 16;
        sample(position);
    }
    assert(motor_telemetry().state == MOTOR_RUNNING && duty == 700);
    command("run 69.9%");
    sample(position);
    assert(duty == 699);
    command("run 70.1");
    expect_report_code(-6);
    sample(position);
    assert(duty == 699);
    command("stop");
    assert(!duty && motor_telemetry().state == MOTOR_READY && erase_count == before);

    command("cal");
    for (unsigned i = 0; i < CAL_RAMP_TICKS / 2u; ++i) sample(position);
    assert(vector == 0 && duty > 1u && duty < CAL_DUTY_PERMILLE);
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
    /* Hold the rotor fixed until the first displaced vector has had its full dwell. */
    for (unsigned i = 1; i < CAL_ALIGN_TICKS + CAL_STEP_TICKS; ++i) sample(position);
    assert(motor_telemetry().state == MOTOR_CALIBRATING && vector == 1 && duty == CAL_DUTY_PERMILLE);
    sample(position);
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

static double shaft_counts;
static int shaft_direction;

static void turn_tick(double rpm)
{
    shaft_counts += shaft_direction * rpm * ENCODER_COUNTS / (60.0 * CONTROL_HZ);
    sample((int)shaft_counts);
}

static void turn_for(int rpm, unsigned ms)
{
    for (unsigned i = 0; i < TICKS_MS(ms); ++i) turn_tick(rpm);
}

/* A first-order motor with a speed loss under load, driven by actual bridge duty. */
static double run_plant(double rpm, int load_rpm, unsigned ms)
{
    for (unsigned i = 0; i < TICKS_MS(ms); ++i) {
        double equilibrium = 12.0 * duty - load_rpm;
        if (equilibrium < 0) equilibrium = 0;
        rpm += (equilibrium - rpm) / TICKS_MS(120);
        turn_tick(rpm);
    }
    return rpm;
}

static void test_speed(int direction)
{
    memset(flash, 0, sizeof(flash));
    motor_init();
    tick = UINT32_MAX - TICKS_MS(100);
    shaft_counts = 0;
    shaft_direction = direction;
    sample(0);
    command("speed 3000");
    expect_report_code(-6);
    assert(motor_telemetry().state == MOTOR_NEEDS_CAL && !duty);
    Calibration cal = {0, (int8_t)direction};
    assert(cal_store_save(&cal));
    motor_init();
    command("speed 3000"); /* first angle after reset is still missing */
    expect_report_code(-1);
    sample(0);
    command("speed 5080");
    expect_report_code(-6);
    command("speed 3000.5");
    expect_report_code(-6);

    command("pi 0 0");
    command("speed 120");
    turn_for(0, 180);
    assert(duty == 50); /* stationary breakaway assist */
    turn_for(120, 200);
    assert(duty == 9 && motor_telemetry().target_rpm == 120);
    command("speed 3000");
    turn_for(3000, 640);
    assert(duty == 248 && abs((int)motor_telemetry().rpm - 3000) <= 1);
    expect_report_code(MOTOR_RUNNING);
    command("speed 3500");
    turn_for(3500, 180);
    assert(duty == 313);
    command("pi 0.005 0");
    turn_for(3000, 120);
    assert(duty == 338);
    command("pi 0 0.0001"); /* Reject the whole pair, including the first valid gain. */
    expect_report_code(-6);
    command("pi 0 1x");
    expect_report_code(-6);
    turn_for(3000, 60);
    assert(duty == 338);

    command("pi 0 0.500");
    command("speed 5079");
    turn_for(2607, 1800); /* rate-limited, then saturated, with sustained positive error */
    assert(duty == MAX_DUTY_PERMILLE);
    command("speed 2607");
    turn_for(2607, 60);
    assert(duty >= 200 && duty <= 201); /* No stored error from ramp or saturation. */
    command("pi 0.500 0");
    turn_for(4000, 80);
    assert(!duty && vector == 6 && motor_telemetry().target_rpm == 2607);
    expect_report_code(MOTOR_RUNNING); /* coasting is still speed mode */
    command("run 7.5%");
    turn_for(1000, 200);
    assert(duty == 75 && !motor_telemetry().target_rpm);
    command("speed 3428");
    turn_for(3428, 720);
    assert(duty >= 300 && duty <= 305);
    command("speed 0");
    assert(!duty && !motor_telemetry().target_rpm);
    expect_report_code(MOTOR_READY);

    command("speed 1000");
    for (unsigned i = 0; i < TICKS_MS(520); ++i) {
        turn_tick(0);
        if (i % TICKS_MS(80) == 0) command("speed 1000");
    }
    assert(motor_telemetry().state == MOTOR_STALL && !motor_telemetry().target_rpm && !duty);
    command("stop");
    command("speed 3000");
    turn_for(1500, 80);
    motor_sample(++tick, false, 0);
    assert(motor_telemetry().state == MOTOR_SENSOR_FAULT && !motor_telemetry().target_rpm && !duty);

    motor_init(); /* restores the default PI and clears feedback and integral */
    turn_tick(0);
    command("speed 3000");
    double rpm = run_plant(0, 0, 4500);
    assert(abs((int)rpm - 3000) < 10 && duty >= 245 && duty <= 255);
    rpm = run_plant(rpm, 400, 4500);
    assert(abs((int)rpm - 3000) < 10 && duty >= 280 && duty <= 286);
    command("speed 3500");
    rpm = run_plant(rpm, 400, 4500);
    assert(abs((int)rpm - 3500) < 10);
    command("speed 3000");
    rpm = run_plant(rpm, 400, 4500);
    assert(abs((int)rpm - 3000) < 10);
    command("run 0");
    assert(!duty && !motor_telemetry().target_rpm);
    expect_report_code(MOTOR_READY);
    printf("PASS speed feedback %+d: feedforward, PI, limits, coasting, load steps, modes and telemetry\n", direction);
}

int main(void)
{
    test_flash();
    test_direction(1, 16372);
    test_direction(-1, 23);
    test_speed(1);
    test_speed(-1);
    puts("All six-step regression checks passed.");
    return 0;
}
