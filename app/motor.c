#include "motor.h"
#include "board.h"
#include "cal_store.h"
#include <stdlib.h>

#define ALIGN_TICKS     TICKS_MS(400)
#define STEP_TICKS      TICKS_MS(150)
#define SETTLED_WINDOW  TICKS_MS(20)
#define STALL_TICKS     TICKS_MS(500)
#define MECH_STEP       (ENCODER_COUNTS / (6 * MOTOR_POLE_PAIRS))

static MotorTelemetry output;
static Calibration calibration;
static bool calibrated, have_angle;
static uint32_t last_sample, speed_start, last_motion, ramp_tick;
static int32_t speed_counts, forward_counts;
static unsigned target_duty, applied_duty;

/* Fixed vectors 0,1,2,3,4,5,0 then 5,4,3,2,1,0. Each point is sampled after settling. */
static struct {
    uint32_t start;
    unsigned point, samples;
    uint16_t anchor, positions[13];
    int32_t sum, minimum, maximum;
    int direction;
} sweep;

static uint16_t wrap_angle(int32_t value)
{
    return (uint16_t)((uint32_t)value & (ENCODER_COUNTS - 1u));
}

static int angle_delta(int to, int from)
{
    return (int)wrap_angle(to - from + ENCODER_COUNTS / 2) - ENCODER_COUNTS / 2;
}

static void stop_as(MotorState state)
{
    bridge_off();
    output.vector = 0;
    output.state = state;
}

void motor_stop(void)
{
    stop_as(calibrated ? MOTOR_READY : MOTOR_NEEDS_CAL);
}

void motor_init(void)
{
    bridge_off();
    output = (MotorTelemetry){0};
    have_angle = false;
    CalRead status = cal_store_load(&calibration);
    calibrated = status == CAL_VALID;
    if (status == CAL_IO_ERROR) output.state = MOTOR_FLASH_FAULT;
    else output.state = calibrated ? MOTOR_READY : MOTOR_NEEDS_CAL;
}

static bool fresh_angle(uint32_t tick)
{
    return have_angle && tick - last_sample <= 2u;
}

bool motor_calibrate(uint32_t tick)
{
    if ((output.state != MOTOR_READY && output.state != MOTOR_NEEDS_CAL) || !fresh_angle(tick))
        return false;
    sweep.start = tick;
    sweep.point = 0;
    sweep.samples = 0;
    sweep.direction = 0;
    output.state = MOTOR_CALIBRATING;
    return true;
}

bool motor_run(uint32_t tick, unsigned duty)
{
    if (duty == 0) { motor_stop(); return true; }
    if (duty > MAX_DUTY_PERMILLE || !calibrated || !fresh_angle(tick) ||
        (output.state != MOTOR_READY && output.state != MOTOR_RUNNING)) return false;
    if (output.state != MOTOR_RUNNING) {
        last_motion = ramp_tick = tick;
        forward_counts = 0;
        applied_duty = 1;
    }
    target_duty = duty;
    output.state = MOTOR_RUNNING;
    return true;
}

static void drive(unsigned vector, unsigned duty)
{
    bridge_drive(vector, duty);
    output.vector = (uint8_t)(vector + 1);
}

static unsigned cal_vector(unsigned point)
{
    return (point <= 6 ? point : 12u - point) % 6u;
}

static bool finish_calibration(Calibration *result)
{
    const int direction = sweep.direction;
    int forward = direction * angle_delta(sweep.positions[6], sweep.positions[0]);
    int backward = direction * angle_delta(sweep.positions[6], sweep.positions[12]);
    int expected = ENCODER_COUNTS / MOTOR_POLE_PAIRS;
    if (abs(forward - expected) > expected / 16 || abs(backward - expected) > expected / 16 ||
        abs(angle_delta(sweep.positions[12], sweep.positions[0])) > MECH_STEP / 4) return false;

    /* Express every settled observation as an estimate of vector-0 electrical angle.
       Unwrap around the first estimate before averaging, including across encoder zero. */
    int reference = wrap_angle(direction * MOTOR_POLE_PAIRS * sweep.positions[0]);
    int sum = 0;
    for (unsigned i = 0; i < 13; ++i) {
        int field = (int)(cal_vector(i) * ENCODER_COUNTS + 3u) / 6;
        int offset = direction * MOTOR_POLE_PAIRS * sweep.positions[i] - field;
        int error = angle_delta(offset, reference);
        if (abs(error) > ENCODER_COUNTS / 18) return false; /* 20 electrical degrees */
        sum += error;
    }
    result->direction = (int8_t)direction;
    result->zero_electrical = wrap_angle(reference + sum / 13);
    return true;
}

static void calibration_sample(uint32_t tick, uint16_t raw)
{
    unsigned point = sweep.point;
    uint32_t elapsed = tick - sweep.start;
    uint32_t dwell = point == 0 ? ALIGN_TICKS : STEP_TICKS;
    unsigned duty = CAL_DUTY_PERMILLE;
    if (point == 0 && elapsed < TICKS_MS(100))
        duty = 1u + (CAL_DUTY_PERMILLE - 1u) * elapsed / TICKS_MS(100);
    drive(cal_vector(point), duty);

    if (elapsed < dwell - SETTLED_WINDOW) return;
    if (!sweep.samples) {
        sweep.anchor = raw;
        sweep.sum = sweep.minimum = sweep.maximum = 0;
    }
    int delta = angle_delta(raw, sweep.anchor);
    sweep.sum += delta;
    if (delta < sweep.minimum) sweep.minimum = delta;
    if (delta > sweep.maximum) sweep.maximum = delta;
    ++sweep.samples;
    if (elapsed < dwell) return;
    if (sweep.samples < SETTLED_WINDOW / 2 || sweep.maximum - sweep.minimum > MECH_STEP / 5) {
        stop_as(MOTOR_CAL_FAILED);
        return;
    }
    sweep.positions[point] = wrap_angle(sweep.anchor + sweep.sum / (int)sweep.samples);
    if (point > 0) {
        int step = angle_delta(sweep.positions[point], sweep.positions[point - 1]);
        if (point == 1) sweep.direction = step < 0 ? -1 : 1;
        step *= point <= 6 ? sweep.direction : -sweep.direction;
        if (step < MECH_STEP / 2 || step > MECH_STEP * 3 / 2) {
            stop_as(MOTOR_CAL_FAILED);
            return;
        }
    }
    if (point < 12) {
        ++sweep.point;
        sweep.samples = 0;
        sweep.start = tick;
        return;
    }

    Calibration candidate;
    stop_as(MOTOR_CAL_FAILED); /* No winding is driven during validation or Flash operations. */
    if (!finish_calibration(&candidate)) return;
    if (!cal_store_save(&candidate)) { output.state = MOTOR_FLASH_FAULT; return; }
    calibration = candidate;
    calibrated = true;
    output.state = MOTOR_READY;
}

static void run_sample(uint32_t tick, uint16_t raw, int delta)
{
    forward_counts += calibration.direction * delta;
    if (forward_counts >= ENCODER_COUNTS / 128) { /* 2.8 mechanical degrees of net progress */
        forward_counts = 0;
        last_motion = tick;
    }
    if (tick - last_motion >= STALL_TICKS) { stop_as(MOTOR_STALL); return; }
    uint32_t steps = (tick - ramp_tick) / TICKS_MS(2);
    ramp_tick += steps * TICKS_MS(2);
    if (applied_duty < target_duty) applied_duty += steps;
    if (applied_duty > target_duty) applied_duty = target_duty;
    uint16_t electrical = wrap_angle(calibration.direction * MOTOR_POLE_PAIRS * raw -
                                     calibration.zero_electrical);
    /* Nearest stator vector to rotor +90 degrees (vector-0 is the calibration field). */
    unsigned vector = ((uint32_t)electrical * 6u / ENCODER_COUNTS + 2u) % 6u;
    drive(vector, applied_duty);
}

void motor_sample(uint32_t tick, bool valid, uint16_t raw)
{
    bool powered = output.state == MOTOR_RUNNING || output.state == MOTOR_CALIBRATING;
    if (!valid || raw >= ENCODER_COUNTS) {
        if (powered) stop_as(MOTOR_SENSOR_FAULT);
        have_angle = false;
        output.rpm = 0;
        return;
    }
    uint32_t gap = tick - last_sample;
    int delta = have_angle ? angle_delta(raw, output.raw_angle) : 0;
    if (powered && (!have_angle || gap > 2 || abs(delta) > (int)gap * MECH_STEP)) {
        stop_as(gap > 2 ? MOTOR_TIMING_FAULT : MOTOR_SENSOR_FAULT);
    }
    if (!have_angle || gap > 2) {
        speed_start = tick;
        speed_counts = 0;
        output.rpm = 0;
    } else {
        speed_counts += delta;
        uint32_t window = tick - speed_start;
        if (window >= TICKS_MS(20)) {
            int direction = calibrated ? calibration.direction : 1;
            output.rpm = (int32_t)((int64_t)speed_counts * direction * 60 * CONTROL_HZ /
                                   ((int64_t)window * ENCODER_COUNTS));
            speed_start = tick;
            speed_counts = 0;
        }
    }
    last_sample = tick;
    have_angle = true;
    output.raw_angle = raw;
    if (output.state == MOTOR_CALIBRATING) calibration_sample(tick, raw);
    else if (output.state == MOTOR_RUNNING) run_sample(tick, raw, delta);
}

MotorTelemetry motor_telemetry(void)
{
    MotorTelemetry snapshot = output;
    if (!have_angle && snapshot.state >= 0) snapshot.state = MOTOR_SENSOR_FAULT;
    return snapshot;
}
