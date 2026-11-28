#ifndef ROUND_FOC_MOTOR_H
#define ROUND_FOC_MOTOR_H

#include <stdbool.h>
#include <stdint.h>

/* Initial bench settings for L4225-6S420-A. Duty is in 0.1 percent. */
#define MOTOR_POLE_PAIRS       7
#define ENCODER_COUNTS         16384
#define CONTROL_HZ             4000u
#define TICKS_MS(ms)           ((ms) * (CONTROL_HZ / 1000u))
#define CAL_DUTY_PERMILLE      70u
#define CAL_RAMP_TICKS         TICKS_MS(200u)
#define CAL_ALIGN_TICKS        TICKS_MS(800u)
#define CAL_STEP_TICKS         TICKS_MS(400u)
#define MAX_DUTY_PERMILLE      700u
#define MAX_SPEED_RPM          5079u
/* PI gains on the wire are in 0.001 percentage point / rpm (Ki also per second). */
#define MAX_PI_GAIN_MILLI      10000u

typedef struct {
    uint16_t zero_electrical;  /* encoder counts per electrical revolution */
    int8_t direction;          /* +1 or -1 for increasing stator vector index */
} Calibration;

typedef enum {
    MOTOR_TIMING_FAULT = -5, MOTOR_FLASH_FAULT = -4, MOTOR_STALL = -3,
    MOTOR_CAL_FAILED = -2, MOTOR_SENSOR_FAULT = -1, MOTOR_NEEDS_CAL = 0,
    MOTOR_READY = 1, MOTOR_CALIBRATING = 2, MOTOR_RUNNING = 3
} MotorState;

typedef struct {
    uint16_t raw_angle;
    int32_t rpm;
    uint8_t vector;            /* 1..6 applied, 0 when gates are off */
    MotorState state;
    unsigned target_rpm;       /* Positive only in speed mode. rpm above is the measured speed. */
    unsigned duty_permille;    /* Duty actually sent to the bridge, including calibration. */
} MotorTelemetry;

void motor_init(void);
void motor_sample(uint32_t tick, bool valid, uint16_t raw);
bool motor_calibrate(uint32_t tick);
bool motor_run(uint32_t tick, unsigned duty_permille);
bool motor_speed(uint32_t tick, unsigned rpm);
bool motor_set_pi(unsigned kp_milli, unsigned ki_milli);
void motor_stop(void);
MotorTelemetry motor_telemetry(void);

#endif
