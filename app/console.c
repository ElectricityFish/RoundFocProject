#include "console.h"
#include "motor.h"
#include "board.h"
#include <stdio.h>
#include <string.h>

static bool rejected;

static bool is_space(char c) { return c == ' ' || c == '\t'; }
static bool is_digit(char c) { return c >= '0' && c <= '9'; }

/* scale is 1 for rpm, 10 for duty and 1000 for the two PI gains. */
static bool read_number(const char **cursor, unsigned scale, unsigned limit, unsigned *result)
{
    const char *p = *cursor;
    if (!is_digit(*p)) return false;
    unsigned value = 0;
    do {
        value = value * 10u + (unsigned)(*p++ - '0');
        if (value > limit / scale) return false;
    } while (is_digit(*p));
    value *= scale;
    if (*p == '.' && scale > 1u) {
        ++p;
        if (!is_digit(*p)) return false;
        while (is_digit(*p)) {
            scale /= 10u;
            if (scale == 0) return false;
            value += (unsigned)(*p++ - '0') * scale;
        }
    }
    if (value > limit) return false;
    *result = value;
    *cursor = p;
    return true;
}

static bool execute(char *line, uint32_t tick)
{
    while (is_space(*line)) ++line;
    size_t length = strlen(line);
    while (length && is_space(line[length - 1])) line[--length] = '\0';
    if (!length) return true;
    if (strcmp(line, "stop") == 0) { motor_stop(); return true; }
    if (strcmp(line, "cal") == 0) return motor_calibrate(tick);
    char *end = line;
    while (*end && !is_space(*end)) ++end;
    if (!*end) return false;
    *end++ = '\0';
    const char *args = end;
    while (is_space(*args)) ++args;
    unsigned value;
    if (strcmp(line, "run") == 0) {
        if (!read_number(&args, 10u, MAX_DUTY_PERMILLE, &value)) return false;
        if (*args == '%') ++args;
        return !*args && motor_run(tick, value);
    }
    if (strcmp(line, "speed") == 0) {
        return read_number(&args, 1u, MAX_SPEED_RPM, &value) && !*args && motor_speed(tick, value);
    }
    if (strcmp(line, "pi") == 0) {
        unsigned ki;
        if (!read_number(&args, 1000u, MAX_PI_GAIN_MILLI, &value) || !is_space(*args)) return false;
        while (is_space(*args)) ++args;
        return read_number(&args, 1000u, MAX_PI_GAIN_MILLI, &ki) && !*args && motor_set_pi(value, ki);
    }
    return false;
}

void console_input(int byte, uint32_t tick)
{
    static char line[32];
    static size_t used;
    static bool discard;
    if (byte == '\r' || byte == '\n') {
        if (used || discard) {
            line[used] = '\0';
            rejected = discard || !execute(line, tick);
        }
        used = 0;
        discard = false;
    } else if (byte < 0 || (byte != '\t' && (byte < ' ' || byte > '~')) || used == sizeof(line) - 1) {
        discard = true;
    } else if (!discard) {
        line[used++] = (char)byte;
    }
}

void console_report(void)
{
    MotorTelemetry t = motor_telemetry();
    /* Truncate at 359.9 rather than round a valid last encoder count to 360.0. */
    unsigned angle10 = (unsigned)((uint32_t)t.raw_angle * 3600u / ENCODER_COUNTS);
    char frame[48];
    int state = rejected && t.state >= 0 ? -6 : (int)t.state;
    rejected = false;
    int n = snprintf(frame, sizeof(frame), "%u.%u,%ld,%u,%d,%u,%u.%u\r\n",
                     angle10 / 10u, angle10 % 10u, (long)t.rpm, (unsigned)t.vector, state,
                     t.target_rpm, t.duty_permille / 10u, t.duty_permille % 10u);
    if (n > 0 && (size_t)n < sizeof(frame)) board_uart_send(frame, (size_t)n);
}
