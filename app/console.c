#include "console.h"
#include "motor.h"
#include "board.h"
#include <stdio.h>
#include <string.h>

static bool rejected;

static bool is_space(char c) { return c == ' ' || c == '\t'; }
static bool is_digit(char c) { return c >= '0' && c <= '9'; }

static bool execute(char *line, uint32_t tick)
{
    while (is_space(*line)) ++line;
    size_t length = strlen(line);
    while (length && is_space(line[length - 1])) line[--length] = '\0';
    if (!length) return true;
    if (strcmp(line, "stop") == 0) { motor_stop(); return true; }
    if (strcmp(line, "cal") == 0) return motor_calibrate(tick);
    if (strncmp(line, "run", 3) != 0 || !is_space(line[3])) return false;
    line += 3;
    while (is_space(*line)) ++line;
    if (!is_digit(*line)) return false;
    unsigned whole = 0;
    while (is_digit(*line)) {
        whole = whole * 10u + (unsigned)(*line++ - '0');
        if (whole > MAX_DUTY_PERMILLE / 10u) return false;
    }
    unsigned duty = whole * 10u;
    if (*line == '.') {
        ++line;
        if (!is_digit(*line)) return false;
        duty += (unsigned)(*line++ - '0');
    }
    if (*line == '%') ++line;
    return *line == '\0' && motor_run(tick, duty);
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
    int n = snprintf(frame, sizeof(frame), "%u.%u,%ld,%u,%d\r\n",
                     angle10 / 10u, angle10 % 10u, (long)t.rpm, (unsigned)t.vector, state);
    if (n > 0 && (size_t)n < sizeof(frame)) board_uart_send(frame, (size_t)n);
}
