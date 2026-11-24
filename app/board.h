#ifndef ROUND_FOC_BOARD_H
#define ROUND_FOC_BOARD_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

bool board_init(void);
void bridge_off(void);
void bridge_drive(unsigned vector, unsigned duty_permille);
bool encoder_read(uint16_t *angle);
int board_uart_get(void);
void board_uart_send(char *frame, size_t length);

#endif
