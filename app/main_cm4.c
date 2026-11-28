#include "cy_project.h"
#include "board.h"
#include "motor.h"
#include "console.h"

static volatile uint32_t ticks;

void SysTick_Handler(void)
{
    ++ticks;
}

int main(void)
{
    SystemCoreClockUpdate();
    if (!board_init() || SysTick_Config(SystemCoreClock / CONTROL_HZ) != 0u) {
        bridge_off();
        for (;;) {
            char fault[] = "0.0,0,0,-7,0,0.0\r\n";
            board_uart_send(fault, sizeof(fault) - 1u);
            Cy_SysLib_Delay(100);
        }
    }
    __enable_irq();
    motor_init();
    uint32_t serviced = ticks, reported = ticks;
    for (;;) {
        uint32_t now = ticks;
        if (now != serviced) {
            uint16_t raw = 0;
            bool valid = encoder_read(&raw);
            motor_sample(now, valid, raw);
            serviced = now;
        }
        /* Bound serial processing per visit so a flood cannot monopolize commutation. */
        for (unsigned i = 0; i < 8; ++i) {
            int byte = board_uart_get();
            if (byte == -1) break;
            console_input(byte, ticks);
        }
        if (now - reported >= TICKS_MS(20)) {
            reported = now;
            console_report();
        }
    }
}
