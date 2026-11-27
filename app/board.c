#include "board.h"
#include "motor.h"
#include "cy_project.h"
#include "gpio/cy_gpio.h"
#include "scb/cy_scb_spi.h"
#include "scb/cy_scb_uart.h"
#include "tcpwm/cy_tcpwm_pwm.h"

#define PWM_HZ              20000u
#define BRIDGE_BLANK_US     2u
#define BOOTSTRAP_US        30u
#define SPI_OVERSAMPLE      8u
#define SPI_HZ              2000000u
#define UART_OVERSAMPLE     16u
#define UART_BAUD           115200u

typedef struct {
    volatile stc_GPIO_PRT_t *port;
    unsigned high, low;
    en_hsiom_sel_t pwm_mux;
    volatile stc_TCPWM_GRP_CNT_t *counter;
    en_clk_dst_t clock;
} Phase;

static const Phase phases[] = {
    {GPIO_PRT14, 0, 1, P14_0_TCPWM0_LINE48, TCPWM0_GRP0_CNT48, PCLK_TCPWM0_CLOCKS48},
    {GPIO_PRT18, 4, 5, P18_4_TCPWM0_LINE53, TCPWM0_GRP0_CNT53, PCLK_TCPWM0_CLOCKS53},
    {GPIO_PRT18, 6, 7, P18_6_TCPWM0_LINE51, TCPWM0_GRP0_CNT51, PCLK_TCPWM0_CLOCKS51}
};

/* Indices advance the stator field by +60 electrical degrees from U+ V-. */
static const uint8_t drive_pair[6][2] = {{0,1}, {0,2}, {1,2}, {1,0}, {2,0}, {2,1}};
static unsigned active_vector = 6, active_duty;
static uint32_t pwm_counts;

/* CM4 IAR Watch: counts since reset and the last complete 16-bit SPI replies. */
static volatile struct {
    uint32_t valid_samples, transfer_errors, frame_errors, diagnostic_errors;
    uint16_t diaagc, anglecom, errfl;
    bool errfl_received;
} encoder_diag;

void bridge_off(void)
{
    /* High-side GPIO latches always contain zero. Remove the PWM route first. */
    for (unsigned i = 0; i < 3; ++i) {
        Cy_GPIO_SetHSIOM(phases[i].port, phases[i].high, HSIOM_SEL_GPIO);
        Cy_GPIO_Clr(phases[i].port, phases[i].low);
        Cy_Tcpwm_Pwm_Disable(phases[i].counter);
    }
    __DSB();
    active_vector = 6;
    active_duty = 0;
}

void bridge_drive(unsigned vector, unsigned duty)
{
    if (vector >= 6 || duty == 0 || duty > MAX_DUTY_PERMILLE) {
        bridge_off();
        return;
    }
    if (vector == active_vector && duty == active_duty) return;

    bool starting = active_vector == 6;
    const Phase *source = &phases[drive_pair[vector][0]];
    const Phase *sink = &phases[drive_pair[vector][1]];
    bridge_off();
    Cy_SysLib_DelayUs(BRIDGE_BLANK_US);
    if (starting) {
        /* Charge the source bootstrap with only one low-side MOSFET on. */
        Cy_GPIO_Set(source->port, source->low);
        Cy_SysLib_DelayUs(BOOTSTRAP_US);
        Cy_GPIO_Clr(source->port, source->low);
        Cy_SysLib_DelayUs(BRIDGE_BLANK_US);
    }
    Cy_Tcpwm_Pwm_SetCompare0(source->counter, (pwm_counts * duty + 500u) / 1000u);
    Cy_Tcpwm_Pwm_SetCounter(source->counter, 0);
    Cy_GPIO_Set(sink->port, sink->low);
    Cy_GPIO_SetHSIOM(source->port, source->high, source->pwm_mux);
    Cy_Tcpwm_Pwm_Enable(source->counter);
    Cy_Tcpwm_TriggerStart(source->counter);
    active_vector = vector;
    active_duty = duty;
}

static bool scb_clock(en_clk_dst_t clock, unsigned divider, uint32_t target)
{
    const cy_en_divider_types_t type = CY_SYSCLK_DIV_24_5_BIT;
    uint32_t actual;
    return Cy_SysClk_PeriphSetFracDividerAuto(type, divider, target, &actual) == CY_SYSCLK_SUCCESS &&
           actual >= target - target / 100u && actual <= target + target / 100u &&
           Cy_SysClk_PeriphAssignDivider(clock, type, divider) == CY_SYSCLK_SUCCESS &&
           Cy_SysClk_PeriphEnableDivider(type, divider) == CY_SYSCLK_SUCCESS;
}

bool board_init(void)
{
    for (unsigned i = 0; i < 3; ++i) {
        Cy_GPIO_Pin_FastInit(phases[i].port, phases[i].high, CY_GPIO_DM_STRONG_IN_OFF, 0, HSIOM_SEL_GPIO);
        Cy_GPIO_Pin_FastInit(phases[i].port, phases[i].low, CY_GPIO_DM_STRONG_IN_OFF, 0, HSIOM_SEL_GPIO);
    }
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    Cy_GPIO_Pin_FastInit(GPIO_PRT7, 0, CY_GPIO_DM_HIGHZ, 0, P7_0_SCB5_UART_RX);
    Cy_GPIO_Pin_FastInit(GPIO_PRT7, 1, CY_GPIO_DM_STRONG_IN_OFF, 1, P7_1_SCB5_UART_TX);
    const cy_stc_scb_uart_config_t uart = {
        .uartMode = CY_SCB_UART_STANDARD, .oversample = UART_OVERSAMPLE, .dataWidth = 8,
        .stopBits = CY_SCB_UART_STOP_BITS_1, .parity = CY_SCB_UART_PARITY_NONE,
        .dropOnFrameError = true, .breakWidth = 11
    };
    if (!scb_clock(PCLK_SCB5_CLOCK, 0, UART_BAUD * UART_OVERSAMPLE) ||
        Cy_SCB_UART_Init(SCB5, &uart, NULL) != CY_SCB_UART_SUCCESS) return false;
    Cy_SCB_UART_Enable(SCB5);

    Cy_GPIO_Pin_FastInit(GPIO_PRT13, 3, CY_GPIO_DM_STRONG_IN_OFF, 1, HSIOM_SEL_GPIO);
    Cy_GPIO_Pin_FastInit(GPIO_PRT13, 0, CY_GPIO_DM_HIGHZ, 0, P13_0_SCB3_SPI_MISO);
    Cy_GPIO_Pin_FastInit(GPIO_PRT13, 1, CY_GPIO_DM_STRONG_IN_OFF, 0, P13_1_SCB3_SPI_MOSI);
    Cy_GPIO_Pin_FastInit(GPIO_PRT13, 2, CY_GPIO_DM_STRONG_IN_OFF, 0, P13_2_SCB3_SPI_CLK);
    const cy_stc_scb_spi_config_t spi = {
        .spiMode = CY_SCB_SPI_MASTER, .subMode = CY_SCB_SPI_MOTOROLA,
        .sclkMode = CY_SCB_SPI_CPHA1_CPOL0, .oversample = SPI_OVERSAMPLE,
        .rxDataWidth = 16, .txDataWidth = 16, .enableMsbFirst = true, .enableTransferSeperation = true
    };
    if (!scb_clock(PCLK_SCB3_CLOCK, 1, SPI_HZ * SPI_OVERSAMPLE) ||
        Cy_SCB_SPI_Init(SCB3, &spi, NULL) != CY_SCB_SPI_SUCCESS) return false;
    Cy_SCB_SPI_Enable(SCB3);

    uint32_t peri_hz;
    if (Cy_SysClk_GetClkPeriFrequency(&peri_hz) != CY_SYSCLK_SUCCESS ||
        Cy_SysClk_PeriphSetDivider(CY_SYSCLK_DIV_16_BIT, 0, 0) != CY_SYSCLK_SUCCESS ||
        Cy_SysClk_PeriphEnableDivider(CY_SYSCLK_DIV_16_BIT, 0) != CY_SYSCLK_SUCCESS) return false;
    pwm_counts = (peri_hz + PWM_HZ / 2u) / PWM_HZ;
    if (pwm_counts < 1000 || pwm_counts > 65536u) return false;
    const cy_stc_tcpwm_pwm_config_t pwm = {
        .pwmMode = CY_TCPWM_PWM_MODE_PWM, .period = pwm_counts - 1u,
        .cc0MatchMode = CY_TCPWM_PWM_TR_CTRL2_CLEAR, .overflowMode = CY_TCPWM_PWM_TR_CTRL2_SET,
        .cc1MatchMode = CY_TCPWM_PWM_TR_CTRL2_NO_CHANGE, .underflowMode = CY_TCPWM_PWM_TR_CTRL2_NO_CHANGE,
        .countInput = CY_TCPWM_INPUT1, .countInputMode = CY_TCPWM_INPUT_LEVEL,
        .pwmOnDisable = CY_TCPWM_PWM_OUT_MODE_LOW
    };
    for (unsigned i = 0; i < 3; ++i) {
        if (Cy_SysClk_PeriphAssignDivider(phases[i].clock, CY_SYSCLK_DIV_16_BIT, 0) != CY_SYSCLK_SUCCESS ||
            Cy_Tcpwm_Pwm_Init(phases[i].counter, &pwm) != CY_RET_SUCCESS) return false;
    }
    return true;
}

/* The AS5047P returns the previous command's reply. Each word needs a CSn rising edge. */
static bool spi_word(uint16_t command, uint16_t *response)
{
    Cy_GPIO_Clr(GPIO_PRT13, 3);
    Cy_SysLib_DelayUs(1);
    bool ok = Cy_SCB_SPI_Write(SCB3, command) == 1;
    uint32_t start = DWT->CYCCNT;
    while (ok && (Cy_SCB_SPI_GetNumInRxFifo(SCB3) == 0 || Cy_SCB_SPI_IsBusBusy(SCB3))) {
        ok = DWT->CYCCNT - start < SystemCoreClock / 25000u; /* 40 us per frame */
    }
    if (ok) {
        *response = (uint16_t)Cy_SCB_SPI_Read(SCB3);
    } else {
        ++encoder_diag.transfer_errors;
        /* SDL 8.7.0 SPI_Disable clears all of CTRL, including MODE, MEM_WIDTH and OVS. */
        SCB3->unCTRL.stcField.u1ENABLED = 0u;
    }
    Cy_SysLib_DelayUs(1); /* tH >= half an SCLK period after the last falling edge. */
    Cy_GPIO_Set(GPIO_PRT13, 3);
    if (!ok) {
        Cy_SCB_SPI_ClearRxFifo(SCB3);
        Cy_SCB_SPI_ClearTxFifo(SCB3);
        Cy_SCB_SPI_Enable(SCB3);
    }
    Cy_SysLib_DelayUs(1); /* CSn high between commands. */
    return ok;
}

static bool response_valid(uint16_t word)
{
    if (word & 0x4000u) return false; /* EF */
    uint16_t parity = word;
    parity ^= parity >> 8;
    parity ^= parity >> 4;
    parity ^= parity >> 2;
    parity ^= parity >> 1;
    return (parity & 1u) == 0;
}

bool encoder_read(uint16_t *angle)
{
    uint16_t discard, diagnostic, position;
    /* Even-parity read commands: DIAAGC (0x3ffc), ANGLECOM (0x3fff), NOP (0). */
    if (!spi_word(0xFFFCu, &discard) || !spi_word(0xFFFFu, &diagnostic) ||
        !spi_word(0xC000u, &position)) return false;
    encoder_diag.diaagc = diagnostic;
    encoder_diag.anglecom = position;
    if (!response_valid(diagnostic) || !response_valid(position)) {
        ++encoder_diag.frame_errors;
        /* Reading ERRFL clears it. Its reply arrives during the following NOP. */
        uint16_t errfl;
        bool received = spi_word(0x4001u, &discard) && spi_word(0xC000u, &errfl);
        if (received) encoder_diag.errfl = errfl;
        encoder_diag.errfl_received = received;
        return false;
    }
    if ((diagnostic & 0x0F00u) != 0x0100u) { /* LF set, COF/MAGH/MAGL clear */
        ++encoder_diag.diagnostic_errors;
        return false;
    }
    ++encoder_diag.valid_samples;
    *angle = position & 0x3FFFu;
    return true;
}

int board_uart_get(void)
{
    const uint32_t errors = CY_SCB_UART_RX_OVERFLOW | CY_SCB_UART_RX_ERR_FRAME |
                            CY_SCB_UART_RX_ERR_PARITY | CY_SCB_UART_RX_BREAK_DETECT;
    if (Cy_SCB_UART_GetRxFifoStatus(SCB5) & errors) {
        Cy_SCB_UART_ClearRxFifo(SCB5);
        Cy_SCB_UART_ClearRxFifoStatus(SCB5, errors);
        return -2; /* Discard the damaged line through its next terminator. */
    }
    /* SDL 8.7.0 Get() reads RX_FIFO_RD directly, even when it is empty. */
    if (Cy_SCB_UART_GetNumInRxFifo(SCB5) == 0) return -1;
    return (int)(Cy_SCB_UART_Get(SCB5) & 0xFFu);
}

void board_uart_send(char *frame, size_t length)
{
    if (length <= Cy_SCB_GetFifoSize(SCB5) - Cy_SCB_UART_GetNumInTxFifo(SCB5)) {
        (void)Cy_SCB_UART_PutArray(SCB5, frame, length);
    }
}
