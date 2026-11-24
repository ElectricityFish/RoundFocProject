#include "cy_project.h"

int main(void)
{
    /* SDL startup installed the SROM IPC vectors. Service CM4 Work Flash calls. */
    Cy_WDT_Disable();
    SystemInit();
    __enable_irq();
    Cy_SysEnableApplCore(CY_CORTEX_M4_APPL_ADDR);

    for (;;)
    {
        __WFI();
    }
}
