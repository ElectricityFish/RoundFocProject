#include "cy_project.h"

int main(void)
{
    /* Allow the learning firmware to stay idle between debugger stops. */
    Cy_WDT_Disable();
    Cy_SysEnableApplCore(CY_CORTEX_M4_APPL_ADDR);

    for (;;)
    {
        __WFI();
    }
}
