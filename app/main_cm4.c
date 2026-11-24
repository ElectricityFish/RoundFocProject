#include "cy_project.h"

int main(void)
{
    /* Inspect SystemCoreClock before configuring the first peripheral. */
    SystemCoreClockUpdate();

    for (;;)
    {
        __WFI();
    }
}
