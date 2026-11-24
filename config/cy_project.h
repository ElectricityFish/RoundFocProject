#ifndef ROUND_FOC_CY_PROJECT_H
#define ROUND_FOC_CY_PROJECT_H

#include "cy_device_headers.h"
/* Board Y1 is the 16 MHz ECO. The SDL kit also waits for a 32 kHz crystal
   that is not fitted on this board (P21.0/P21.1 are unconnected). */
#undef CY_SYSTEM_WCO_ENABLE
#include "syslib/cy_syslib.h"
#include "sysclk/cy_sysclk.h"
#include "syswdt/cy_syswdt.h"
#include "sysint/cy_sysint.h"
#include "ipc/cy_ipc_drv.h"
#include "cpu/cy_cpu.h"

#endif
