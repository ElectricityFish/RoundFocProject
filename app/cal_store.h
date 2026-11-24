#ifndef ROUND_FOC_CAL_STORE_H
#define ROUND_FOC_CAL_STORE_H

#include "motor.h"

typedef enum { CAL_EMPTY, CAL_VALID, CAL_IO_ERROR } CalRead;
CalRead cal_store_load(Calibration *cal);
bool cal_store_save(const Calibration *cal);

#endif
