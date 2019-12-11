#ifndef __CHASSIS_H
#define __CHASSIS_H

#include "myInclude.h"
#include "rc.h"
#include "gimbal.h"


void ChassisInit(void);
void ChassisSpeedCtrl(void);
void SendToChassis(int16_t motorOutput);

#endif

