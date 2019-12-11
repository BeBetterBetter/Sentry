#ifndef __GIMBAL_H
#define __GIMBAL_H

#include "can1.h"
#include "system.h"
#include "myInclude.h"
#include "can2.h"


//

void GimbalInit(void);
void GimbalJointControl(void);
void GimbalStartUpControl(void);
void PitchControl(void);
void YawControlMechMode(void);
void YawControlGyroMode(void);

void YawControlAutoMode(void);
void PitchControlAutoMode(void);

void SendToGimbal(int16_t pitchOutput,int16_t yawOutput);


void GimbalUpdatePosture(int16_t yawTmp,int16_t gyroZ,int16_t gyroY);

int16_t TargetCoast(int16_t goalTarget,int16_t currentTarget);
int16_t GetThrCompensate(int16_t angle);

#endif





