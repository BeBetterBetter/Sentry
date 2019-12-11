#ifndef __SHOOT_H
#define __SHOOT_H

#include "myInclude.h"
#include "pwm.h"
void ShootInit(void);
void ShootCtrl(void);
void TurnplateCtrl(void);
void FrictionCtrl(void);
void SendToTurnplate(int16_t turnplateOutput);
#endif



