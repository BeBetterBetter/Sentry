#ifndef __PWM_H__
#define __PWM_H__

#include "system.h"

void TIM1_Init(void);
void TIM3_Init(void);
void FrictionOutput(uint16_t target,uint16_t *current);
void Friction_PWM(int16_t pwm1,int16_t pwm2);
void MagazineCtrl(uint16_t target,uint16_t *current);
#define PWM1  TIM3->CCR1     //Ä¦²ÁÂÖ,PA6
#define PWM2  TIM3->CCR2     //Ä¦²ÁÂÖ,PA7'
#define SERVO TIM1->CCR2		//µ¯²Ö

#endif

