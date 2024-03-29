#ifndef __SYSTEM_H
#define __SYSTEM_H

#include <stdio.h>
#include "stdbool.h"
#include "stm32f4xx_can.h"
#include "stm32f4xx.h"
#include "stdint.h"
#include "sys.h" 
#include "usart.h"

#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "myiic.h" 
#include "mpu6050.h"

#include "led.h"
#include "usart2.h"
#include "control.h"
#include "crc.h"
//#include "stdlib.h"

#include "temp.h"
#include "can1.h"
#include "myInclude.h"
#include "rc.h"
#include "gimbal.h"
#include "chassis.h"
#include "shoot.h"
#include "can2.h"
#include "usart4.h"
#include "vision.h"
extern volatile uint32_t sysTickUptime;

#define abs(x) ((x)>0? (x):(-(x)))

typedef struct 
{   
	float P[2];
	float I[2];
	float D[2];
	float IMax;
	float Looptime;
} PID_t;

extern PID_t PID;

float constrain(float amt, float low, float high);
int32_t constrain_int32(int32_t amt, int32_t low, int32_t high);
int16_t constrain_int16(int16_t amt, int16_t low, int16_t high);

uint64_t micros(void);
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);

uint32_t millis(void);
void Parameter_Init(void);
void System_Init(void);
void Loop(void);
void Stop(void);
#endif


