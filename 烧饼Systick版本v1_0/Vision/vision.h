#ifndef __VISION_H
#define __VISION_H

#include "system.h"
#include "myInclude.h"
void VisionProcess(void);
void UpdateVisionInfo(float input,uint16_t length,float *mathExpect,float *standardDeviation);
void VisionTask(void);
void GetPredictAngle(void);
#endif



