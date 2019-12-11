#ifndef __MYMATH_H
#define __MYMATH_H

#include "myInclude.h"
#include "control.h"
/*			math info 		*/


typedef struct
{
    float X_last; //上一时刻的最优结果  X(k-1|k-1)
    float X_mid;  //当前时刻的预测结果  X(k|k-1)
    float X_now;  //当前时刻的最优结果  X(k|k)
    float P_mid;  //当前时刻预测结果的协方差  P(k|k-1)
    float P_now;  //当前时刻最优结果的协方差  P(k|k)
    float P_last; //上一时刻最优结果的协方差  P(k-1|k-1)
    float kg;     //kalman增益
    float A;      //系统参数
		float B;
    float Q;
    float R;
    float H;
}extKalman_t;

/*		dividing line 	*/
float myDeathZoom(float center,float width,float input);
int16_t myDeathZoom_int(int16_t center,uint16_t width,int16_t input);
float Median_filter(int16_t data,uint16_t measureNum,int16_t *Filterdata);
float Median_filter_float(float data,uint16_t measureNum,float *Filterdata);
float AverageToInstant(int16_t data,uint16_t num,int16_t *array,int16_t deltaT);
void KalmanCreate(extKalman_t *p,float T_Q,float T_R);
float KalmanFilter(extKalman_t* p,float dat);
float RampFloat(float step,float target,float current);
#endif

