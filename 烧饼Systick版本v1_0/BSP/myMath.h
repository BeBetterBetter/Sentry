#ifndef __MYMATH_H
#define __MYMATH_H

#include "myInclude.h"
#include "control.h"
/*			math info 		*/


typedef struct
{
    float X_last; //��һʱ�̵����Ž��  X(k-1|k-1)
    float X_mid;  //��ǰʱ�̵�Ԥ����  X(k|k-1)
    float X_now;  //��ǰʱ�̵����Ž��  X(k|k)
    float P_mid;  //��ǰʱ��Ԥ������Э����  P(k|k-1)
    float P_now;  //��ǰʱ�����Ž����Э����  P(k|k)
    float P_last; //��һʱ�����Ž����Э����  P(k-1|k-1)
    float kg;     //kalman����
    float A;      //ϵͳ����
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

