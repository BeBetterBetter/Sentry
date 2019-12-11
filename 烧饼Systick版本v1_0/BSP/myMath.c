#include "myMath.h"

float matrix_value1;
float matrix_value2;

/** 
	*@param:�������˲����ṹ��*F���������ṹ�����ݳ�ʼ���Ľṹ��
	*@return:void
	*@brief:
	�������˲����Ľṹ���ʼ��
*/
void kalman_filter_init(kalman_filter_t *F, kalman_filter_init_t *I)
{
  mat_init(&F->xhat,2,1,(float *)I->xhat_data);
  mat_init(&F->xhatminus,2,1,(float *)I->xhatminus_data);
  mat_init(&F->z,2,1,(float *)I->z_data);
  mat_init(&F->A,2,2,(float *)I->A_data);
  mat_init(&F->H,2,2,(float *)I->H_data);
  mat_init(&F->Q,2,2,(float *)I->Q_data);
  mat_init(&F->R,2,2,(float *)I->R_data);
  mat_init(&F->P,2,2,(float *)I->P_data);
  mat_init(&F->Pminus,2,2,(float *)I->Pminus_data);
  mat_init(&F->K,2,2,(float *)I->K_data);
  mat_init(&F->AT,2,2,(float *)I->AT_data);
  mat_trans(&F->A, &F->AT);
  mat_init(&F->HT,2,2,(float *)I->HT_data);
  mat_trans(&F->H, &F->HT);
//  matrix_value2 = F->A.pData[1];
}


// xhatminus==x(k|k-1)  xhat==X(k-1|k-1)
// Pminus==p(k|k-1)     P==p(k-1|k-1)    AT==A'
// HT==H'   K==kg(k)    I=1
//

/**
  *@param �����������ṹ��
  *@param �Ƕ�-�ź�1
  *@param �ٶ�-�ź�2
*/
float *kalman_filter_calc(kalman_filter_t *F, float signal1, float signal2)
{
  float TEMP_data[4] = {0, 0, 0, 0};
  float TEMP_data21[2] = {0, 0};
  mat TEMP,TEMP21;

  mat_init(&TEMP,2,2,(float *)TEMP_data);//
  mat_init(&TEMP21,2,1,(float *)TEMP_data21);//

  F->z.pData[0] = signal1;//z(k)
  F->z.pData[1] = signal2;//z(k)

  //1. xhat'(k)= A xhat(k-1)
  mat_mult(&F->A, &F->xhat, &F->xhatminus);//  x(k|k-1) = A*X(k-1|k-1)+B*U(k)+W(K)
	
  //2. P'(k) = A P(k-1) AT + Q
  mat_mult(&F->A, &F->P, &F->Pminus);//   p(k|k-1) = A*p(k-1|k-1)*A'+Q
  mat_mult(&F->Pminus, &F->AT, &TEMP);//  p(k|k-1) = A*p(k-1|k-1)*A'+Q
  mat_add(&TEMP, &F->Q, &F->Pminus);//    p(k|k-1) = A*p(k-1|k-1)*A'+Q

  //3. K(k) = P'(k) HT / (H P'(k) HT + R)
  mat_mult(&F->H, &F->Pminus, &F->K);//  kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
  mat_mult(&F->K, &F->HT, &TEMP);//      kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
  mat_add(&TEMP, &F->R, &F->K);//        kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)

  mat_inv(&F->K, &F->P);//
  mat_mult(&F->Pminus, &F->HT, &TEMP);//
  mat_mult(&TEMP, &F->P, &F->K);//

  //4. xhat(k) = xhat'(k) + K(k) (z(k) - H xhat'(k))
  mat_mult(&F->H, &F->xhatminus, &TEMP21);//      x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
  mat_sub(&F->z, &TEMP21, &F->xhat);//            x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
  mat_mult(&F->K, &F->xhat, &TEMP21);//           x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
  mat_add(&F->xhatminus, &TEMP21, &F->xhat);//    x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))

  //5. P(k) = (1-K(k)H)P'(k)
  mat_mult(&F->K, &F->H, &F->P);//            p(k|k) = (I-kg(k)*H)*P(k|k-1)
  mat_sub(&F->Q, &F->P, &TEMP);//
  mat_mult(&TEMP, &F->Pminus, &F->P);

  matrix_value1 = F->xhat.pData[0];
  matrix_value2 = F->xhat.pData[1];

  F->filtered_value[0] = F->xhat.pData[0];
  F->filtered_value[1] = F->xhat.pData[1];
  return F->filtered_value;
}




/**
* @brief �������͵��������ƺ���
* @param
- float center:��������ֵ
- float width:�������
- float input:����ֵ
* @return int16_t
* ���뵽������ķ���ֵ.
*/
float myDeathZoom(float center,float width,float input)
{
	if(abs(input-center)>width)
	{
		//���������ĵľ�����ڿ��,������������
		return input;
		//�򷵻�����ֵ,��ֵ����
	}
	else
	{
		//���������ĵľ���С�ڿ��,����������
		return center;
		//�򷵻�����ֵ.
	}

}



/**
* @brief �������͵��������ƺ���
* @param
- int16_t center:��������ֵ
- uint16_t width:�������
- int16_t input:����ֵ
* @return int16_t
* ���뵽������ķ���ֵ.
*/
int16_t myDeathZoom_int(int16_t center,uint16_t width,int16_t input)
{
	if(abs(input-center)>width)
	{
		//���������ĵľ�����ڿ��,������������
		return input;
		//�򷵻�����ֵ,��ֵ����
	}
	else
	{
		//���������ĵľ���С�ڿ��,����������
		return center;
		//�򷵻�����ֵ.
	}

}

/**
* @brief ��ֵ�˲���
* @param
- float data:��������ֵ
- uint16_t measureNum:�������
- float input:����ֵ
* @return float 
* �˲����ֵ
*/

float Median_filter_float(float data,uint16_t measureNum,float *Filterdata)
{
  unsigned int i = 0;
	unsigned int j = 0;
	float temp;
	float MAX_error_targe = 0;
	float MAX_error1;
	Filterdata[measureNum-1] = data;
	for(i=0;i<measureNum-1;i++)
	{
	 Filterdata[i]=Filterdata[i+1];
	}
	for(i = 0 ; i < measureNum-1 ; i++)
	{
			for(j = 0 ; j < measureNum-1-i; j++)
			{
					if(Filterdata[j] > Filterdata[j+1] )
					{
							temp = Filterdata[j];
							Filterdata[j] =  Filterdata[j+1];
							Filterdata[j+1] = temp;
					}
			}
	}
	MAX_error1 = Filterdata[1] - Filterdata[0];
	for(i = 1 ; i < measureNum-1 ; i++)
	{
			if(MAX_error1 < Filterdata[i+1] - Filterdata[i] )
			{
					MAX_error1 =  Filterdata[i+1] - Filterdata[i];
					MAX_error_targe = i; 
			}
	}
	float Average_data=0;
	if(MAX_error_targe+1 > (measureNum+1)/2)
	{
			for(i = 0 ; i <= MAX_error_targe ; i++)
			{
					Average_data += Filterdata[i];
			}
			Average_data /= (MAX_error_targe+1);
	}
	else
	{
			for(i = MAX_error_targe + 1 ; i < measureNum ; i++)
			{
					Average_data += Filterdata[i];
			}
			Average_data /= (measureNum - MAX_error_targe -1);
	}
	return Average_data;
	
}

/**
* @brief ��ֵ�˲���
* @param
- int16_t data:��������ֵ
- uint16_t measureNum:�������
- int16_t input:����ֵ
* @return float 
* �˲����ֵ
*/

float Median_filter(int16_t data,uint16_t measureNum,int16_t *Filterdata)
{
  unsigned int i = 0;
	unsigned int j = 0;
	int temp;
	unsigned int MAX_error_targe = 0;
	int MAX_error1;
	Filterdata[measureNum-1] = data;
	for(i=0;i<measureNum-1;i++)
	{
	 Filterdata[i]=Filterdata[i+1];
	}
	for(i = 0 ; i < measureNum-1 ; i++)
	{
			for(j = 0 ; j < measureNum-1-i; j++)
			{
					if(Filterdata[j] > Filterdata[j+1] )
					{
							temp = Filterdata[j];
							Filterdata[j] =  Filterdata[j+1];
							Filterdata[j+1] = temp;
					}
			}
	}
	MAX_error1 = Filterdata[1] - Filterdata[0];
	for(i = 1 ; i < measureNum-1 ; i++)
	{
			if(MAX_error1 < Filterdata[i+1] - Filterdata[i] )
			{
					MAX_error1 =  Filterdata[i+1] - Filterdata[i];
					MAX_error_targe = i; 
			}
	}
	float Average_data=0;
	if(MAX_error_targe+1 > (measureNum+1)/2)
	{
			for(i = 0 ; i <= MAX_error_targe ; i++)
			{
					Average_data += Filterdata[i];
			}
			Average_data /= (MAX_error_targe+1);
	}
	else
	{
			for(i = MAX_error_targe + 1 ; i < measureNum ; i++)
			{
					Average_data += Filterdata[i];
			}
			Average_data /= (measureNum - MAX_error_targe -1);
	}
	return Average_data;
	
}


float AverageToInstant(int16_t data,uint16_t num,int16_t *array,int16_t deltaT)
{
	uint8_t i=0;
	int16_t sum1=0,sum2=0;
	float tmp=0.f;
	float res;
	tmp = num*num/(4.f*deltaT);
	
	for(i=0;i<num-1;i++)
	{
		array[i] = array[i+1];
	}
	array[num-1] = data;
	//���¶���
	
	for(i=0;i<num/2.f;i++)
	{
		sum1 += array[i];
	}
	for(i=num-1;i>=num/2.f;i--)
	{
		sum2 += array[i];
	}
	res = (sum2-sum1)/tmp;
	
	return res;
}


/**
  * @name   kalmanCreate
  * @brief  ����һ���������˲���
  * @param  p:  �˲���
  *         T_Q:ϵͳ����Э����
  *         T_R:��������Э����
  *         
  * @retval none
  * @attention R�̶���QԽ�󣬴���Խ���β���ֵ��Q�������ֻ�ò���ֵ
  *		       	��֮��QԽС����Խ����ģ��Ԥ��ֵ��QΪ������ֻ��ģ��Ԥ��
  */
void KalmanCreate(extKalman_t *p,float T_Q,float T_R)
{
    p->X_last = (float)0;
    p->P_last = 0;
    p->Q = T_Q;
    p->R = T_R;
    p->A = 1;
		p->B = 0;
    p->H = 1;
    p->X_mid = p->X_last;
}

/**
  * @name   KalmanFilter
  * @brief  �������˲���
  * @param  p:  �˲���
  *         dat:���˲�����
  * @retval �˲��������
  * @attention Z(k)��ϵͳ����,������ֵ   X(k|k)�ǿ������˲����ֵ,���������
  *            A=1 B=0 H=1 I=1  W(K)  V(k)�Ǹ�˹������,�����ڲ���ֵ����,���Բ��ù�
  *            �����ǿ�������5�����Ĺ�ʽ
  *            һ��H'��Ϊ������,����Ϊת�þ���
  */

float KalmanFilter(extKalman_t* p,float dat)
{
    p->X_mid =p->A*p->X_last;                     //�ٶȶ�Ӧ��ʽ(1)    x(k|k-1) = A*X(k-1|k-1)+B*U(k)+W(K)
    p->P_mid = p->A*p->P_last+p->Q;               //�ٶȶ�Ӧ��ʽ(2)    p(k|k-1) = A*p(k-1|k-1)*A'+Q
    p->kg = p->P_mid/(p->P_mid+p->R);             //�ٶȶ�Ӧ��ʽ(4)    kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
    p->X_now = p->X_mid+p->kg*(dat-p->X_mid);     //�ٶȶ�Ӧ��ʽ(3)    x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
    p->P_now = (1-p->kg)*p->P_mid;                //�ٶȶ�Ӧ��ʽ(5)    p(k|k) = (I-kg(k)*H)*P(k|k-1)
    p->P_last = p->P_now;                         //״̬����
    p->X_last = p->X_now;
    return p->X_now;							  //���Ԥ����x(k|k)
}




float RampFloat(float step,float target,float current)
{
	float tmp;
	if(abs(current - target)>step)
	{
		if(current < target)
			tmp = current + step;
		else
			tmp = current - step;
	}
	else
	{
		tmp = target;
	}
	return tmp;
	







}

