#ifndef __MYINCLUDE_H
#define __MYINCLUDE_H


#include "system.h"
#include "arm_math.h"

/*  priority define *
	can1 rx0  1 - 1  ����
	usart2 rx 1 - 0  ң��
	can2 rx0  1 - 2  �������
	usart4 rx 0 - 1  �Ӿ����� �Ƚ���Ҫ	


*/


/*			 		math						*/

#define mat         arm_matrix_instance_f32 //float
#define mat_64      arm_matrix_instance_f64 //double
#define mat_init    arm_mat_init_f32
#define mat_add     arm_mat_add_f32
#define mat_sub     arm_mat_sub_f32
#define mat_mult    arm_mat_mult_f32
#define mat_trans   arm_mat_trans_f32//�������ת��
#define mat_inv     arm_mat_inverse_f32
#define mat_inv_f64 arm_mat_inverse_f64



typedef struct
{
  float raw_value;
  float filtered_value[2];
  mat xhat, xhatminus, z, A, H, AT, HT, Q, R, P, Pminus, K;
} kalman_filter_t;

typedef struct
{
  float raw_value;
  float filtered_value[2];
  float xhat_data[2], xhatminus_data[2], z_data[2],Pminus_data[4], K_data[4];
  float P_data[4];
  float AT_data[4], HT_data[4];
  float A_data[4];
  float H_data[4];
  float Q_data[4];
  float R_data[4];
} kalman_filter_init_t;



/*				dividing line 		*/



/*			 		vision						*/

/*
	float��IEEE�����ʽ���ڴ������ȴ洢β��λ��洢����λ��ָ��λ
	int��IEEE�����ʽӦ��Ҳ�� -- ������Ǿ���ΰ������
*/


typedef struct
{
	uint16_t nowLength;
	uint16_t queueLength;
	//����
	float queue[100];
	//ָ��

}QueueObj;
//���ж���


#define VISION_BUF_SIZE 66

typedef __packed struct
{
	//  ֡ͷ
	uint8_t SOF; //֡ͷ
	uint8_t cmdID;	//֡����
	uint8_t CRC8; 	//֡ͷ�����
	
	//����
	float     pitchAngle;
	float     yawAngle;
	float     distance;			//����
	
	
	uint8_t   centreLock;		//�Ƿ���׼�����м�  0û��  1��׼����
	uint8_t	  identifyTarget;	//��Ұ���Ƿ���Ŀ��/�Ƿ�ʶ����Ŀ��   0��  1��
	
	uint8_t   backTarget;			//Ԥ��
	uint8_t	  heroTarget;
	uint8_t	  blank_c;
	
	float 		pitchSpeed;
	float 		yawSpeed;
	float 		distanceSpeed;
	
	//֡β����У��
	uint16_t  CRC16;  
	

}VisionRxDataObj;
//�Ӿ����ݽṹ��






/*				dividing line 		*/









/*			pid control						*/


typedef struct
{
	float kp;
	float ki;
	float kd;
	
	float error;
	float lastError;
	float previousError;
	
	float pIterm;
	float iIterm;
	float dIterm;
	
	float iterm;
	
	float output;
	
	
}IncrementalPID;


//����ʽPID
typedef struct
{
	float kp;
	float ki;
	float kd;
	float error;
	float lastError;
	float integrate;
	float integrateMax;
	float pIterm;
	float iIterm;
	float dIterm;
	float iterm;	
}SinglePID;
//����PID

typedef struct
{
	float kpIn;
	float kiIn;
	float kdIn;
	float kpOut;
	float kiOut;
	float kdOut;
	
	float errorIn;
	float errorOut;
	float lastErrorIn;
	float lastErrorOut;
	
	float integrateIn;
	float integrateMaxIn;
	
	float integrateOut;
	float integrateMaxOut;
	
	float pItermIn;
	float iItermIn;
	float dItermIn;
	float itermIn;
	
	float pItermOut;
	float iItermOut;
	float dItermOut;
	float itermOut;
}CascadePID;
//����PID

/*		dividing line 	*/





/*			remote control						*/

typedef void (*rcFn)(void);
//ң��ͨ�ú�������

/* ----------------------- RC Channel Definition---------------------------- */
#define RC_CH_VALUE_MIN ((uint16_t)364 )
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX ((uint16_t)1684)
/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP ((uint16_t)1)
#define RC_SW_MID ((uint16_t)3)
#define RC_SW_DOWN ((uint16_t)2)
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W ((uint16_t)0x01<<0)		//0x01
#define KEY_PRESSED_OFFSET_S ((uint16_t)0x01<<1)		//0x02
#define KEY_PRESSED_OFFSET_A ((uint16_t)0x01<<2)		//0x04
#define KEY_PRESSED_OFFSET_D ((uint16_t)0x01<<3)		//0x08
#define KEY_PRESSED_OFFSET_SHIFT ((uint16_t)0x01<<4)	//0x10
#define KEY_PRESSED_OFFSET_CTRL ((uint16_t)0x01<<5)		//0x20
#define KEY_PRESSED_OFFSET_Q ((uint16_t)0x01<<6)		//0x40
#define KEY_PRESSED_OFFSET_E ((uint16_t)0x01<<7)		//0x80
#define KEY_PRESSED_OFFSET_R ((uint16_t)0x01<<8)		//0x100
#define KEY_PRESSED_OFFSET_F ((uint16_t)0x01<<9)		//0x200
#define KEY_PRESSED_OFFSET_G ((uint16_t)0x01<<10)		//0x400
#define KEY_PRESSED_OFFSET_Z ((uint16_t)0x01<<11)		//0x800
#define KEY_PRESSED_OFFSET_X ((uint16_t)0x01<<12)		//0x1000
#define KEY_PRESSED_OFFSET_C ((uint16_t)0x01<<13)		//0x2000
#define KEY_PRESSED_OFFSET_V ((uint16_t)0x01<<14)		//0x4000
#define KEY_PRESSED_OFFSET_B ((uint16_t)0x01<<15)		//0x8000

#define RC_CH0_3_MID_VALUE 1024
#define RC_CH0_3_WID_VALUE 660.f //�������
//ͨ��0-3��ȡֵ��Χ
#define RC_S1_2_UP_VALUE 1
#define RC_S1_2_DOWN_VALUE 2
#define RC_S1_2_MID_VALUE 3
//S1 S2 ͨ����ֵȡֵ��ϵ
#define RC_MOUSE_XYZ_MAX_VALUE 32767
#define RC_MOUSE_XYZ_MID_VALUE 0
#define RC_MOUSE_XYZ_MIN_VALUE -32768
//���XYZ�������
#define RC_MOUSE_PRESS_VALUE 1
#define RC_MOUSE_RELEASE_VALUE 0
//������Ҽ��ɿ�����ֵ



#define RC_OFFLINE_VALUE 25
typedef struct
{
	struct
	{
		struct
		{
			uint16_t ch0;
			uint16_t ch1;
			uint16_t ch2;
			uint16_t ch3;
			uint8_t s1;
			uint8_t s2;
		}rc;
		struct
		{
			int16_t x;
			int16_t y;
			int16_t z;
			uint8_t press_l;
			uint8_t press_r;
		}mouse;		
		struct
		{
			uint16_t v;
		}key;
	}info;
	//ң������Ϣ

	uint8_t rxBuf[25];	
	//���հ�
	bool status;
	//ң����״̬��Ϣ
	bool updateFlag;
	//ң��������״̬��Ϣ
	uint32_t rcAliveCnt;
	//ң�������Լ���ֵ
	rcFn rcRx;
	//ң�ؽ��պ���
	rcFn rcProtocol;
	//ң��Э��
	
}RemoteCtrlObj;
//ң��������

/*		dividing line 	*/


/*			gimbal control						*/
#define GIMBAL_PITCH_INIT_ANGLE 570
#define GIMBAL_YAW_INIT_ANGLE  2065
//��̨��ʼ����е�Ƕ�

#define GIMBAL_PITCH_MAX_SPEED 3000
#define GIMBAL_GYRO_YAW_MAX_SPEED	6000.f
//

#define GIMBAL_MOTOR_ANGLE_RANGE 8191.f

#define GIMBAL_PITCH_ANGLE_UP 4800
#define GIMBAL_PITCH_ANGLE_DOWN 3800
#define GIMBAL_PITCH_ANGLE_MID 4250
//pitch
#define GIMBAL_YAW_START_ANGLE 2050
//

typedef void (*gimbalFn)(void);


typedef enum
{
	MECHANICAL_MODE, //��еģʽ
	GYRO_MODE,			 //������ģʽ
	AUTO_MODE				 //�Զ�ģʽ
}GimbalMode;



typedef enum
{
	TRACK,//����
	AIM		//��׼
}AutoMode;

typedef struct
{
	GimbalMode mode;
	//��̨ģʽ
	gimbalFn startUpCtrl;
	//��������
	bool startUpFlag;
	//������־λ
	gimbalFn jointCtrl;
	//���Ͽ��ƺ���
	struct
	{
		CascadePID pid;
		//����pid����
		int16_t measureMechAngle;
		//������е�Ƕ�
		int16_t lastMeasureMechAngle;
		//������е�Ƕ�
		int16_t targetMechAngle;
		//������е�Ƕ�
		int16_t measureGyroY;
		//�������ٶ�ֵ  pitch���ٶ�ֵ
		uint32_t measurePeriod;
		//�������� -- ���ڲ�����ת�ٶ�
		int16_t targetGyroY;
		//�������ٶ�ֵ  pitch���ٶ�ֵ
		gimbalFn ctrl;
		//���ƺ���
	}pitch;
	
	struct
	{
		struct
		{
			CascadePID pid;
			//����pid����
			int16_t measureMechAngle;
			//�����Ƕ�ֵ -- �����е�Ƕ�ֵ
			int16_t lastMeasureMechAngle;
			//������е�Ƕ�			
			int16_t targetMechAngle;
			//�����Ƕ�ֵ -- �����е�Ƕ�ֵ
			float measureGyroZ;
			//�������ٶ� -- yaw���ٶ�ֵ
			uint32_t measurePeriod;
			//�������ڣ����ڲ�����ת�ٶ�
			float targetGyroZ;
			//�������ٶ� -- yaw���ٶ�ֵ
			gimbalFn ctrl;
			//���ƺ���
		}mechMode;
		//��еģʽ
		
		struct
		{
			CascadePID pid;
			//����pid����
			float measureYawAngle;
			//�����Ƕ�  -- yaw�Ƕ�
			float targetYawAngle;
			//�����Ƕ�  -- yaw�Ƕ�
			int16_t measureGyroZ;
			//�������ٶ�	-- yaw���ٶ�
			int16_t targetGyroZ;
			//�������ٶ� -- yaw���ٶ�
			gimbalFn ctrl;
			//���ƺ���
		}gyroMode;
		//������ģʽ
	}yaw;
	
	
	struct 
	{
		AutoMode mode;
		//�Զ�ģʽ
		
		struct
		{
			CascadePID pid;
			//����pid����
			
			
			gimbalFn ctrl;
			//������ƺ���
		}pitch;
		
		struct
		{
			CascadePID pid;
			//����pid����
			
			
			
			gimbalFn ctrl;
			//������ƺ���
		}yaw;	
	}autoMode;
	//����ģʽ
}GimbalCtrlObj;
//��̨���ƶ���

/*		dividing line   */

/*			chassis control						*/

#define CHASSIS_MAX_SPEED 7000
//����ٶ�����
#define CHASSIS_MAX_OUTPUT 32767
//������ֵ
typedef void (*chassisFn)(void);
//��������

typedef struct
{
	struct
	{
		chassisFn ctrl;
		int16_t measureSpeed;
		int16_t targetSpeed;
		
		SinglePID pid;
		//�ٶȵ���pid
	}motor;
	//���̵������   RM3510���
	



}ChassisCtrlObj;
//���̿��ƶ���


/*		dividing line   */



/*			shoot control						*/
#define SHOOT_PER_ANGLE 24573


typedef enum
{
	SINGLE_SHOOT,
	CONTINUS_SHOOT

}ShootMode;


typedef enum
{
	NORMAL,
	STUCK

}TurnplateStatus;



typedef void (*shootFn)(void);

typedef struct
{
	shootFn ctrl;

	uint16_t shootFreq;
	//����Ƶ��
	ShootMode	mode;
		//����ģʽ
	struct
	{
		TurnplateStatus	status;
		//״̬
		SinglePID pid;
		//�����ٶ�PID
		int16_t measureSpeed;
		//�����ٶ�
		int16_t targetSpeed;
		//�����ٶ�
		int16_t measureAngle;
		//�����Ƕ�
		int16_t targetAngle;
		//�����Ƕ�
		shootFn ctrl;
		//���ƺ���
	}turnplate;
	//���̿���
	
	struct
	{
		bool isReadyFlag;
		//����ϱ�־λ
		bool status;
		//״̬
		shootFn ctrl;
		//���ƺ���
		uint16_t targetPWM;
		//����pwmֵ
		uint16_t currentPWM;
		//��ǰpwmֵ
		
	}friction;
	//Ħ���ֿ���

	
}ShootCtrlObj;
//�������



/*		dividing line   */

#endif

