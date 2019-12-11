#ifndef __MYINCLUDE_H
#define __MYINCLUDE_H


#include "system.h"
#include "arm_math.h"

/*  priority define *
	can1 rx0  1 - 1  暂无
	usart2 rx 1 - 0  遥控
	can2 rx0  1 - 2  电机控制
	usart4 rx 0 - 1  视觉数据 比较重要	


*/


/*			 		math						*/

#define mat         arm_matrix_instance_f32 //float
#define mat_64      arm_matrix_instance_f64 //double
#define mat_init    arm_mat_init_f32
#define mat_add     arm_mat_add_f32
#define mat_sub     arm_mat_sub_f32
#define mat_mult    arm_mat_mult_f32
#define mat_trans   arm_mat_trans_f32//浮点矩阵转置
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
	float的IEEE编码格式在内存中是先存储尾数位后存储符号位和指数位
	int的IEEE编码格式应该也是 -- 如果不是就是伟明背锅
*/


typedef struct
{
	uint16_t nowLength;
	uint16_t queueLength;
	//长度
	float queue[100];
	//指针

}QueueObj;
//队列对象


#define VISION_BUF_SIZE 66

typedef __packed struct
{
	//  帧头
	uint8_t SOF; //帧头
	uint8_t cmdID;	//帧命令
	uint8_t CRC8; 	//帧头包检测
	
	//数据
	float     pitchAngle;
	float     yawAngle;
	float     distance;			//距离
	
	
	uint8_t   centreLock;		//是否瞄准到了中间  0没有  1瞄准到了
	uint8_t	  identifyTarget;	//视野内是否有目标/是否识别到了目标   0否  1是
	
	uint8_t   backTarget;			//预留
	uint8_t	  heroTarget;
	uint8_t	  blank_c;
	
	float 		pitchSpeed;
	float 		yawSpeed;
	float 		distanceSpeed;
	
	//帧尾数据校验
	uint16_t  CRC16;  
	

}VisionRxDataObj;
//视觉数据结构体






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


//增量式PID
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
//单极PID

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
//串级PID

/*		dividing line 	*/





/*			remote control						*/

typedef void (*rcFn)(void);
//遥控通用函数类型

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
#define RC_CH0_3_WID_VALUE 660.f //方便计算
//通道0-3的取值范围
#define RC_S1_2_UP_VALUE 1
#define RC_S1_2_DOWN_VALUE 2
#define RC_S1_2_MID_VALUE 3
//S1 S2 通道的值取值关系
#define RC_MOUSE_XYZ_MAX_VALUE 32767
#define RC_MOUSE_XYZ_MID_VALUE 0
#define RC_MOUSE_XYZ_MIN_VALUE -32768
//鼠标XYZ轴的数据
#define RC_MOUSE_PRESS_VALUE 1
#define RC_MOUSE_RELEASE_VALUE 0
//鼠标左右键松开数据值



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
	//遥控器信息

	uint8_t rxBuf[25];	
	//接收包
	bool status;
	//遥控器状态信息
	bool updateFlag;
	//遥控器更新状态信息
	uint32_t rcAliveCnt;
	//遥控器活性计数值
	rcFn rcRx;
	//遥控接收函数
	rcFn rcProtocol;
	//遥控协议
	
}RemoteCtrlObj;
//遥控器对象

/*		dividing line 	*/


/*			gimbal control						*/
#define GIMBAL_PITCH_INIT_ANGLE 570
#define GIMBAL_YAW_INIT_ANGLE  2065
//云台初始化机械角度

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
	MECHANICAL_MODE, //机械模式
	GYRO_MODE,			 //陀螺仪模式
	AUTO_MODE				 //自动模式
}GimbalMode;



typedef enum
{
	TRACK,//跟踪
	AIM		//瞄准
}AutoMode;

typedef struct
{
	GimbalMode mode;
	//云台模式
	gimbalFn startUpCtrl;
	//启动函数
	bool startUpFlag;
	//启动标志位
	gimbalFn jointCtrl;
	//联合控制函数
	struct
	{
		CascadePID pid;
		//串级pid参数
		int16_t measureMechAngle;
		//测量机械角度
		int16_t lastMeasureMechAngle;
		//测量机械角度
		int16_t targetMechAngle;
		//期望机械角度
		int16_t measureGyroY;
		//测量角速度值  pitch角速度值
		uint32_t measurePeriod;
		//测量周期 -- 用于测量旋转速度
		int16_t targetGyroY;
		//期望角速度值  pitch角速度值
		gimbalFn ctrl;
		//控制函数
	}pitch;
	
	struct
	{
		struct
		{
			CascadePID pid;
			//串级pid参数
			int16_t measureMechAngle;
			//测量角度值 -- 电机机械角度值
			int16_t lastMeasureMechAngle;
			//测量机械角度			
			int16_t targetMechAngle;
			//期望角度值 -- 电机机械角度值
			float measureGyroZ;
			//测量角速度 -- yaw角速度值
			uint32_t measurePeriod;
			//测量周期，用于测量旋转速度
			float targetGyroZ;
			//期望角速度 -- yaw角速度值
			gimbalFn ctrl;
			//控制函数
		}mechMode;
		//机械模式
		
		struct
		{
			CascadePID pid;
			//串级pid参数
			float measureYawAngle;
			//测量角度  -- yaw角度
			float targetYawAngle;
			//期望角度  -- yaw角度
			int16_t measureGyroZ;
			//测量角速度	-- yaw角速度
			int16_t targetGyroZ;
			//期望角速度 -- yaw角速度
			gimbalFn ctrl;
			//控制函数
		}gyroMode;
		//陀螺仪模式
	}yaw;
	
	
	struct 
	{
		AutoMode mode;
		//自动模式
		
		struct
		{
			CascadePID pid;
			//串级pid参数
			
			
			gimbalFn ctrl;
			//自瞄控制函数
		}pitch;
		
		struct
		{
			CascadePID pid;
			//串级pid参数
			
			
			
			gimbalFn ctrl;
			//自瞄控制函数
		}yaw;	
	}autoMode;
	//自瞄模式
}GimbalCtrlObj;
//云台控制对象

/*		dividing line   */

/*			chassis control						*/

#define CHASSIS_MAX_SPEED 7000
//最大速度期望
#define CHASSIS_MAX_OUTPUT 32767
//最大输出值
typedef void (*chassisFn)(void);
//函数类型

typedef struct
{
	struct
	{
		chassisFn ctrl;
		int16_t measureSpeed;
		int16_t targetSpeed;
		
		SinglePID pid;
		//速度单级pid
	}motor;
	//底盘电机控制   RM3510电机
	



}ChassisCtrlObj;
//底盘控制对象


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
	//发弹频率
	ShootMode	mode;
		//发弹模式
	struct
	{
		TurnplateStatus	status;
		//状态
		SinglePID pid;
		//单环速度PID
		int16_t measureSpeed;
		//测量速度
		int16_t targetSpeed;
		//期望速度
		int16_t measureAngle;
		//测量角度
		int16_t targetAngle;
		//期望角度
		shootFn ctrl;
		//控制函数
	}turnplate;
	//拨盘控制
	
	struct
	{
		bool isReadyFlag;
		//打开完毕标志位
		bool status;
		//状态
		shootFn ctrl;
		//控制函数
		uint16_t targetPWM;
		//期望pwm值
		uint16_t currentPWM;
		//当前pwm值
		
	}friction;
	//摩擦轮控制

	
}ShootCtrlObj;
//射击控制



/*		dividing line   */

#endif

