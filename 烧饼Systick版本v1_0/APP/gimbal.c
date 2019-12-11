#include "gimbal.h"


GimbalCtrlObj gimbal;
//云台控制对象
extern VisionRxDataObj visionData;
//视觉数据结构体
float kFeedFoward=0.f;
//前馈系数
/**
  * @brief  云台控制的初始化函数
  * @param  void
  * @retval void
  * @attention 初始化云台对象的函数，参数及相关变量
*/
void GimbalInit(void)
{
	gimbal.startUpCtrl = GimbalStartUpControl;
	gimbal.startUpFlag = true;
	gimbal.mode = MECHANICAL_MODE;
	gimbal.jointCtrl = GimbalJointControl;
	//基本信息初始化
	gimbal.pitch.ctrl = PitchControl;
	gimbal.pitch.measureGyroY = 0;
	gimbal.pitch.measureMechAngle = 0;
	gimbal.pitch.lastMeasureMechAngle = 0;
	gimbal.pitch.measurePeriod = 0;
	gimbal.pitch.targetGyroY = 0;
	gimbal.pitch.targetMechAngle = GIMBAL_PITCH_ANGLE_MID;
	//pitch控制基本信息
	gimbal.pitch.pid.kpIn = 3.f;//0.6f;
	gimbal.pitch.pid.kiIn = 0.025f;//0.004f;
	gimbal.pitch.pid.kdIn = 0;//0.2f;
	gimbal.pitch.pid.errorIn = 0;
	gimbal.pitch.pid.lastErrorIn = 0;
	gimbal.pitch.pid.integrateIn = 0; 
	gimbal.pitch.pid.integrateMaxIn = 100000;//200000;
	gimbal.pitch.pid.pItermIn = 0;
	gimbal.pitch.pid.iItermIn = 0;
	gimbal.pitch.pid.dItermIn = 0;
	gimbal.pitch.pid.itermIn = 0;
	//pitch控制内环参数
	gimbal.pitch.pid.kpOut = 2.5f;//3;
	gimbal.pitch.pid.kiOut = 0.00f;//0.01f;
	gimbal.pitch.pid.kdOut = 0;
	gimbal.pitch.pid.errorOut = 0;
	gimbal.pitch.pid.lastErrorOut = 0;
	gimbal.pitch.pid.integrateOut = 0;
	gimbal.pitch.pid.integrateMaxOut = 0;//20000;
	gimbal.pitch.pid.pItermOut = 0;
	gimbal.pitch.pid.iItermOut = 0;
	gimbal.pitch.pid.dItermOut = 0;
	gimbal.pitch.pid.itermOut = 0;
	//pitch控制外环参数
	gimbal.yaw.gyroMode.ctrl = YawControlGyroMode;
	gimbal.yaw.gyroMode.measureGyroZ = 0;
	gimbal.yaw.gyroMode.measureYawAngle = 0;
	gimbal.yaw.gyroMode.targetGyroZ = 0;
	gimbal.yaw.gyroMode.targetYawAngle = 0;
	//yaw陀螺仪模式控制基本参数
	gimbal.yaw.mechMode.ctrl = YawControlMechMode;
	gimbal.yaw.mechMode.measureMechAngle = 0;
	gimbal.yaw.mechMode.lastMeasureMechAngle = 0;
	gimbal.yaw.mechMode.measureGyroZ = 0;
	gimbal.yaw.mechMode.measurePeriod = 0;
	gimbal.yaw.mechMode.targetGyroZ = 0;
	gimbal.yaw.mechMode.targetMechAngle = GIMBAL_YAW_START_ANGLE;
	//yaw机械模式控制基本参数
	gimbal.yaw.gyroMode.pid.kpIn = 3.6f;//
	gimbal.yaw.gyroMode.pid.kiIn = 0.02f;//
	gimbal.yaw.gyroMode.pid.kdIn = 0;
	gimbal.yaw.gyroMode.pid.errorIn = 0;
	gimbal.yaw.gyroMode.pid.lastErrorIn = 0;
	gimbal.yaw.gyroMode.pid.integrateIn = 0;
	gimbal.yaw.gyroMode.pid.integrateMaxIn = 75000;//
	gimbal.yaw.gyroMode.pid.pItermIn = 0;
	gimbal.yaw.gyroMode.pid.iItermIn = 0;
	gimbal.yaw.gyroMode.pid.dItermIn = 0;
	gimbal.yaw.gyroMode.pid.itermIn = 0;
	//yaw轴陀螺仪模式内环参数
	
	gimbal.yaw.gyroMode.pid.kpOut = 70.f;//90.f;
	gimbal.yaw.gyroMode.pid.kiOut = 0.0f;
	gimbal.yaw.gyroMode.pid.kdOut = 0;
	gimbal.yaw.gyroMode.pid.errorOut = 0;
	gimbal.yaw.gyroMode.pid.lastErrorOut = 0;	
	gimbal.yaw.gyroMode.pid.integrateOut = 0;
	gimbal.yaw.gyroMode.pid.integrateMaxOut = 0;
	gimbal.yaw.gyroMode.pid.pItermOut = 0;
	gimbal.yaw.gyroMode.pid.iItermOut = 0;
	gimbal.yaw.gyroMode.pid.dItermOut = 0;
	gimbal.yaw.gyroMode.pid.itermOut = 0;
	//yaw轴陀螺仪模式外环参数
	gimbal.yaw.mechMode.pid.kpIn = 3.6f;//8;
	gimbal.yaw.mechMode.pid.kiIn = 0.02f;//0.3f;
	gimbal.yaw.mechMode.pid.kdIn = 0;
	gimbal.yaw.mechMode.pid.errorIn = 0;
	gimbal.yaw.mechMode.pid.lastErrorIn = 0;
	gimbal.yaw.mechMode.pid.integrateIn = 0;
	gimbal.yaw.mechMode.pid.integrateMaxIn = 75000;//
	gimbal.yaw.mechMode.pid.pItermIn = 0;
	gimbal.yaw.mechMode.pid.iItermIn = 0;
	gimbal.yaw.mechMode.pid.dItermIn = 0;
	gimbal.yaw.mechMode.pid.itermIn = 0;
	//yaw轴机械模式内环参数
	gimbal.yaw.mechMode.pid.kpOut = 8.f;//10;
	gimbal.yaw.mechMode.pid.kiOut = 0.00f;
	gimbal.yaw.mechMode.pid.kdOut = 0;
	gimbal.yaw.mechMode.pid.errorOut = 0;
	gimbal.yaw.mechMode.pid.lastErrorOut = 0;
	gimbal.yaw.mechMode.pid.integrateOut = 0;
	gimbal.yaw.mechMode.pid.integrateMaxOut = 0;
	gimbal.yaw.mechMode.pid.pItermOut = 0;
	gimbal.yaw.mechMode.pid.iItermOut = 0;
	gimbal.yaw.mechMode.pid.dItermOut = 0;
	gimbal.yaw.mechMode.pid.itermOut = 0;
	//yaw轴机械模式外环参数
	
	
	/*		自动模式下的参数		*/
	gimbal.autoMode.mode = TRACK;
	gimbal.autoMode.yaw.ctrl = YawControlAutoMode;
	gimbal.autoMode.pitch.ctrl = PitchControlAutoMode;
	//模式/方法初始化
	gimbal.autoMode.yaw.pid.kpIn = 3.6f;
	gimbal.autoMode.yaw.pid.kiIn = 0.02f;//0.02
	gimbal.autoMode.yaw.pid.kdIn = 0;
	gimbal.autoMode.yaw.pid.errorIn = 0;
	gimbal.autoMode.yaw.pid.lastErrorIn = 0;
	gimbal.autoMode.yaw.pid.integrateIn = 0;
	gimbal.autoMode.yaw.pid.integrateMaxIn = 75000;
	gimbal.autoMode.yaw.pid.pItermIn = 0;
	gimbal.autoMode.yaw.pid.iItermIn = 0;
	gimbal.autoMode.yaw.pid.dItermIn = 0;
	gimbal.autoMode.yaw.pid.itermIn = 0;
	//yaw轴自动模式内环参数
	gimbal.autoMode.yaw.pid.kpOut = 100;//100
	gimbal.autoMode.yaw.pid.kiOut = 0.f;//0.1f
	gimbal.autoMode.yaw.pid.kdOut = 0;
	gimbal.autoMode.yaw.pid.errorOut = 0;
	gimbal.autoMode.yaw.pid.lastErrorOut = 0;
	gimbal.autoMode.yaw.pid.integrateOut = 0;
	gimbal.autoMode.yaw.pid.integrateMaxOut = 10000;//10000
	gimbal.autoMode.yaw.pid.pItermOut = 0;
	gimbal.autoMode.yaw.pid.iItermOut = 0;
	gimbal.autoMode.yaw.pid.dItermOut = 0;
	gimbal.autoMode.yaw.pid.itermOut = 0;
	//yaw轴自动模式外环参数	
	
	
	gimbal.autoMode.pitch.pid.kpIn = 3.f;//0.8
	gimbal.autoMode.pitch.pid.kiIn = 0.025f;//0.0025
	gimbal.autoMode.pitch.pid.kdIn = 0.f;
	gimbal.autoMode.pitch.pid.errorIn = 0;
	gimbal.autoMode.pitch.pid.lastErrorIn = 0;
	gimbal.autoMode.pitch.pid.integrateIn = 0;
	gimbal.autoMode.pitch.pid.integrateMaxIn = 100000;//600000
	gimbal.autoMode.pitch.pid.pItermIn = 0;
	gimbal.autoMode.pitch.pid.iItermIn = 0;
	gimbal.autoMode.pitch.pid.dItermIn = 0;
	gimbal.autoMode.pitch.pid.itermIn = 0;
	//pitch轴自动模式内环参数
	gimbal.autoMode.pitch.pid.kpOut = 40;
	gimbal.autoMode.pitch.pid.kiOut = 0;
	gimbal.autoMode.pitch.pid.kdOut = 0;
	gimbal.autoMode.pitch.pid.errorOut = 0;
	gimbal.autoMode.pitch.pid.lastErrorOut = 0;
	gimbal.autoMode.pitch.pid.integrateOut = 0;
	gimbal.autoMode.pitch.pid.integrateMaxOut = 0;
	gimbal.autoMode.pitch.pid.pItermOut = 0;
	gimbal.autoMode.pitch.pid.iItermOut = 0;
	gimbal.autoMode.pitch.pid.dItermOut = 0;
	gimbal.autoMode.pitch.pid.itermOut = 0;
	//pitch轴自动模式外环参数		
}


/**
  * @brief  云台组合控制函数
  * @param  void
  * @retval void
  * @attention 云台控制模式根据云台的当前模式来决定。由模式的变量mode决定
*/
void GimbalJointControl(void)
{
	if(gimbal.startUpFlag == true)
	{
		//启动标志位：刚启动
		gimbal.startUpCtrl();
		//初始化完成
		SendToGimbal(gimbal.pitch.pid.itermIn,gimbal.yaw.mechMode.pid.itermIn);
	}
	else
	{
		//启动标志位：启动完成
		if(gimbal.mode == MECHANICAL_MODE)
		{
			//机械模式
			gimbal.pitch.ctrl();
			//pitch
			gimbal.yaw.mechMode.ctrl();
			//yaw
			SendToGimbal(gimbal.pitch.pid.itermIn,gimbal.yaw.mechMode.pid.itermIn);
			//将PID计算结果发送至云台
		}
		else if(gimbal.mode == GYRO_MODE)
		{
			//陀螺仪模式
			gimbal.pitch.ctrl();
			//pitch
			gimbal.yaw.gyroMode.ctrl();
			//yaw			
			SendToGimbal(gimbal.pitch.pid.itermIn,gimbal.yaw.gyroMode.pid.itermIn);
			//将PID计算结果发送至云台			
		}
		else if(gimbal.mode == AUTO_MODE)
		{
			//自动模式
			//何为自动模式
			SendToGimbal(0,0);
		}
		else
		{
			//异常
			//异常掉电处理
			SendToGimbal(0,0);
		}	
	}
	
	
}

/**
  * @brief  云台启动的控制函数
  * @param  void
  * @retval void
  * @attention 主要用于初始化过程中使云台缓慢归中。由启动的标志位决定
*/
void GimbalStartUpControl(void)
{
	//启动过程
	bool yawOK=false,pitchOK=false;
	static uint16_t targetYaw,targetPitch;
	//作为期望值。
	int16_t step=10;
	//步长
	
	if(abs(targetPitch - GIMBAL_PITCH_INIT_ANGLE) <= step )
	{
		targetPitch = GIMBAL_PITCH_INIT_ANGLE;
		pitchOK=true;
		//pitch轴初始化完成
	}	
	else if(targetPitch > GIMBAL_PITCH_INIT_ANGLE)
	{
		targetPitch -= step;
	}
	else 
	{
		targetPitch += step;
	}
	
	
	
	if(abs(targetYaw - GIMBAL_YAW_INIT_ANGLE) <= step )
	{
		targetYaw = GIMBAL_YAW_INIT_ANGLE;
		yawOK=true;
		//yaw轴初始化完成
	}	
	else if(targetYaw > GIMBAL_YAW_INIT_ANGLE)
	{
		targetYaw -= step;
	}
	else 
	{
		targetYaw += step;
	}	
	
	if((pitchOK == true) && (yawOK == true))
		gimbal.startUpFlag=false;
	//完成初始化过程。
	
	gimbal.pitch.targetMechAngle = targetPitch;
	gimbal.yaw.mechMode.targetMechAngle = targetYaw;
	
	gimbal.pitch.ctrl();
	gimbal.yaw.mechMode.ctrl();
	
	
}
int16_t currentTarget=0;
extern extKalman_t kalman_targetGyrox;
/**
  * @brief  pitch轴控制函数
  * @param  void
  * @retval void
  * @attention 云台双模式的pitch轴控制都一样。 机械角度-陀螺仪角速度的串级PID控制
*/
void PitchControl(void)
{
//	float kLpf=0.1f;
//	static float lastTargetGyroY=0;
	gimbal.pitch.pid.errorOut = gimbal.pitch.targetMechAngle - gimbal.pitch.measureMechAngle;
	//计算外环误差
	gimbal.pitch.pid.integrateOut += gimbal.pitch.pid.errorOut;
	//积分
	gimbal.pitch.pid.integrateOut = constrain(gimbal.pitch.pid.integrateOut,-gimbal.pitch.pid.integrateMaxOut,gimbal.pitch.pid.integrateMaxOut);
	//积分限幅
//	if(((gimbal.pitch.pid.errorOut*gimbal.pitch.pid.integrateOut)<0)&&((abs(gimbal.pitch.pid.integrateOut))>(gimbal.pitch.pid.integrateMaxOut/4.f)))
//	{
//		gimbal.pitch.pid.integrateOut=0;
//	}
	gimbal.pitch.pid.pItermOut = gimbal.pitch.pid.kpOut * gimbal.pitch.pid.errorOut;
	//外环P
	gimbal.pitch.pid.pItermOut = constrain(gimbal.pitch.pid.pItermOut,-GIMBAL_PITCH_MAX_SPEED,GIMBAL_PITCH_MAX_SPEED);
	//外环P限幅
	gimbal.pitch.pid.iItermOut = gimbal.pitch.pid.kiOut * gimbal.pitch.pid.integrateOut;
	//外环I
	gimbal.pitch.pid.dItermOut = gimbal.pitch.pid.kdOut * (gimbal.pitch.pid.errorOut - gimbal.pitch.pid.lastErrorOut);
	//外环D
	gimbal.pitch.pid.dItermOut = constrain(gimbal.pitch.pid.dItermOut,-300,300);
	//外环D限幅
	gimbal.pitch.pid.itermOut = gimbal.pitch.pid.pItermOut + gimbal.pitch.pid.iItermOut + gimbal.pitch.pid.dItermOut;
	//外环输出
	gimbal.pitch.pid.itermOut = constrain(gimbal.pitch.pid.itermOut,-GIMBAL_PITCH_MAX_SPEED,GIMBAL_PITCH_MAX_SPEED);
	//输出限幅
	gimbal.pitch.pid.lastErrorOut = gimbal.pitch.pid.errorOut;
	//记录上一次误差
	
	gimbal.pitch.targetGyroY = -gimbal.pitch.pid.itermOut;
	
	gimbal.pitch.targetGyroY = KalmanFilter(&kalman_targetGyrox,gimbal.pitch.targetGyroY);
	
	
	/*	-----------------以上是pitch外环计算过程------------------------	*/
	/*
		油门和角度在静止状态下的对应关系公式：
			
			油门 = -3.1105*机械角度+13951  以此来弥补机械重量  -- 无负载
			
			油门 = -3.0459*机械角度+12795  以此来弥补机械重量  -- 带枪管
			
			油门 -- 以此来弥补机械重量 -- 带枪管和假摄像头

			3746 - 3770  -  油门 = -15.192*机械角度+58639
			3770 - 3780  -  油门 = -41.667*机械角度+158508
			3780 - 4054  -  油门 = -2.2791*机械角度+9647.5
			4054 - 4086  -  油门 = -14.443*机械角度+58953
			4086 - 4088  -  油门 = -100*机械角度+408500
			4088 - 4259  -  油门 = -2.4952*机械角度+9851.2
			4259 - 4752  -  油门 = -2.1127*机械角度+8216.3
			
			油门 = -5.7153*机械角度 + 24461 	 -- 带枪管和真摄像头和激光模块
			
	*/

	int16_t thrCompensate = 0;
	
//	thrCompensate = -3.1105f * gimbal.pitch.measureMechAngle + 13951;
//	thrCompensate = -3.0459f * gimbal.pitch.measureMechAngle + 12795;

//	thrCompensate = GetThrCompensate(gimbal.pitch.measureMechAngle);

//	if(gimbal.pitch.targetMechAngle>GIMBAL_PITCH_ANGLE_UP)
//	{
//		thrCompensate = -5.7153f * GIMBAL_PITCH_ANGLE_UP + 24461;	
//	}
//	else if(gimbal.pitch.targetMechAngle<GIMBAL_PITCH_ANGLE_DOWN)
//	{
//		thrCompensate = -5.7153f * GIMBAL_PITCH_ANGLE_DOWN + 24461;	
//	}
//	else
//		thrCompensate = -5.7153f * gimbal.pitch.targetMechAngle + 24461;	

	
	gimbal.pitch.pid.errorIn = gimbal.pitch.targetGyroY - gimbal.pitch.measureGyroY;
	//计算内环误差
	
	gimbal.pitch.pid.integrateIn += gimbal.pitch.pid.errorIn;
	//内环积分
	gimbal.pitch.pid.integrateIn = constrain(gimbal.pitch.pid.integrateIn,-gimbal.pitch.pid.integrateMaxIn,gimbal.pitch.pid.integrateMaxIn);
	//积分限幅
//	if(((gimbal.pitch.pid.errorIn*gimbal.pitch.pid.integrateIn)<0)&&((abs(gimbal.pitch.pid.integrateIn))>(gimbal.pitch.pid.integrateMaxIn/4.f)))
//	{
//		gimbal.pitch.pid.integrateIn=0;
//	}
	
	gimbal.pitch.pid.pItermIn = gimbal.pitch.pid.kpIn * gimbal.pitch.pid.errorIn;
	//内环P
	gimbal.pitch.pid.pItermIn = constrain(gimbal.pitch.pid.pItermIn,-5000,5000);
	//内环P限幅
	gimbal.pitch.pid.iItermIn = gimbal.pitch.pid.kiIn * gimbal.pitch.pid.integrateIn;
	//内环I
	gimbal.pitch.pid.dItermIn = gimbal.pitch.pid.kdIn * (gimbal.pitch.pid.errorIn - gimbal.pitch.pid.lastErrorIn);
	//内环D
	gimbal.pitch.pid.dItermIn = constrain(gimbal.pitch.pid.dItermIn,-1000,1000);
	//内环D限幅
	gimbal.pitch.pid.itermIn = gimbal.pitch.pid.pItermIn + gimbal.pitch.pid.iItermIn + gimbal.pitch.pid.dItermIn;
	//内环输出
	
	gimbal.pitch.pid.itermIn += thrCompensate;
	
	
	gimbal.pitch.pid.itermIn = constrain(gimbal.pitch.pid.itermIn,-5000,5000);
	
	
	//内环输出限幅
	gimbal.pitch.pid.lastErrorIn = gimbal.pitch.pid.errorIn;
	//记录内环误差
	/*	-----------------以上是pitch内环计算过程------------------------	*/	
}

/**
  * @brief  Yaw轴机械模式控制函数
  * @param  void
  * @retval void
  * @attention 机械角度-陀螺仪角速度的串级PID控制  -- 输出为正时，往减小机械角度的方向转
*/
void YawControlMechMode(void)
{
//	float kLpf=0.1f;
	static int16_t lastTargetGyroZ=0;
	gimbal.yaw.mechMode.pid.errorOut = gimbal.yaw.mechMode.targetMechAngle - gimbal.yaw.mechMode.measureMechAngle;
	//计算外环误差
	
	if(gimbal.yaw.mechMode.pid.errorOut>5000)
	{
		gimbal.yaw.mechMode.pid.errorOut = -(8191 - gimbal.yaw.mechMode.targetMechAngle + gimbal.yaw.mechMode.measureMechAngle);
	
	}
	else if(gimbal.yaw.mechMode.pid.errorOut<-5000)
	{
		gimbal.yaw.mechMode.pid.errorOut = (8191 - gimbal.yaw.mechMode.measureMechAngle + gimbal.yaw.mechMode.targetMechAngle);
	}
	
	gimbal.yaw.mechMode.pid.integrateOut += gimbal.yaw.mechMode.pid.errorOut;
	//积分
	gimbal.yaw.mechMode.pid.integrateOut = constrain(gimbal.yaw.mechMode.pid.integrateOut,-gimbal.yaw.mechMode.pid.integrateMaxOut,gimbal.yaw.mechMode.pid.integrateMaxOut);
	//积分限幅
	
	if(((gimbal.yaw.mechMode.pid.errorOut*gimbal.yaw.mechMode.pid.integrateOut)<0)&&(abs(gimbal.yaw.mechMode.pid.integrateOut)>(gimbal.yaw.mechMode.pid.integrateMaxOut/10.f)))
	{
		gimbal.yaw.mechMode.pid.integrateOut=0;
	}	
	//清除积分
	
	gimbal.yaw.mechMode.pid.pItermOut = gimbal.yaw.mechMode.pid.kpOut * gimbal.yaw.mechMode.pid.errorOut;
	//外环P
	gimbal.yaw.mechMode.pid.pItermOut = constrain(gimbal.yaw.mechMode.pid.pItermOut,-GIMBAL_GYRO_YAW_MAX_SPEED,GIMBAL_GYRO_YAW_MAX_SPEED);
	//外环P限幅
	gimbal.yaw.mechMode.pid.iItermOut = gimbal.yaw.mechMode.pid.kiOut * gimbal.yaw.mechMode.pid.integrateOut;
	//外环I
	gimbal.yaw.mechMode.pid.dItermOut = gimbal.yaw.mechMode.pid.kdOut * (gimbal.yaw.mechMode.pid.errorOut - gimbal.yaw.mechMode.pid.lastErrorOut);
	//外环D
	gimbal.yaw.mechMode.pid.dItermOut = constrain(gimbal.yaw.mechMode.pid.dItermOut,-2000,2000);
	//外环D限幅
	gimbal.yaw.mechMode.pid.itermOut = gimbal.yaw.mechMode.pid.pItermOut + gimbal.yaw.mechMode.pid.iItermOut + gimbal.yaw.mechMode.pid.dItermOut;
	//外环输出
	gimbal.yaw.mechMode.pid.itermOut = constrain(gimbal.yaw.mechMode.pid.itermOut,-GIMBAL_GYRO_YAW_MAX_SPEED,GIMBAL_GYRO_YAW_MAX_SPEED);
	//输出限幅
	gimbal.yaw.mechMode.pid.lastErrorOut = gimbal.yaw.mechMode.pid.errorOut;
	//记录上一次误差
	
	gimbal.yaw.mechMode.targetGyroZ = gimbal.yaw.mechMode.pid.itermOut;
//	






//	gimbal.yaw.mechMode.targetGyroZ = kLpf*gimbal.yaw.mechMode.targetGyroZ + (1-kLpf)*lastTargetGyroZ;
//	
	/*	-----------------以上是yaw.mechMode外环计算过程------------------------	*/
	float feedwardIterm=0;
	//前馈项
	feedwardIterm = kFeedFoward*(gimbal.yaw.mechMode.targetGyroZ - lastTargetGyroZ);
	//前馈计算
	feedwardIterm = constrain(feedwardIterm,-3000,3000);
	//限幅
	lastTargetGyroZ = gimbal.yaw.mechMode.targetGyroZ;	
	//记录上一次误差值
	
	gimbal.yaw.mechMode.pid.errorIn = gimbal.yaw.mechMode.targetGyroZ - gimbal.yaw.mechMode.measureGyroZ;
	//计算内环误差
	gimbal.yaw.mechMode.pid.integrateIn += gimbal.yaw.mechMode.pid.errorIn;
	//内环积分
	gimbal.yaw.mechMode.pid.integrateIn = constrain(gimbal.yaw.mechMode.pid.integrateIn,-gimbal.yaw.mechMode.pid.integrateMaxIn,gimbal.yaw.mechMode.pid.integrateMaxIn);
	//积分限幅
	if(((gimbal.yaw.mechMode.pid.errorIn*gimbal.yaw.mechMode.pid.integrateIn)<0)&&(abs(gimbal.yaw.mechMode.pid.integrateIn)>(gimbal.yaw.mechMode.pid.integrateMaxIn/5.f)))
	{
		gimbal.yaw.mechMode.pid.integrateIn=0;
	}
	gimbal.yaw.mechMode.pid.pItermIn = gimbal.yaw.mechMode.pid.kpIn * gimbal.yaw.mechMode.pid.errorIn;
	//内环P
	gimbal.yaw.mechMode.pid.pItermIn = constrain(gimbal.yaw.mechMode.pid.pItermIn,-5000,5000);
	//内环P限幅
	gimbal.yaw.mechMode.pid.iItermIn = gimbal.yaw.mechMode.pid.kiIn * gimbal.yaw.mechMode.pid.integrateIn;
	//内环I
	gimbal.yaw.mechMode.pid.dItermIn = gimbal.yaw.mechMode.pid.kdIn * (gimbal.yaw.mechMode.pid.errorIn - gimbal.yaw.mechMode.pid.lastErrorIn);
	//内环D
	gimbal.yaw.mechMode.pid.dItermIn = constrain(gimbal.yaw.mechMode.pid.dItermIn,-2000,2000);
	//内环D限幅
	gimbal.yaw.mechMode.pid.itermIn = gimbal.yaw.mechMode.pid.pItermIn + gimbal.yaw.mechMode.pid.iItermIn + gimbal.yaw.mechMode.pid.dItermIn + feedwardIterm;
	//内环输出
	gimbal.yaw.mechMode.pid.itermIn = constrain(gimbal.yaw.mechMode.pid.itermIn,-5000,5000);
	//内环输出限幅
	gimbal.yaw.mechMode.pid.lastErrorIn = gimbal.yaw.mechMode.pid.errorIn;
	//记录内环误差	
	
		/*	-----------------以上是yaw.mechMode内环计算过程------------------------	*/
}

/**
  * @brief  Yaw轴陀螺仪模式控制函数
  * @param  void
  * @retval void
  * @attention 姿态YAW角度-陀螺仪角速度的串级PID控制
*/
void YawControlGyroMode(void)
{
//	float kLpf=0.1f;
	static int16_t lastTargetGyroZ=0;
	gimbal.yaw.gyroMode.pid.errorOut = gimbal.yaw.gyroMode.targetYawAngle - gimbal.yaw.gyroMode.measureYawAngle;
	//计算外环误差
	if(gimbal.yaw.gyroMode.pid.errorOut>180)
	{
		//反馈-期望>180  说明yaw角度从-180阶跃到了180的方向。
		gimbal.yaw.gyroMode.pid.errorOut = gimbal.yaw.gyroMode.pid.errorOut - 360;
		//此数值为正确的误差值。
	}
	else if(gimbal.yaw.gyroMode.pid.errorOut <-180)
	{
		//反馈-期望<0，说明yaw角度从180阶跃到了-180的方向
		gimbal.yaw.gyroMode.pid.errorOut = 360+gimbal.yaw.gyroMode.pid.errorOut;
	}	

	gimbal.yaw.gyroMode.pid.integrateOut += gimbal.yaw.gyroMode.pid.errorOut;
	//积分
	gimbal.yaw.gyroMode.pid.integrateOut = constrain(gimbal.yaw.gyroMode.pid.integrateOut,-gimbal.yaw.gyroMode.pid.integrateMaxOut,gimbal.yaw.gyroMode.pid.integrateMaxOut);
	//积分限幅
	if(((gimbal.yaw.gyroMode.pid.errorOut*gimbal.yaw.gyroMode.pid.integrateOut)<0)&&(abs(gimbal.yaw.gyroMode.pid.integrateOut)>(gimbal.yaw.gyroMode.pid.integrateMaxOut/10.f)))
	{
		gimbal.yaw.gyroMode.pid.integrateOut=0;
	}	
	//清除积分	
	gimbal.yaw.gyroMode.pid.pItermOut = gimbal.yaw.gyroMode.pid.kpOut * gimbal.yaw.gyroMode.pid.errorOut;
	//外环P
	gimbal.yaw.gyroMode.pid.pItermOut = constrain(gimbal.yaw.gyroMode.pid.pItermOut,-GIMBAL_GYRO_YAW_MAX_SPEED,GIMBAL_GYRO_YAW_MAX_SPEED);
//	//外环P限幅
	gimbal.yaw.gyroMode.pid.iItermOut = gimbal.yaw.gyroMode.pid.kiOut * gimbal.yaw.gyroMode.pid.integrateOut;
	//外环I
	gimbal.yaw.gyroMode.pid.iItermOut = constrain(gimbal.yaw.gyroMode.pid.iItermOut,-gimbal.yaw.gyroMode.pid.integrateMaxOut,gimbal.yaw.gyroMode.pid.integrateMaxOut);
	//外环I限幅
	gimbal.yaw.gyroMode.pid.dItermOut = gimbal.yaw.gyroMode.pid.kdOut * (gimbal.yaw.gyroMode.pid.errorOut - gimbal.yaw.gyroMode.pid.lastErrorOut);
	//外环D
	gimbal.yaw.gyroMode.pid.dItermOut = constrain(gimbal.yaw.gyroMode.pid.dItermOut,-2000,2000);
	//外环D限幅
	gimbal.yaw.gyroMode.pid.itermOut = gimbal.yaw.gyroMode.pid.pItermOut + gimbal.yaw.gyroMode.pid.iItermOut + gimbal.yaw.gyroMode.pid.dItermOut;
	//外环输出
	gimbal.yaw.gyroMode.pid.itermOut = constrain(gimbal.yaw.gyroMode.pid.itermOut,-GIMBAL_GYRO_YAW_MAX_SPEED,GIMBAL_GYRO_YAW_MAX_SPEED);
	//输出限幅
	gimbal.yaw.gyroMode.pid.lastErrorOut = gimbal.yaw.gyroMode.pid.errorOut;
	//记录上一次误差
	

	gimbal.yaw.gyroMode.targetGyroZ = -gimbal.yaw.gyroMode.pid.itermOut;
//	



//	gimbal.yaw.gyroMode.targetGyroZ = kLpf*gimbal.yaw.gyroMode.targetGyroZ + (1-kLpf)*lastTargetGyroZ;
//	

	/*	-----------------以上是yaw.gyroMode外环计算过程------------------------	*/
	float feedwardIterm=0;
	//前馈项
	feedwardIterm = kFeedFoward*(gimbal.yaw.gyroMode.targetGyroZ - lastTargetGyroZ);
	//前馈计算
	feedwardIterm = constrain(feedwardIterm,-3000,3000);
	//限幅
	lastTargetGyroZ = gimbal.yaw.gyroMode.targetGyroZ;
	//记录上一次期望值
	
	gimbal.yaw.gyroMode.pid.errorIn = gimbal.yaw.gyroMode.targetGyroZ - gimbal.yaw.gyroMode.measureGyroZ;
	//计算内环误差
	gimbal.yaw.gyroMode.pid.integrateIn += gimbal.yaw.gyroMode.pid.errorIn;
	//内环积分
	gimbal.yaw.gyroMode.pid.integrateIn = constrain(gimbal.yaw.gyroMode.pid.integrateIn,-gimbal.yaw.gyroMode.pid.integrateMaxIn,gimbal.yaw.gyroMode.pid.integrateMaxIn);
	//积分限幅
	if(((gimbal.yaw.gyroMode.pid.errorIn*gimbal.yaw.gyroMode.pid.integrateIn)<0)&&(abs(gimbal.yaw.gyroMode.pid.integrateIn)>(gimbal.yaw.gyroMode.pid.integrateMaxIn/5.f)))
	{
		gimbal.yaw.gyroMode.pid.integrateIn=0;
	}
	gimbal.yaw.gyroMode.pid.pItermIn = gimbal.yaw.gyroMode.pid.kpIn * gimbal.yaw.gyroMode.pid.errorIn;
	//内环P
	gimbal.yaw.gyroMode.pid.pItermIn = constrain(gimbal.yaw.gyroMode.pid.pItermIn,-5000,5000);
	//内环P限幅
	gimbal.yaw.gyroMode.pid.iItermIn = gimbal.yaw.gyroMode.pid.kiIn * gimbal.yaw.gyroMode.pid.integrateIn;
	//内环I
	gimbal.yaw.gyroMode.pid.dItermIn = gimbal.yaw.gyroMode.pid.kdIn * (gimbal.yaw.gyroMode.pid.errorIn - gimbal.yaw.gyroMode.pid.lastErrorIn);
	//内环D
	gimbal.yaw.gyroMode.pid.dItermIn = constrain(gimbal.yaw.gyroMode.pid.dItermIn,-2000,2000);
	//内环D限幅
	gimbal.yaw.gyroMode.pid.itermIn = gimbal.yaw.gyroMode.pid.pItermIn + gimbal.yaw.gyroMode.pid.iItermIn + gimbal.yaw.gyroMode.pid.dItermIn + feedwardIterm;
	//内环输出
	gimbal.yaw.gyroMode.pid.itermIn = constrain(gimbal.yaw.gyroMode.pid.itermIn,-5000,5000);
	//内环输出限幅
	gimbal.yaw.gyroMode.pid.lastErrorIn = gimbal.yaw.gyroMode.pid.errorIn;
	//记录内环误差	
	
		/*	-----------------以上是yaw.gyroMode内环计算过程------------------------	*/
}



/**
  * @brief  Yaw轴自动模式控制函数
  * @param  void
  * @retval void
  * @attention 视觉角度-陀螺仪角速度的串级PID控制
*/
extern float visionYawMathExpect;
//数学期望
extern float visionYawStdDevia;
//标准差
int16_t gyrozTarget=0;
		
//float km=120,ks=20;
float km=0,ks=0;
//系数
extern QueueObj visionQueue;
float compensateGyroYaw=0;
extern float predictAngle;
void YawControlAutoMode(void)
{
	if(gimbal.autoMode.mode == TRACK)
	{
		//跟踪模式
		gimbal.autoMode.yaw.pid.errorOut = visionData.yawAngle-predictAngle;
		//这里要判断清楚方向
		
		gimbal.autoMode.yaw.pid.errorOut = myDeathZoom(0,0.01f,gimbal.autoMode.yaw.pid.errorOut);
		
		gimbal.autoMode.yaw.pid.integrateOut+=gimbal.autoMode.yaw.pid.errorOut;
		//积分
		
		gimbal.autoMode.yaw.pid.integrateOut = constrain(gimbal.autoMode.yaw.pid.integrateOut,-gimbal.autoMode.yaw.pid.integrateMaxOut,gimbal.autoMode.yaw.pid.integrateMaxOut);
		//积分限幅
		
		
		if(((gimbal.autoMode.yaw.pid.errorOut*gimbal.autoMode.yaw.pid.integrateOut)<0)&&(abs(gimbal.autoMode.yaw.pid.integrateOut)>(gimbal.autoMode.yaw.pid.integrateMaxOut/10.f)))
		{
			gimbal.autoMode.yaw.pid.integrateOut=0;
		}	
		
		gimbal.autoMode.yaw.pid.pItermOut = gimbal.autoMode.yaw.pid.kpOut * gimbal.autoMode.yaw.pid.errorOut;
		//p
		
		gimbal.autoMode.yaw.pid.pItermOut = constrain(gimbal.autoMode.yaw.pid.pItermOut,-GIMBAL_GYRO_YAW_MAX_SPEED,GIMBAL_GYRO_YAW_MAX_SPEED);
		//p限幅
		
		
		gimbal.autoMode.yaw.pid.iItermOut = gimbal.autoMode.yaw.pid.kiOut * gimbal.autoMode.yaw.pid.integrateOut;
		//i
		
		gimbal.autoMode.yaw.pid.dItermOut = gimbal.autoMode.yaw.pid.kdOut * (gimbal.autoMode.yaw.pid.errorOut - gimbal.autoMode.yaw.pid.lastErrorOut);
		//d
		
		gimbal.autoMode.yaw.pid.dItermOut = constrain(gimbal.autoMode.yaw.pid.dItermOut,-1000,1000);
		//d限幅
		
		gimbal.autoMode.yaw.pid.itermOut = gimbal.autoMode.yaw.pid.pItermOut + gimbal.autoMode.yaw.pid.iItermOut + gimbal.autoMode.yaw.pid.dItermOut;
		//out
		
		gimbal.autoMode.yaw.pid.itermOut = constrain(gimbal.autoMode.yaw.pid.itermOut,-GIMBAL_GYRO_YAW_MAX_SPEED,GIMBAL_GYRO_YAW_MAX_SPEED);
		//限幅
		
		/*	-----------------以上是yaw.Aim外环计算过程------------------------	*/
		
		

		

		compensateGyroYaw = km*(visionYawMathExpect-predictAngle) + ks*(visionYawStdDevia+predictAngle);
		

		
		
		
		
		
		
		gyrozTarget = gimbal.autoMode.yaw.pid.itermOut+compensateGyroYaw;
		
		gimbal.autoMode.yaw.pid.errorIn = gimbal.autoMode.yaw.pid.itermOut + compensateGyroYaw - gimbal.yaw.gyroMode.measureGyroZ;
		//误差
		
		gimbal.autoMode.yaw.pid.integrateIn += gimbal.autoMode.yaw.pid.errorIn;
		//内环积分
		
		gimbal.autoMode.yaw.pid.integrateIn = constrain(gimbal.autoMode.yaw.pid.integrateIn,-gimbal.autoMode.yaw.pid.integrateMaxIn,gimbal.autoMode.yaw.pid.integrateMaxIn);
		//积分限幅
		
		if(((gimbal.autoMode.yaw.pid.errorIn*gimbal.autoMode.yaw.pid.integrateIn)<0)&&(abs(gimbal.autoMode.yaw.pid.integrateIn)>(gimbal.autoMode.yaw.pid.integrateMaxIn/5.f)))
		{
			gimbal.autoMode.yaw.pid.integrateIn=0;
		}		
		
		gimbal.autoMode.yaw.pid.pItermIn = gimbal.autoMode.yaw.pid.kpIn * gimbal.autoMode.yaw.pid.errorIn;
		//内环P
		
		gimbal.autoMode.yaw.pid.pItermIn = constrain(gimbal.autoMode.yaw.pid.pItermIn,-5000,5000);
		//内环P限幅
		
		gimbal.autoMode.yaw.pid.iItermIn = gimbal.autoMode.yaw.pid.kiIn * gimbal.autoMode.yaw.pid.integrateIn;
		//内环I
		
		gimbal.autoMode.yaw.pid.dItermIn = gimbal.autoMode.yaw.pid.kdIn * (gimbal.autoMode.yaw.pid.errorIn - gimbal.autoMode.yaw.pid.lastErrorIn);
		//内环D
		
		gimbal.autoMode.yaw.pid.dItermIn = constrain(gimbal.autoMode.yaw.pid.dItermIn,-2000,2000);
		//内环D限幅
		
		gimbal.autoMode.yaw.pid.itermIn = gimbal.autoMode.yaw.pid.pItermIn + gimbal.autoMode.yaw.pid.iItermIn + gimbal.autoMode.yaw.pid.dItermIn;
		//内环输出
		
		gimbal.autoMode.yaw.pid.itermIn = constrain(gimbal.autoMode.yaw.pid.itermIn,-5000,5000);
		//内环输出限幅
		
		gimbal.autoMode.yaw.pid.lastErrorIn = gimbal.autoMode.yaw.pid.errorIn;
		//记录内环误差	
		
		
			/*	-----------------以上是yaw.Aim内环计算过程------------------------	*/		
	}
	else if(gimbal.autoMode.mode == AIM)
	{
		//瞄准模式

	}
}





/**
  * @brief  pitch轴自动模式控制函数
  * @param  void
  * @retval void
  * @attention 视觉角度-陀螺仪角速度的串级PID控制
*/
void PitchControlAutoMode(void)
{
	if(gimbal.autoMode.mode == TRACK)
	{
		//跟踪模式
		gimbal.autoMode.pitch.pid.errorOut = visionData.pitchAngle;
		//这里要判断清楚方向
		
		gimbal.autoMode.pitch.pid.errorOut = myDeathZoom(0,0.01f,gimbal.autoMode.pitch.pid.errorOut);
		//死区
		
		if(gimbal.pitch.measureMechAngle>=GIMBAL_PITCH_ANGLE_UP)
		{
			gimbal.autoMode.pitch.pid.errorOut=0;
		
		}
		else if(gimbal.pitch.measureMechAngle<=GIMBAL_PITCH_ANGLE_DOWN)
		{
			gimbal.autoMode.pitch.pid.errorOut=0;
		
		}
		
		
		
		if((gimbal.pitch.measureMechAngle>=GIMBAL_PITCH_ANGLE_UP) || (GIMBAL_PITCH_ANGLE_UP<=GIMBAL_PITCH_ANGLE_DOWN))
		{
			gimbal.autoMode.pitch.pid.errorOut = 0;
		}
		static float lastOut;
		
		float kLPF=0.1f;
		gimbal.autoMode.pitch.pid.integrateOut+=gimbal.autoMode.pitch.pid.errorOut;
		//积分
		
		gimbal.autoMode.pitch.pid.integrateOut = constrain(gimbal.autoMode.pitch.pid.integrateOut,-gimbal.autoMode.pitch.pid.integrateMaxOut,gimbal.autoMode.pitch.pid.integrateMaxOut);
		//积分限幅
		
		gimbal.autoMode.pitch.pid.pItermOut = gimbal.autoMode.pitch.pid.kpOut * gimbal.autoMode.pitch.pid.errorOut;
		//p
		
		gimbal.autoMode.pitch.pid.pItermOut = constrain(gimbal.autoMode.pitch.pid.pItermOut,-GIMBAL_GYRO_YAW_MAX_SPEED,GIMBAL_GYRO_YAW_MAX_SPEED);
		//p限幅
		
		
		gimbal.autoMode.pitch.pid.iItermOut = gimbal.autoMode.pitch.pid.kiOut * gimbal.autoMode.pitch.pid.integrateOut;
		//i
		
		gimbal.autoMode.pitch.pid.dItermOut = gimbal.autoMode.pitch.pid.kdOut * (gimbal.autoMode.pitch.pid.errorOut - gimbal.autoMode.pitch.pid.lastErrorOut);
		//d
		
		gimbal.autoMode.pitch.pid.dItermOut = constrain(gimbal.autoMode.pitch.pid.dItermOut,-1000,1000);
		//d限幅
		
		gimbal.autoMode.pitch.pid.itermOut = gimbal.autoMode.pitch.pid.pItermOut + gimbal.autoMode.pitch.pid.iItermOut + gimbal.autoMode.pitch.pid.dItermOut;
		//out
		

		
		gimbal.autoMode.pitch.pid.itermOut = kLPF*gimbal.autoMode.pitch.pid.itermOut+(1-kLPF)*lastOut;
		
		
		gimbal.autoMode.pitch.pid.itermOut = constrain(gimbal.autoMode.pitch.pid.itermOut,-GIMBAL_PITCH_MAX_SPEED,GIMBAL_PITCH_MAX_SPEED);
		//限幅
		
		lastOut = gimbal.autoMode.pitch.pid.itermOut;
		
		/*	-----------------以上是pitch.Aim外环计算过程------------------------	*/
		

		
		int16_t thrCompensate = 0;
		
	//	thrCompensate = -3.1105f * gimbal.pitch.measureMechAngle + 13951;
	//	thrCompensate = -3.0459f * gimbal.pitch.measureMechAngle + 12795;

		thrCompensate = GetThrCompensate(gimbal.pitch.measureMechAngle);
	
		
		gimbal.autoMode.pitch.pid.errorIn = gimbal.autoMode.pitch.pid.itermOut - gimbal.pitch.measureGyroY;
		//误差
		
		gimbal.autoMode.pitch.pid.integrateIn += gimbal.autoMode.pitch.pid.errorIn;
		//内环积分
		
		gimbal.autoMode.pitch.pid.integrateIn = constrain(gimbal.autoMode.pitch.pid.integrateIn,-gimbal.autoMode.pitch.pid.integrateMaxIn,gimbal.autoMode.pitch.pid.integrateMaxIn);
		//积分限幅
		
		gimbal.autoMode.pitch.pid.pItermIn = gimbal.autoMode.pitch.pid.kpIn * gimbal.autoMode.pitch.pid.errorIn;
		//内环P
		
		gimbal.autoMode.pitch.pid.pItermIn = constrain(gimbal.autoMode.pitch.pid.pItermIn,-5000,5000);
		//内环P限幅
		
		gimbal.autoMode.pitch.pid.iItermIn = gimbal.autoMode.pitch.pid.kiIn * gimbal.autoMode.pitch.pid.integrateIn;
		//内环I
		
		gimbal.autoMode.pitch.pid.dItermIn = gimbal.autoMode.pitch.pid.kdIn * (gimbal.autoMode.pitch.pid.errorIn - gimbal.autoMode.pitch.pid.lastErrorIn);
		//内环D
		
		gimbal.autoMode.pitch.pid.dItermIn = constrain(gimbal.autoMode.pitch.pid.dItermIn,-2000,2000);
		//内环D限幅
		
		gimbal.autoMode.pitch.pid.itermIn = gimbal.autoMode.pitch.pid.pItermIn + gimbal.autoMode.pitch.pid.iItermIn + gimbal.autoMode.pitch.pid.dItermIn;
		//内环输出
		
		gimbal.autoMode.pitch.pid.itermIn+=thrCompensate;
		
		gimbal.autoMode.pitch.pid.itermIn = constrain(gimbal.autoMode.pitch.pid.itermIn,-5000,5000);
		//内环输出限幅
		
		gimbal.autoMode.pitch.pid.lastErrorIn = gimbal.autoMode.pitch.pid.errorIn;
		//记录内环误差			
	
	}
	else if(gimbal.autoMode.mode == AIM)
	{
	
	
	
	}

	
	
}


/**
  * @brief  云台电机输出
  * @param  pitchOutput pitch轴电机输出
						yawOutput  yaw轴电机输出
  * @retval void
	* @attention None
*/
void SendToGimbal(int16_t pitchOutput,int16_t yawOutput)
{
	CAN2_Send(0x1FF,yawOutput,pitchOutput,0,0);
}



/**
  * @brief  云台电机角度数据更新
  * @param yawTmp  yaw轴角度数据更新
  * @retval void
	* @attention None
*/
void GimbalUpdatePosture(int16_t yawTmp,int16_t gyroZ,int16_t gyroY)
{
	gimbal.yaw.gyroMode.measureYawAngle = yawTmp;
	gimbal.yaw.gyroMode.measureGyroZ = gyroZ;
	gimbal.yaw.mechMode.measureGyroZ = gyroZ;
	gimbal.pitch.measureGyroY = gyroY;
}


/*
			3746 - 3770  -  油门 = -15.192*机械角度+58639
			3770 - 3780  -  油门 = -41.667*机械角度+158508
			3780 - 4054  -  油门 = -2.2791*机械角度+9647.5
			4054 - 4078  -  油门 = -11.14*机械角度+45545
			4078 - 4080  -  油门 = 0.975*机械角度+100
			4080 - 4086  -  油门 = 0.9747*机械角度+97.468
			4086 - 4088  -  油门 = -100*机械角度+408500
			4088 - 4259  -  油门 = -2.4952*机械角度+9851.2
			4259 - 4752  -  油门 = -2.1127*机械角度+8216.3
*/
int16_t GetThrCompensate(int16_t angle)
{
	int16_t tmp;

	if(angle<=GIMBAL_PITCH_ANGLE_DOWN)
	{
		tmp = -2.2791f*GIMBAL_PITCH_ANGLE_DOWN + 9647.5f;
	}
	else if((angle>GIMBAL_PITCH_ANGLE_DOWN)&&(angle<=4054))
	{
		tmp = -2.2791f*angle + 9647.5f;
	}
	else if((angle>4054)&&(angle<=4078))
	{
		tmp = -11.14f*angle + 45545;
	}
	else if((angle>4078)&&(angle<=4080))
	{
		tmp = -50.f*angle + 204000;
	}
	else if((angle>4080)&&(angle<=4086))
	{
		tmp = -16.667f*angle + 68000;
	}	
	else if((angle>4086)&&(angle<=4088))
	{
		tmp = -100.f*angle + 408500;
	}
	else if((angle>4088)&&(angle<=4259))
	{
		tmp = -2.4952f*angle + 9851.2f;
	}
	else if((angle>4259)&&(angle<=GIMBAL_PITCH_ANGLE_UP))
	{
		tmp = -2.1127f*angle + 8216.3f;
	}
	else
		tmp = -2.1127f*GIMBAL_PITCH_ANGLE_UP + 8216.3f;
	
	return tmp;
}

int16_t TargetCoast(int16_t goalTarget,int16_t currentTarget)
{
	uint16_t step=1;
	
	
	if(abs(goalTarget - currentTarget)<step)
	{
		//差值绝对值小于步长
		currentTarget = goalTarget;
	}
	else
	{
		//差值大于步长
		if(goalTarget>currentTarget)
			currentTarget+=step;
		else
			currentTarget-=step;
	}
	return currentTarget;
}


