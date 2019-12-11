#include "vision.h"

VisionRxDataObj visionData;
//视觉数据结构体

QueueObj visionQueue=
{
	.nowLength =0,
	.queueLength = 100,
	.queue = {0}
};

extern extKalman_t kalman_visionPitch;
extern extKalman_t kalman_visionYaw;
extern GimbalCtrlObj gimbal;

short visionYaw=0;
short visionPitch=0;

bool visionFlag=false;

float visionYawMathExpect;
//数学期望
float visionYawStdDevia;
//标准差
extern float compensateGyroYaw;


uint16_t visionAliveCnt=0;
bool visionStatus=false;

void VisionTask(void)
{
	visionAliveCnt++;
	if(visionAliveCnt>=200)
	{
		visionAliveCnt=500;
		visionStatus=false;
		gimbal.autoMode.yaw.pid.integrateIn=0;
		gimbal.autoMode.yaw.pid.integrateOut=0;
		
		
		gimbal.autoMode.pitch.pid.integrateIn=0;
		gimbal.autoMode.pitch.pid.integrateOut=0;		
		visionFlag=false;
		
		for(uint16_t z=0;z<visionQueue.queueLength;z++)
			visionQueue.queue[z] =0.f;
		visionQueue.nowLength=0;
		visionYawMathExpect=0;
		visionYawStdDevia=0;
		compensateGyroYaw=0;		
	}
}

/**
  * @brief  视觉处理函数
  * @param  void
  * @retval void
  * @attention 视觉数据的处理函数，主要用于处理掉帧情况以及预测数据
	·连续丢失四帧数据，认为视觉丢失。
*/
void VisionProcess(void)
{
//	static bool lastVisionFlag;
//	if((visionData.pitchAngle==0.f)&&(visionData.yawAngle==0.f)&&(visionData.distance==0.f))
	if(visionData.identifyTarget == 0)
	{
		//未识别到
		//防止数据溢出		
		gimbal.autoMode.yaw.pid.integrateIn=0;
		gimbal.autoMode.yaw.pid.integrateOut=0;
		gimbal.autoMode.pitch.pid.integrateIn=0;
		gimbal.autoMode.pitch.pid.integrateOut=0;
		visionFlag=false;
		
		for(uint16_t z=0;z<visionQueue.queueLength;z++)
			visionQueue.queue[z] =0.f;
		visionQueue.nowLength=0;
		visionYawMathExpect=0;
		visionYawStdDevia=0;
		compensateGyroYaw=0;
//		if(visionFlag!=lastVisionFlag)
//		{
//			gimbal.pitch.targetMechAngle = gimbal.pitch.measureMechAngle;
//			gimbal.yaw.gyroMode.targetYawAngle = gimbal.yaw.gyroMode.measureYawAngle;
//			gimbal.yaw.mechMode.targetMechAngle = gimbal.yaw.mechMode.measureMechAngle;
//		}
	}
	else
	{
		GetPredictAngle();
		visionFlag=true;

		UpdateVisionInfo(visionData.yawAngle,50,&visionYawMathExpect,&visionYawStdDevia);
		//更新视觉信息
	}
	visionAliveCnt=0;
	visionStatus=true;
//	lastVisionFlag=visionFlag;
}



/*
	记录视觉前几帧的信息。
*/
void UpdateVisionInfo(float input,uint16_t length,float *mathExpect,float *standardDeviation)
{
	float sum=0;

	
	if(length>visionQueue.queueLength)
		length=visionQueue.queueLength;
	//防止溢出
	
	
	if(visionQueue.nowLength<length)
	{
		//队列未满，只进不出
		visionQueue.queue[visionQueue.nowLength] = input;
		visionQueue.nowLength++;
	}
	else
	{
		//队列已满，FIFO。
		for(uint16_t i=0;i<length-1;i++)
		{
			visionQueue.queue[i] = visionQueue.queue[i+1];
			//更新队列
		
		}
		visionQueue.queue[length-1] = input;
	}
	
	//更新完队列
	
	
	for(uint16_t j=0;j<length;j++)
	{
		sum+=visionQueue.queue[j];
	}
	*mathExpect = sum/(length/1.f);
	
	*standardDeviation = (input - *mathExpect);
}

extern extKalman_t kalman_visionYaw;
float predictAngle;
float kpre=0;//12
float tmpTargetSpeed;
float deltaYaw1=0,deltaYaw2=0;
void GetPredictAngle(void)
{
	float tmp;
	static float yaw1=0,lastYaw1=0,yaw2=0,lastYaw2=0;
	static float targetAngle=0;
	
	yaw1 = gimbal.yaw.mechMode.measureMechAngle;
	yaw2 = visionData.yawAngle;
	
	deltaYaw1 = yaw1 - lastYaw1;
	if(abs(deltaYaw1) >=4096)
	{
		//计算变化量
		if(deltaYaw1>0)
		{
			deltaYaw1 = deltaYaw1 - 8191;
		}
		else 
		{
			deltaYaw1 = deltaYaw1 + 8191;
		}
		
	}
	deltaYaw1 = deltaYaw1/22.75f;
	//单位转化
	
	deltaYaw2 = yaw2 - lastYaw2;
	
	lastYaw1=yaw1;
	lastYaw2=yaw2;
	
	
	tmp = deltaYaw1 + deltaYaw2;
	
	tmpTargetSpeed = KalmanFilter(&kalman_visionYaw,tmp);
	
	tmpTargetSpeed = myDeathZoom(0,0.01f,tmpTargetSpeed);
	
//	predictAngle = -tmpTargetSpeed*kpre;
	
//	predictAngle = myDeathZoom(0,0.2f,predictAngle);
	
	if(tmpTargetSpeed>0.1)
		targetAngle = -4;
	else if(tmpTargetSpeed<-0.1)
		targetAngle = 4;
	else
		targetAngle = 0;
	
	
	predictAngle = RampFloat(0.15f,targetAngle,predictAngle);
	
//	predictAngle = constrain(predictAngle,-6,6);

	

	if(visionQueue.nowLength<50)
	{
		predictAngle=0;
	}
}








void Aim(void)
{




}









