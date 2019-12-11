#include "shoot.h"


ShootCtrlObj shoot;




void ShootInit(void)
{
	shoot.ctrl = ShootCtrl;
	shoot.shootFreq = 0;
	shoot.mode = SINGLE_SHOOT;
	//默认点射模式
	
	shoot.turnplate.ctrl=TurnplateCtrl;
	//控制函数
	shoot.turnplate.status=NORMAL;
	shoot.turnplate.measureAngle=0;
	shoot.turnplate.measureSpeed=0;
	shoot.turnplate.targetAngle=0;
	shoot.turnplate.targetSpeed=0;
	//参数初始化
	shoot.turnplate.pid.kp=18; //18
	shoot.turnplate.pid.ki=0;
	shoot.turnplate.pid.kd=0;
	shoot.turnplate.pid.error=0;
	shoot.turnplate.pid.lastError=0;
	shoot.turnplate.pid.integrate=0;
	shoot.turnplate.pid.integrateMax=0;
	shoot.turnplate.pid.pIterm=0;
	shoot.turnplate.pid.iIterm=0;
	shoot.turnplate.pid.dIterm=0;
	shoot.turnplate.pid.iterm=0;
	//拨盘PID参数
	
	shoot.friction.ctrl=FrictionCtrl;
	shoot.friction.currentPWM=0;
	shoot.friction.isReadyFlag=false;
	shoot.friction.status=false;
	shoot.friction.targetPWM=0;
}


void ShootCtrl(void)
{
	shoot.friction.ctrl();
	shoot.turnplate.ctrl();
}


void TurnplateCtrl(void)
{
	
	static uint32_t sample=0,lastSample=0;
	int32_t error=0;

	lastSample = sample;
	sample = shoot.turnplate.measureAngle;
	error = sample - lastSample;
	//计算误差方向
	


	if(shoot.friction.isReadyFlag == false)
	{
		shoot.shootFreq=0;
		shoot.turnplate.targetAngle=0;
		shoot.turnplate.targetSpeed=0;
	}
	//摩擦轮未准备好则不允许发弹，调试过程先注释掉
	
	
	if(error<0)
	{
		error = 8191 - lastSample + sample;
	}
	

	//未卡住 则正常发弹
	if(shoot.mode == SINGLE_SHOOT)
	{
		//单点模式 -- 控制发射数量
		
		/* 计算公式
			拨一个子弹为1/12圈，又一圈为转子36圈，故一个子弹是转子转3圈。
			
			又一圈机械角度为8191，则需要转过的角度值为8191*3=24573
			
			因此转过的角度值 = 发射个数*24573
		
			发弹的转速期望应该根据具体的调试效果。应协调好加速减速的时间，过快会导致采样失效，过慢会使发弹迟钝。
		*/
		if(shoot.turnplate.targetAngle>abs(error))
		{		
			shoot.turnplate.targetAngle -= abs(error);
			shoot.shootFreq = 10;
		}
		else
		{
			shoot.turnplate.targetAngle = 0;
			shoot.shootFreq = 0;
		}		
	}
	else
		shoot.turnplate.targetAngle=0;	
	//清空点射时残留的期望，防止bug
	
	
		//连射模式 -- 控制射击频率。   
	
	/*	计算公式	
		转速的单位是rpm  转子每分钟的转速
		
		期望转速 = 射击频率 （个/s） / 12 (12个/圈) * 36 （1：36减速比） * 60（s/min）
		
						 = 射击频率 * 180
	
		例：射频为12，即1s内要发射12个子弹，即1s要转一整圈，即转子1s要转36圈，即转子1min要转36*60圈，即转速为2160rpm
	*/
	shoot.turnplate.targetSpeed = shoot.shootFreq * 180;
	//计算期望转速
	
	
	
	/*									以下是PID计算过程 -- 拨盘电机的速度环控制*/
	shoot.turnplate.pid.error = shoot.turnplate.targetSpeed - shoot.turnplate.measureSpeed;
	//计算误差
	shoot.turnplate.pid.integrate += shoot.turnplate.pid.error;
	//积分
	shoot.turnplate.pid.integrate = constrain(shoot.turnplate.pid.integrate,-shoot.turnplate.pid.integrateMax,shoot.turnplate.pid.integrateMax);
	//积分限幅
	
	shoot.turnplate.pid.pIterm = shoot.turnplate.pid.kp * shoot.turnplate.pid.error;
	//p项
	shoot.turnplate.pid.pIterm = constrain(shoot.turnplate.pid.pIterm,-8000,8000);
	//p项限幅
	shoot.turnplate.pid.iIterm = shoot.turnplate.pid.ki * shoot.turnplate.pid.integrate;
	//i项输出
	shoot.turnplate.pid.dIterm = shoot.turnplate.pid.kd * (shoot.turnplate.pid.error - shoot.turnplate.pid.lastError);
	//d项
	shoot.turnplate.pid.dIterm = constrain(shoot.turnplate.pid.dIterm,-2000,2000);
	//d项限幅
	shoot.turnplate.pid.iterm = shoot.turnplate.pid.pIterm + shoot.turnplate.pid.iIterm + shoot.turnplate.pid.dIterm;
	//
	shoot.turnplate.pid.iterm = constrain(shoot.turnplate.pid.iterm,-10000,10000);
	//
	shoot.turnplate.pid.lastError = shoot.turnplate.pid.error;
	
}



void FrictionCtrl(void)
{
	if(shoot.friction.status == true)
	{
		shoot.friction.targetPWM = 685;
		if(shoot.friction.targetPWM == shoot.friction.currentPWM)
		{
			shoot.friction.isReadyFlag = true;
		}		
		else
		{
			shoot.friction.isReadyFlag = false;
		}
	}
	else
	{
		shoot.friction.targetPWM = 0;
		shoot.friction.isReadyFlag = false;
	}
	FrictionOutput(shoot.friction.targetPWM,&shoot.friction.currentPWM);
		//设置摩擦轮转速

}



void SendToTurnplate(int16_t output)
{
	CAN2_Send(0x200,output,0,0,0);
}


