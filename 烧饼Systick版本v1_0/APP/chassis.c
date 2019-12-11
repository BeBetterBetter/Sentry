#include "chassis.h"


ChassisCtrlObj chassis;


void ChassisInit(void)
{
	chassis.motor.ctrl = ChassisSpeedCtrl;
	//电机控制函数初始化
	chassis.motor.measureSpeed=0;
	chassis.motor.targetSpeed=0;
	//速度初始化
	chassis.motor.pid.kp=8;
	chassis.motor.pid.ki=0.6f;
	chassis.motor.pid.kd=0;
	chassis.motor.pid.error=0;
	chassis.motor.pid.lastError=0;
	chassis.motor.pid.integrate=0;
	chassis.motor.pid.integrateMax=10000;
	chassis.motor.pid.pIterm=0;
	chassis.motor.pid.iIterm=0;
	chassis.motor.pid.dIterm=0;
	chassis.motor.pid.iterm=0;
	//底盘PID控制参数初始化
}

/**
  * @brief  底盘电机速度环控制
  * @param  void
  * @retval void
  * @attention 底盘电机速度环控制
*/
void ChassisSpeedCtrl(void)
{
	chassis.motor.pid.error = chassis.motor.targetSpeed - chassis.motor.measureSpeed;
	//计算误差
	
	chassis.motor.pid.integrate += chassis.motor.pid.error;
	//积分
	chassis.motor.pid.integrate = constrain(chassis.motor.pid.integrate,-chassis.motor.pid.integrateMax,chassis.motor.pid.integrateMax);
	//积分限幅
	chassis.motor.pid.pIterm = chassis.motor.pid.kp * chassis.motor.pid.error;
	//p项
	chassis.motor.pid.pIterm = constrain(chassis.motor.pid.pIterm,-CHASSIS_MAX_OUTPUT,CHASSIS_MAX_OUTPUT);
	//p项限幅
	chassis.motor.pid.iIterm = chassis.motor.pid.ki * chassis.motor.pid.integrate;
	//i项
	chassis.motor.pid.dIterm = chassis.motor.pid.kd * (chassis.motor.pid.error - chassis.motor.pid.lastError);
	//d项
	chassis.motor.pid.dIterm = constrain(chassis.motor.pid.dIterm,-5000,5000);
	//d项限幅
	
	chassis.motor.pid.iterm = chassis.motor.pid.pIterm + chassis.motor.pid.iIterm + chassis.motor.pid.dIterm;
	//总输出
	
	chassis.motor.pid.iterm = constrain(chassis.motor.pid.iterm,-CHASSIS_MAX_OUTPUT,CHASSIS_MAX_OUTPUT);
	//输出限幅
	
	chassis.motor.pid.lastError = chassis.motor.pid.error;
	//记录误差
}




void SendToChassis(int16_t motorOutput)
{
	CAN2_Send(0x200,0,0,motorOutput,0);
}













