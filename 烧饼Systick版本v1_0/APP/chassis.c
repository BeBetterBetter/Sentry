#include "chassis.h"


ChassisCtrlObj chassis;


void ChassisInit(void)
{
	chassis.motor.ctrl = ChassisSpeedCtrl;
	//������ƺ�����ʼ��
	chassis.motor.measureSpeed=0;
	chassis.motor.targetSpeed=0;
	//�ٶȳ�ʼ��
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
	//����PID���Ʋ�����ʼ��
}

/**
  * @brief  ���̵���ٶȻ�����
  * @param  void
  * @retval void
  * @attention ���̵���ٶȻ�����
*/
void ChassisSpeedCtrl(void)
{
	chassis.motor.pid.error = chassis.motor.targetSpeed - chassis.motor.measureSpeed;
	//�������
	
	chassis.motor.pid.integrate += chassis.motor.pid.error;
	//����
	chassis.motor.pid.integrate = constrain(chassis.motor.pid.integrate,-chassis.motor.pid.integrateMax,chassis.motor.pid.integrateMax);
	//�����޷�
	chassis.motor.pid.pIterm = chassis.motor.pid.kp * chassis.motor.pid.error;
	//p��
	chassis.motor.pid.pIterm = constrain(chassis.motor.pid.pIterm,-CHASSIS_MAX_OUTPUT,CHASSIS_MAX_OUTPUT);
	//p���޷�
	chassis.motor.pid.iIterm = chassis.motor.pid.ki * chassis.motor.pid.integrate;
	//i��
	chassis.motor.pid.dIterm = chassis.motor.pid.kd * (chassis.motor.pid.error - chassis.motor.pid.lastError);
	//d��
	chassis.motor.pid.dIterm = constrain(chassis.motor.pid.dIterm,-5000,5000);
	//d���޷�
	
	chassis.motor.pid.iterm = chassis.motor.pid.pIterm + chassis.motor.pid.iIterm + chassis.motor.pid.dIterm;
	//�����
	
	chassis.motor.pid.iterm = constrain(chassis.motor.pid.iterm,-CHASSIS_MAX_OUTPUT,CHASSIS_MAX_OUTPUT);
	//����޷�
	
	chassis.motor.pid.lastError = chassis.motor.pid.error;
	//��¼���
}




void SendToChassis(int16_t motorOutput)
{
	CAN2_Send(0x200,0,0,motorOutput,0);
}













