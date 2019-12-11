#include "shoot.h"


ShootCtrlObj shoot;




void ShootInit(void)
{
	shoot.ctrl = ShootCtrl;
	shoot.shootFreq = 0;
	shoot.mode = SINGLE_SHOOT;
	//Ĭ�ϵ���ģʽ
	
	shoot.turnplate.ctrl=TurnplateCtrl;
	//���ƺ���
	shoot.turnplate.status=NORMAL;
	shoot.turnplate.measureAngle=0;
	shoot.turnplate.measureSpeed=0;
	shoot.turnplate.targetAngle=0;
	shoot.turnplate.targetSpeed=0;
	//������ʼ��
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
	//����PID����
	
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
	//��������
	


	if(shoot.friction.isReadyFlag == false)
	{
		shoot.shootFreq=0;
		shoot.turnplate.targetAngle=0;
		shoot.turnplate.targetSpeed=0;
	}
	//Ħ����δ׼�����������������Թ�����ע�͵�
	
	
	if(error<0)
	{
		error = 8191 - lastSample + sample;
	}
	

	//δ��ס ����������
	if(shoot.mode == SINGLE_SHOOT)
	{
		//����ģʽ -- ���Ʒ�������
		
		/* ���㹫ʽ
			��һ���ӵ�Ϊ1/12Ȧ����һȦΪת��36Ȧ����һ���ӵ���ת��ת3Ȧ��
			
			��һȦ��е�Ƕ�Ϊ8191������Ҫת���ĽǶ�ֵΪ8191*3=24573
			
			���ת���ĽǶ�ֵ = �������*24573
		
			������ת������Ӧ�ø��ݾ���ĵ���Ч����ӦЭ���ü��ټ��ٵ�ʱ�䣬����ᵼ�²���ʧЧ��������ʹ�����ٶۡ�
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
	//��յ���ʱ��������������ֹbug
	
	
		//����ģʽ -- �������Ƶ�ʡ�   
	
	/*	���㹫ʽ	
		ת�ٵĵ�λ��rpm  ת��ÿ���ӵ�ת��
		
		����ת�� = ���Ƶ�� ����/s�� / 12 (12��/Ȧ) * 36 ��1��36���ٱȣ� * 60��s/min��
		
						 = ���Ƶ�� * 180
	
		������ƵΪ12����1s��Ҫ����12���ӵ�����1sҪתһ��Ȧ����ת��1sҪת36Ȧ����ת��1minҪת36*60Ȧ����ת��Ϊ2160rpm
	*/
	shoot.turnplate.targetSpeed = shoot.shootFreq * 180;
	//��������ת��
	
	
	
	/*									������PID������� -- ���̵�����ٶȻ�����*/
	shoot.turnplate.pid.error = shoot.turnplate.targetSpeed - shoot.turnplate.measureSpeed;
	//�������
	shoot.turnplate.pid.integrate += shoot.turnplate.pid.error;
	//����
	shoot.turnplate.pid.integrate = constrain(shoot.turnplate.pid.integrate,-shoot.turnplate.pid.integrateMax,shoot.turnplate.pid.integrateMax);
	//�����޷�
	
	shoot.turnplate.pid.pIterm = shoot.turnplate.pid.kp * shoot.turnplate.pid.error;
	//p��
	shoot.turnplate.pid.pIterm = constrain(shoot.turnplate.pid.pIterm,-8000,8000);
	//p���޷�
	shoot.turnplate.pid.iIterm = shoot.turnplate.pid.ki * shoot.turnplate.pid.integrate;
	//i�����
	shoot.turnplate.pid.dIterm = shoot.turnplate.pid.kd * (shoot.turnplate.pid.error - shoot.turnplate.pid.lastError);
	//d��
	shoot.turnplate.pid.dIterm = constrain(shoot.turnplate.pid.dIterm,-2000,2000);
	//d���޷�
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
		//����Ħ����ת��

}



void SendToTurnplate(int16_t output)
{
	CAN2_Send(0x200,output,0,0,0);
}


