#include "rc.h"
RemoteCtrlObj rc;
extern ChassisCtrlObj chassis;
extern GimbalCtrlObj gimbal;
extern ShootCtrlObj shoot;
//ң�ض���
/**
* @brief ��ʱ����
* @param void
* @return void
*	��������Ҫ���жϺ����е��ã�����ǲ���ʹ�õ��ö�ʱ������ʱ��
*/
static void delay_ms1(unsigned int t)
{
	int i;
	for( i=0;i<t;i++)
	{
		int a=42000;
		while(a--);
	}
}

/**
* @brief ң������ʼ������
* @param void
* @return void
*/
void RemotrCtrlInit(void)
{
	USART2_Init();
	//Ӳ����ʼ��
	rc.rcRx = RemoteCtrlRecieve;
	rc.rcProtocol = RemoteCtrlProtocol;
	//����ָ��
	
	rc.info.rc.ch0 = RC_CH0_3_MID_VALUE;
	rc.info.rc.ch1 = RC_CH0_3_MID_VALUE;
	rc.info.rc.ch2 = RC_CH0_3_MID_VALUE;
	rc.info.rc.ch3 = RC_CH0_3_MID_VALUE;
	rc.info.rc.s1 = RC_S1_2_UP_VALUE;
	rc.info.rc.s2 = RC_S1_2_MID_VALUE;
	rc.info.mouse.press_l = RC_MOUSE_RELEASE_VALUE;
	rc.info.mouse.press_r = RC_MOUSE_RELEASE_VALUE;
	rc.info.mouse.x = 0;
	rc.info.mouse.y = 0;
	rc.info.mouse.z =	0;
	rc.info.key.v = 0x0000;
	//ң����ֵ��ʼ��
	
	rc.rcAliveCnt=0;
	//����ֵ��ʼ��
	rc.status=false;
	//Ĭ������
}

/**
* @brief ң�������ݽ��պ���
* ����ң��������
* @param void
* @return void
*/
void RemoteCtrlRecieve(void)
{
	rc.info.rc.ch0 = (rc.rxBuf[0]| (rc.rxBuf[1] << 8)) & 0x07ff; //!< Channel 0
	rc.info.rc.ch1 = ((rc.rxBuf[1] >> 3) | (rc.rxBuf[2] << 5)) & 0x07ff; //!< Channel 1
	rc.info.rc.ch2 = ((rc.rxBuf[2] >> 6) | (rc.rxBuf[3] << 2) | (rc.rxBuf[4] << 10)) & 0x07ff;//!< Channel 2
	rc.info.rc.ch3 = ((rc.rxBuf[4] >> 1) | (rc.rxBuf[5] << 7)) & 0x07ff; //!< Channel 3
	rc.info.rc.s1 = ((rc.rxBuf[5] >> 4)& 0x000C) >> 2; //!< Switch left
	rc.info.rc.s2 = ((rc.rxBuf[5] >> 4)& 0x0003); //!< Switch right	
	rc.info.mouse.x = rc.rxBuf[6] | (rc.rxBuf[7] << 8); //!< Mouse X axis
	rc.info.mouse.y = rc.rxBuf[8] | (rc.rxBuf[9] << 8); //!< Mouse Y axis
	rc.info.mouse.z = rc.rxBuf[10] | (rc.rxBuf[11] << 8); //!< Mouse Z axis
	rc.info.mouse.press_l = rc.rxBuf[12]; //!< Mouse Left Is Press ?
	rc.info.mouse.press_r = rc.rxBuf[13]; //!< Mouse Right Is Press ?
	rc.info.key.v = rc.rxBuf[14] | (rc.rxBuf[15] << 8); //!< KeyBoard value
		
	if((rc.info.rc.s1 != 1)&&(rc.info.rc.s1 != 2)&&(rc.info.rc.s1 != 3))
	{
		delay_ms1(1);
		//reset CPU
		__set_FAULTMASK(1);
		NVIC_SystemReset();
	}
	
	rc.updateFlag=true;
	//���±�־λ
}


void RemoteCtrlProtocol(void)
{
	if(rc.updateFlag==true)
	{
		//�������������
		GimbalRemoteCtrlProtocol();
		rc.rcAliveCnt=0;
		rc.status=true;
		rc.updateFlag=false;
		//����״̬
	}
	else
	{
		//����û���յ���
		rc.rcAliveCnt+=2;
		if(rc.rcAliveCnt>=RC_OFFLINE_VALUE)
		{
			rc.info.rc.ch0 = RC_CH0_3_MID_VALUE; //!< Channel 0
			rc.info.rc.ch1 = RC_CH0_3_MID_VALUE; //!< Channel 1
			rc.info.rc.ch2 = RC_CH0_3_MID_VALUE;//!< Channel 2
			rc.info.rc.ch3 = RC_CH0_3_MID_VALUE; //!< Channel 3
			rc.info.rc.s1 = RC_S1_2_MID_VALUE; //!< Switch left
			rc.info.rc.s2 = RC_S1_2_MID_VALUE; //!< Switch right	
			rc.info.mouse.x = 0; //!< Mouse X axis
			rc.info.mouse.y = 0; //!< Mouse Y axis
			rc.info.mouse.z = 0; //!< Mouse Z axis
			rc.info.mouse.press_l = RC_MOUSE_RELEASE_VALUE; //!< Mouse Left Is Press ?
			rc.info.mouse.press_r = RC_MOUSE_RELEASE_VALUE; //!< Mouse Right Is Press ?
			rc.info.key.v = 0x00; //!< KeyBoard value			
			//�����е���ֵ��Ϊ��ʼֵ
			rc.rcAliveCnt=RC_OFFLINE_VALUE;
			//��ֹ���
			rc.status = false;
			//����
		}
	}
}
int16_t target = 0;
/**
* @brief ��̨ң��Э����ƺ���
* ��̨ң��Э��
* @param void
* @return void
	@pitch - gyrox���������������Ǹ�   ��е�Ƕ� �����Ǹ� ���ܳ������߽�ֵ��
	@yaw - gyroz��ʱ���Ǹ�����е�Ƕ���ʱ���Ǹ����������¿�
*/
extern bool visionFlag;
void GimbalRemoteCtrlProtocol(void)
{
	static uint8_t s1,s2,lastS1=RC_S1_2_UP_VALUE,lastS2;
	float ch0,ch1,ch2,ch3;
	
	ch0 = (myDeathZoom_int(RC_CH0_3_MID_VALUE,10,rc.info.rc.ch0)-RC_CH0_3_MID_VALUE)/RC_CH0_3_WID_VALUE;
	ch1 = (myDeathZoom_int(RC_CH0_3_MID_VALUE,10,rc.info.rc.ch1)-RC_CH0_3_MID_VALUE)/RC_CH0_3_WID_VALUE;
	ch2 = (myDeathZoom_int(RC_CH0_3_MID_VALUE,10,rc.info.rc.ch2)-RC_CH0_3_MID_VALUE)/RC_CH0_3_WID_VALUE;
	ch3 = (myDeathZoom_int(RC_CH0_3_MID_VALUE,10,rc.info.rc.ch3)-RC_CH0_3_MID_VALUE)/RC_CH0_3_WID_VALUE;
	//��������ϵ��
	
	s1 = rc.info.rc.s1;
	s2 = rc.info.rc.s2;	
	//��������

	if(s1 == RC_S1_2_UP_VALUE)
	{
		//��

			
		if(s1!=lastS1)
		{
			if(shoot.friction.status == true)
				shoot.friction.status = false;
			else
				shoot.friction.status = true;
		}
	
	}
	else if(s1 == RC_S1_2_MID_VALUE) 
	{
		//��
		shoot.shootFreq = 0;	
	}
	else
	{
		//��
		if(shoot.mode == SINGLE_SHOOT)
		{
			//����
			if(s1!=lastS1)
			{
				shoot.turnplate.targetAngle += SHOOT_PER_ANGLE;
			}
		}
		else
		{
			//����
			shoot.shootFreq = 10;			
		}
	}
	
	
	if(s2 == RC_S1_2_UP_VALUE)
	{
		//��
		gimbal.mode = MECHANICAL_MODE;
		if(s2!=lastS2)
		{
			gimbal.yaw.mechMode.targetMechAngle = gimbal.yaw.mechMode.measureMechAngle;
			gimbal.pitch.targetMechAngle = gimbal.pitch.measureMechAngle;
		}
		shoot.mode = SINGLE_SHOOT;
	}
	else if(s2 == RC_S1_2_MID_VALUE) 
	{
		//��
		gimbal.mode = GYRO_MODE;
		if(s2!=lastS2)
		{
			gimbal.yaw.gyroMode.targetYawAngle = gimbal.yaw.gyroMode.measureYawAngle;
			gimbal.pitch.targetMechAngle = gimbal.pitch.measureMechAngle;
		}
		shoot.mode = CONTINUS_SHOOT;
	}
	else
	{
		//��
		gimbal.mode = AUTO_MODE;
		if(s2!=lastS2)
		{
			gimbal.yaw.mechMode.targetMechAngle = gimbal.yaw.mechMode.measureMechAngle;
			gimbal.yaw.gyroMode.targetYawAngle = gimbal.yaw.gyroMode.measureYawAngle;
			gimbal.pitch.targetMechAngle = gimbal.pitch.measureMechAngle;
		}
		shoot.mode = SINGLE_SHOOT;
	}
	
//	if(gimbal.mode != AUTO_MODE) //���Զ�ģʽ��
	{
//		target = -ch1*GIMBAL_PITCH_MAX_SPEED;
//		gimbal.pitch.targetGyroY = target;
		
		gimbal.pitch.targetMechAngle += ch1*15;	
		if(gimbal.pitch.targetMechAngle >= GIMBAL_PITCH_ANGLE_UP)
			gimbal.pitch.targetMechAngle = GIMBAL_PITCH_ANGLE_UP;
		else if(gimbal.pitch.targetMechAngle <= GIMBAL_PITCH_ANGLE_DOWN)
			gimbal.pitch.targetMechAngle = GIMBAL_PITCH_ANGLE_DOWN;
		target = gimbal.pitch.targetMechAngle;
	}
//	else
	{
//		if(visionFlag==false)
//		{
//			gimbal.pitch.targetMechAngle += ch1*15;	
//			if(gimbal.pitch.targetMechAngle >= GIMBAL_PITCH_ANGLE_UP)
//				gimbal.pitch.targetMechAngle = GIMBAL_PITCH_ANGLE_UP;
//			else if(gimbal.pitch.targetMechAngle <= GIMBAL_PITCH_ANGLE_DOWN)
//				gimbal.pitch.targetMechAngle = GIMBAL_PITCH_ANGLE_DOWN;
//			target = gimbal.pitch.targetMechAngle;
//		}
	
	}

	
	if(gimbal.mode == MECHANICAL_MODE)
	{
		//��еģʽ
		gimbal.yaw.mechMode.targetMechAngle += ch0*56.f;
	
		if(gimbal.yaw.mechMode.targetMechAngle>8191)
		{
			gimbal.yaw.mechMode.targetMechAngle=gimbal.yaw.mechMode.targetMechAngle-8191;
		}
		else if(gimbal.yaw.mechMode.targetMechAngle<0)
		{
			gimbal.yaw.mechMode.targetMechAngle = 8191 - abs(gimbal.yaw.mechMode.targetMechAngle);
			
		}	
		
//		gimbal.yaw.mechMode.targetGyroZ = ch0*GIMBAL_GYRO_YAW_MAX_SPEED;
//		target = gimbal.yaw.mechMode.targetGyroZ;
//		target = gimbal.yaw.mechMode.targetMechAngle;	
	}
	else if(gimbal.mode == GYRO_MODE)
	{
		//������ģʽ
		gimbal.yaw.gyroMode.targetYawAngle += -ch0*2.5f;
		
		if(gimbal.yaw.gyroMode.targetYawAngle > 179.9f)
		{
			gimbal.yaw.gyroMode.targetYawAngle = (gimbal.yaw.gyroMode.targetYawAngle-180) - 179.9f;
		}
		else if(gimbal.yaw.gyroMode.targetYawAngle < -179.9f)
		{
			gimbal.yaw.gyroMode.targetYawAngle = 179.9f - (180 + gimbal.yaw.gyroMode.targetYawAngle);
		}
//		gimbal.yaw.gyroMode.targetGyroZ = ch0*GIMBAL_GYRO_YAW_MAX_SPEED;
//		target = gimbal.yaw.gyroMode.targetGyroZ;
//		target = gimbal.yaw.gyroMode.targetYawAngle;
	}
	else
	{
		if(visionFlag==false)
		{
			gimbal.yaw.gyroMode.targetYawAngle += -ch0*2.5f;
			
			if(gimbal.yaw.gyroMode.targetYawAngle > 179.9f)
			{
				gimbal.yaw.gyroMode.targetYawAngle = (gimbal.yaw.gyroMode.targetYawAngle-180) - 179.9f;
			}
			else if(gimbal.yaw.gyroMode.targetYawAngle < -179.9f)
			{
				gimbal.yaw.gyroMode.targetYawAngle = 179.9f - (180 + gimbal.yaw.gyroMode.targetYawAngle);
			}		
			
		}

	}
	
	
	if(gimbal.mode != AUTO_MODE)
		chassis.motor.targetSpeed = ch2 * CHASSIS_MAX_SPEED;
	else //�Զ�ģʽ����ʱ����
		chassis.motor.targetSpeed = 0;
	//���̿���
	
	chassis.motor.targetSpeed = 0;
	
	
	lastS1 = s1;
	lastS2 = s2;
	//��¼��һ��״̬
}


