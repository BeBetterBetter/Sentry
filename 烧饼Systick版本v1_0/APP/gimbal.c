#include "gimbal.h"


GimbalCtrlObj gimbal;
//��̨���ƶ���
extern VisionRxDataObj visionData;
//�Ӿ����ݽṹ��
float kFeedFoward=0.f;
//ǰ��ϵ��
/**
  * @brief  ��̨���Ƶĳ�ʼ������
  * @param  void
  * @retval void
  * @attention ��ʼ����̨����ĺ�������������ر���
*/
void GimbalInit(void)
{
	gimbal.startUpCtrl = GimbalStartUpControl;
	gimbal.startUpFlag = true;
	gimbal.mode = MECHANICAL_MODE;
	gimbal.jointCtrl = GimbalJointControl;
	//������Ϣ��ʼ��
	gimbal.pitch.ctrl = PitchControl;
	gimbal.pitch.measureGyroY = 0;
	gimbal.pitch.measureMechAngle = 0;
	gimbal.pitch.lastMeasureMechAngle = 0;
	gimbal.pitch.measurePeriod = 0;
	gimbal.pitch.targetGyroY = 0;
	gimbal.pitch.targetMechAngle = GIMBAL_PITCH_ANGLE_MID;
	//pitch���ƻ�����Ϣ
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
	//pitch�����ڻ�����
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
	//pitch�����⻷����
	gimbal.yaw.gyroMode.ctrl = YawControlGyroMode;
	gimbal.yaw.gyroMode.measureGyroZ = 0;
	gimbal.yaw.gyroMode.measureYawAngle = 0;
	gimbal.yaw.gyroMode.targetGyroZ = 0;
	gimbal.yaw.gyroMode.targetYawAngle = 0;
	//yaw������ģʽ���ƻ�������
	gimbal.yaw.mechMode.ctrl = YawControlMechMode;
	gimbal.yaw.mechMode.measureMechAngle = 0;
	gimbal.yaw.mechMode.lastMeasureMechAngle = 0;
	gimbal.yaw.mechMode.measureGyroZ = 0;
	gimbal.yaw.mechMode.measurePeriod = 0;
	gimbal.yaw.mechMode.targetGyroZ = 0;
	gimbal.yaw.mechMode.targetMechAngle = GIMBAL_YAW_START_ANGLE;
	//yaw��еģʽ���ƻ�������
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
	//yaw��������ģʽ�ڻ�����
	
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
	//yaw��������ģʽ�⻷����
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
	//yaw���еģʽ�ڻ�����
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
	//yaw���еģʽ�⻷����
	
	
	/*		�Զ�ģʽ�µĲ���		*/
	gimbal.autoMode.mode = TRACK;
	gimbal.autoMode.yaw.ctrl = YawControlAutoMode;
	gimbal.autoMode.pitch.ctrl = PitchControlAutoMode;
	//ģʽ/������ʼ��
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
	//yaw���Զ�ģʽ�ڻ�����
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
	//yaw���Զ�ģʽ�⻷����	
	
	
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
	//pitch���Զ�ģʽ�ڻ�����
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
	//pitch���Զ�ģʽ�⻷����		
}


/**
  * @brief  ��̨��Ͽ��ƺ���
  * @param  void
  * @retval void
  * @attention ��̨����ģʽ������̨�ĵ�ǰģʽ����������ģʽ�ı���mode����
*/
void GimbalJointControl(void)
{
	if(gimbal.startUpFlag == true)
	{
		//������־λ��������
		gimbal.startUpCtrl();
		//��ʼ�����
		SendToGimbal(gimbal.pitch.pid.itermIn,gimbal.yaw.mechMode.pid.itermIn);
	}
	else
	{
		//������־λ���������
		if(gimbal.mode == MECHANICAL_MODE)
		{
			//��еģʽ
			gimbal.pitch.ctrl();
			//pitch
			gimbal.yaw.mechMode.ctrl();
			//yaw
			SendToGimbal(gimbal.pitch.pid.itermIn,gimbal.yaw.mechMode.pid.itermIn);
			//��PID��������������̨
		}
		else if(gimbal.mode == GYRO_MODE)
		{
			//������ģʽ
			gimbal.pitch.ctrl();
			//pitch
			gimbal.yaw.gyroMode.ctrl();
			//yaw			
			SendToGimbal(gimbal.pitch.pid.itermIn,gimbal.yaw.gyroMode.pid.itermIn);
			//��PID��������������̨			
		}
		else if(gimbal.mode == AUTO_MODE)
		{
			//�Զ�ģʽ
			//��Ϊ�Զ�ģʽ
			SendToGimbal(0,0);
		}
		else
		{
			//�쳣
			//�쳣���紦��
			SendToGimbal(0,0);
		}	
	}
	
	
}

/**
  * @brief  ��̨�����Ŀ��ƺ���
  * @param  void
  * @retval void
  * @attention ��Ҫ���ڳ�ʼ��������ʹ��̨�������С��������ı�־λ����
*/
void GimbalStartUpControl(void)
{
	//��������
	bool yawOK=false,pitchOK=false;
	static uint16_t targetYaw,targetPitch;
	//��Ϊ����ֵ��
	int16_t step=10;
	//����
	
	if(abs(targetPitch - GIMBAL_PITCH_INIT_ANGLE) <= step )
	{
		targetPitch = GIMBAL_PITCH_INIT_ANGLE;
		pitchOK=true;
		//pitch���ʼ�����
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
		//yaw���ʼ�����
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
	//��ɳ�ʼ�����̡�
	
	gimbal.pitch.targetMechAngle = targetPitch;
	gimbal.yaw.mechMode.targetMechAngle = targetYaw;
	
	gimbal.pitch.ctrl();
	gimbal.yaw.mechMode.ctrl();
	
	
}
int16_t currentTarget=0;
extern extKalman_t kalman_targetGyrox;
/**
  * @brief  pitch����ƺ���
  * @param  void
  * @retval void
  * @attention ��̨˫ģʽ��pitch����ƶ�һ���� ��е�Ƕ�-�����ǽ��ٶȵĴ���PID����
*/
void PitchControl(void)
{
//	float kLpf=0.1f;
//	static float lastTargetGyroY=0;
	gimbal.pitch.pid.errorOut = gimbal.pitch.targetMechAngle - gimbal.pitch.measureMechAngle;
	//�����⻷���
	gimbal.pitch.pid.integrateOut += gimbal.pitch.pid.errorOut;
	//����
	gimbal.pitch.pid.integrateOut = constrain(gimbal.pitch.pid.integrateOut,-gimbal.pitch.pid.integrateMaxOut,gimbal.pitch.pid.integrateMaxOut);
	//�����޷�
//	if(((gimbal.pitch.pid.errorOut*gimbal.pitch.pid.integrateOut)<0)&&((abs(gimbal.pitch.pid.integrateOut))>(gimbal.pitch.pid.integrateMaxOut/4.f)))
//	{
//		gimbal.pitch.pid.integrateOut=0;
//	}
	gimbal.pitch.pid.pItermOut = gimbal.pitch.pid.kpOut * gimbal.pitch.pid.errorOut;
	//�⻷P
	gimbal.pitch.pid.pItermOut = constrain(gimbal.pitch.pid.pItermOut,-GIMBAL_PITCH_MAX_SPEED,GIMBAL_PITCH_MAX_SPEED);
	//�⻷P�޷�
	gimbal.pitch.pid.iItermOut = gimbal.pitch.pid.kiOut * gimbal.pitch.pid.integrateOut;
	//�⻷I
	gimbal.pitch.pid.dItermOut = gimbal.pitch.pid.kdOut * (gimbal.pitch.pid.errorOut - gimbal.pitch.pid.lastErrorOut);
	//�⻷D
	gimbal.pitch.pid.dItermOut = constrain(gimbal.pitch.pid.dItermOut,-300,300);
	//�⻷D�޷�
	gimbal.pitch.pid.itermOut = gimbal.pitch.pid.pItermOut + gimbal.pitch.pid.iItermOut + gimbal.pitch.pid.dItermOut;
	//�⻷���
	gimbal.pitch.pid.itermOut = constrain(gimbal.pitch.pid.itermOut,-GIMBAL_PITCH_MAX_SPEED,GIMBAL_PITCH_MAX_SPEED);
	//����޷�
	gimbal.pitch.pid.lastErrorOut = gimbal.pitch.pid.errorOut;
	//��¼��һ�����
	
	gimbal.pitch.targetGyroY = -gimbal.pitch.pid.itermOut;
	
	gimbal.pitch.targetGyroY = KalmanFilter(&kalman_targetGyrox,gimbal.pitch.targetGyroY);
	
	
	/*	-----------------������pitch�⻷�������------------------------	*/
	/*
		���źͽǶ��ھ�ֹ״̬�µĶ�Ӧ��ϵ��ʽ��
			
			���� = -3.1105*��е�Ƕ�+13951  �Դ����ֲ���е����  -- �޸���
			
			���� = -3.0459*��е�Ƕ�+12795  �Դ����ֲ���е����  -- ��ǹ��
			
			���� -- �Դ����ֲ���е���� -- ��ǹ�ܺͼ�����ͷ

			3746 - 3770  -  ���� = -15.192*��е�Ƕ�+58639
			3770 - 3780  -  ���� = -41.667*��е�Ƕ�+158508
			3780 - 4054  -  ���� = -2.2791*��е�Ƕ�+9647.5
			4054 - 4086  -  ���� = -14.443*��е�Ƕ�+58953
			4086 - 4088  -  ���� = -100*��е�Ƕ�+408500
			4088 - 4259  -  ���� = -2.4952*��е�Ƕ�+9851.2
			4259 - 4752  -  ���� = -2.1127*��е�Ƕ�+8216.3
			
			���� = -5.7153*��е�Ƕ� + 24461 	 -- ��ǹ�ܺ�������ͷ�ͼ���ģ��
			
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
	//�����ڻ����
	
	gimbal.pitch.pid.integrateIn += gimbal.pitch.pid.errorIn;
	//�ڻ�����
	gimbal.pitch.pid.integrateIn = constrain(gimbal.pitch.pid.integrateIn,-gimbal.pitch.pid.integrateMaxIn,gimbal.pitch.pid.integrateMaxIn);
	//�����޷�
//	if(((gimbal.pitch.pid.errorIn*gimbal.pitch.pid.integrateIn)<0)&&((abs(gimbal.pitch.pid.integrateIn))>(gimbal.pitch.pid.integrateMaxIn/4.f)))
//	{
//		gimbal.pitch.pid.integrateIn=0;
//	}
	
	gimbal.pitch.pid.pItermIn = gimbal.pitch.pid.kpIn * gimbal.pitch.pid.errorIn;
	//�ڻ�P
	gimbal.pitch.pid.pItermIn = constrain(gimbal.pitch.pid.pItermIn,-5000,5000);
	//�ڻ�P�޷�
	gimbal.pitch.pid.iItermIn = gimbal.pitch.pid.kiIn * gimbal.pitch.pid.integrateIn;
	//�ڻ�I
	gimbal.pitch.pid.dItermIn = gimbal.pitch.pid.kdIn * (gimbal.pitch.pid.errorIn - gimbal.pitch.pid.lastErrorIn);
	//�ڻ�D
	gimbal.pitch.pid.dItermIn = constrain(gimbal.pitch.pid.dItermIn,-1000,1000);
	//�ڻ�D�޷�
	gimbal.pitch.pid.itermIn = gimbal.pitch.pid.pItermIn + gimbal.pitch.pid.iItermIn + gimbal.pitch.pid.dItermIn;
	//�ڻ����
	
	gimbal.pitch.pid.itermIn += thrCompensate;
	
	
	gimbal.pitch.pid.itermIn = constrain(gimbal.pitch.pid.itermIn,-5000,5000);
	
	
	//�ڻ�����޷�
	gimbal.pitch.pid.lastErrorIn = gimbal.pitch.pid.errorIn;
	//��¼�ڻ����
	/*	-----------------������pitch�ڻ��������------------------------	*/	
}

/**
  * @brief  Yaw���еģʽ���ƺ���
  * @param  void
  * @retval void
  * @attention ��е�Ƕ�-�����ǽ��ٶȵĴ���PID����  -- ���Ϊ��ʱ������С��е�Ƕȵķ���ת
*/
void YawControlMechMode(void)
{
//	float kLpf=0.1f;
	static int16_t lastTargetGyroZ=0;
	gimbal.yaw.mechMode.pid.errorOut = gimbal.yaw.mechMode.targetMechAngle - gimbal.yaw.mechMode.measureMechAngle;
	//�����⻷���
	
	if(gimbal.yaw.mechMode.pid.errorOut>5000)
	{
		gimbal.yaw.mechMode.pid.errorOut = -(8191 - gimbal.yaw.mechMode.targetMechAngle + gimbal.yaw.mechMode.measureMechAngle);
	
	}
	else if(gimbal.yaw.mechMode.pid.errorOut<-5000)
	{
		gimbal.yaw.mechMode.pid.errorOut = (8191 - gimbal.yaw.mechMode.measureMechAngle + gimbal.yaw.mechMode.targetMechAngle);
	}
	
	gimbal.yaw.mechMode.pid.integrateOut += gimbal.yaw.mechMode.pid.errorOut;
	//����
	gimbal.yaw.mechMode.pid.integrateOut = constrain(gimbal.yaw.mechMode.pid.integrateOut,-gimbal.yaw.mechMode.pid.integrateMaxOut,gimbal.yaw.mechMode.pid.integrateMaxOut);
	//�����޷�
	
	if(((gimbal.yaw.mechMode.pid.errorOut*gimbal.yaw.mechMode.pid.integrateOut)<0)&&(abs(gimbal.yaw.mechMode.pid.integrateOut)>(gimbal.yaw.mechMode.pid.integrateMaxOut/10.f)))
	{
		gimbal.yaw.mechMode.pid.integrateOut=0;
	}	
	//�������
	
	gimbal.yaw.mechMode.pid.pItermOut = gimbal.yaw.mechMode.pid.kpOut * gimbal.yaw.mechMode.pid.errorOut;
	//�⻷P
	gimbal.yaw.mechMode.pid.pItermOut = constrain(gimbal.yaw.mechMode.pid.pItermOut,-GIMBAL_GYRO_YAW_MAX_SPEED,GIMBAL_GYRO_YAW_MAX_SPEED);
	//�⻷P�޷�
	gimbal.yaw.mechMode.pid.iItermOut = gimbal.yaw.mechMode.pid.kiOut * gimbal.yaw.mechMode.pid.integrateOut;
	//�⻷I
	gimbal.yaw.mechMode.pid.dItermOut = gimbal.yaw.mechMode.pid.kdOut * (gimbal.yaw.mechMode.pid.errorOut - gimbal.yaw.mechMode.pid.lastErrorOut);
	//�⻷D
	gimbal.yaw.mechMode.pid.dItermOut = constrain(gimbal.yaw.mechMode.pid.dItermOut,-2000,2000);
	//�⻷D�޷�
	gimbal.yaw.mechMode.pid.itermOut = gimbal.yaw.mechMode.pid.pItermOut + gimbal.yaw.mechMode.pid.iItermOut + gimbal.yaw.mechMode.pid.dItermOut;
	//�⻷���
	gimbal.yaw.mechMode.pid.itermOut = constrain(gimbal.yaw.mechMode.pid.itermOut,-GIMBAL_GYRO_YAW_MAX_SPEED,GIMBAL_GYRO_YAW_MAX_SPEED);
	//����޷�
	gimbal.yaw.mechMode.pid.lastErrorOut = gimbal.yaw.mechMode.pid.errorOut;
	//��¼��һ�����
	
	gimbal.yaw.mechMode.targetGyroZ = gimbal.yaw.mechMode.pid.itermOut;
//	






//	gimbal.yaw.mechMode.targetGyroZ = kLpf*gimbal.yaw.mechMode.targetGyroZ + (1-kLpf)*lastTargetGyroZ;
//	
	/*	-----------------������yaw.mechMode�⻷�������------------------------	*/
	float feedwardIterm=0;
	//ǰ����
	feedwardIterm = kFeedFoward*(gimbal.yaw.mechMode.targetGyroZ - lastTargetGyroZ);
	//ǰ������
	feedwardIterm = constrain(feedwardIterm,-3000,3000);
	//�޷�
	lastTargetGyroZ = gimbal.yaw.mechMode.targetGyroZ;	
	//��¼��һ�����ֵ
	
	gimbal.yaw.mechMode.pid.errorIn = gimbal.yaw.mechMode.targetGyroZ - gimbal.yaw.mechMode.measureGyroZ;
	//�����ڻ����
	gimbal.yaw.mechMode.pid.integrateIn += gimbal.yaw.mechMode.pid.errorIn;
	//�ڻ�����
	gimbal.yaw.mechMode.pid.integrateIn = constrain(gimbal.yaw.mechMode.pid.integrateIn,-gimbal.yaw.mechMode.pid.integrateMaxIn,gimbal.yaw.mechMode.pid.integrateMaxIn);
	//�����޷�
	if(((gimbal.yaw.mechMode.pid.errorIn*gimbal.yaw.mechMode.pid.integrateIn)<0)&&(abs(gimbal.yaw.mechMode.pid.integrateIn)>(gimbal.yaw.mechMode.pid.integrateMaxIn/5.f)))
	{
		gimbal.yaw.mechMode.pid.integrateIn=0;
	}
	gimbal.yaw.mechMode.pid.pItermIn = gimbal.yaw.mechMode.pid.kpIn * gimbal.yaw.mechMode.pid.errorIn;
	//�ڻ�P
	gimbal.yaw.mechMode.pid.pItermIn = constrain(gimbal.yaw.mechMode.pid.pItermIn,-5000,5000);
	//�ڻ�P�޷�
	gimbal.yaw.mechMode.pid.iItermIn = gimbal.yaw.mechMode.pid.kiIn * gimbal.yaw.mechMode.pid.integrateIn;
	//�ڻ�I
	gimbal.yaw.mechMode.pid.dItermIn = gimbal.yaw.mechMode.pid.kdIn * (gimbal.yaw.mechMode.pid.errorIn - gimbal.yaw.mechMode.pid.lastErrorIn);
	//�ڻ�D
	gimbal.yaw.mechMode.pid.dItermIn = constrain(gimbal.yaw.mechMode.pid.dItermIn,-2000,2000);
	//�ڻ�D�޷�
	gimbal.yaw.mechMode.pid.itermIn = gimbal.yaw.mechMode.pid.pItermIn + gimbal.yaw.mechMode.pid.iItermIn + gimbal.yaw.mechMode.pid.dItermIn + feedwardIterm;
	//�ڻ����
	gimbal.yaw.mechMode.pid.itermIn = constrain(gimbal.yaw.mechMode.pid.itermIn,-5000,5000);
	//�ڻ�����޷�
	gimbal.yaw.mechMode.pid.lastErrorIn = gimbal.yaw.mechMode.pid.errorIn;
	//��¼�ڻ����	
	
		/*	-----------------������yaw.mechMode�ڻ��������------------------------	*/
}

/**
  * @brief  Yaw��������ģʽ���ƺ���
  * @param  void
  * @retval void
  * @attention ��̬YAW�Ƕ�-�����ǽ��ٶȵĴ���PID����
*/
void YawControlGyroMode(void)
{
//	float kLpf=0.1f;
	static int16_t lastTargetGyroZ=0;
	gimbal.yaw.gyroMode.pid.errorOut = gimbal.yaw.gyroMode.targetYawAngle - gimbal.yaw.gyroMode.measureYawAngle;
	//�����⻷���
	if(gimbal.yaw.gyroMode.pid.errorOut>180)
	{
		//����-����>180  ˵��yaw�Ƕȴ�-180��Ծ����180�ķ���
		gimbal.yaw.gyroMode.pid.errorOut = gimbal.yaw.gyroMode.pid.errorOut - 360;
		//����ֵΪ��ȷ�����ֵ��
	}
	else if(gimbal.yaw.gyroMode.pid.errorOut <-180)
	{
		//����-����<0��˵��yaw�Ƕȴ�180��Ծ����-180�ķ���
		gimbal.yaw.gyroMode.pid.errorOut = 360+gimbal.yaw.gyroMode.pid.errorOut;
	}	

	gimbal.yaw.gyroMode.pid.integrateOut += gimbal.yaw.gyroMode.pid.errorOut;
	//����
	gimbal.yaw.gyroMode.pid.integrateOut = constrain(gimbal.yaw.gyroMode.pid.integrateOut,-gimbal.yaw.gyroMode.pid.integrateMaxOut,gimbal.yaw.gyroMode.pid.integrateMaxOut);
	//�����޷�
	if(((gimbal.yaw.gyroMode.pid.errorOut*gimbal.yaw.gyroMode.pid.integrateOut)<0)&&(abs(gimbal.yaw.gyroMode.pid.integrateOut)>(gimbal.yaw.gyroMode.pid.integrateMaxOut/10.f)))
	{
		gimbal.yaw.gyroMode.pid.integrateOut=0;
	}	
	//�������	
	gimbal.yaw.gyroMode.pid.pItermOut = gimbal.yaw.gyroMode.pid.kpOut * gimbal.yaw.gyroMode.pid.errorOut;
	//�⻷P
	gimbal.yaw.gyroMode.pid.pItermOut = constrain(gimbal.yaw.gyroMode.pid.pItermOut,-GIMBAL_GYRO_YAW_MAX_SPEED,GIMBAL_GYRO_YAW_MAX_SPEED);
//	//�⻷P�޷�
	gimbal.yaw.gyroMode.pid.iItermOut = gimbal.yaw.gyroMode.pid.kiOut * gimbal.yaw.gyroMode.pid.integrateOut;
	//�⻷I
	gimbal.yaw.gyroMode.pid.iItermOut = constrain(gimbal.yaw.gyroMode.pid.iItermOut,-gimbal.yaw.gyroMode.pid.integrateMaxOut,gimbal.yaw.gyroMode.pid.integrateMaxOut);
	//�⻷I�޷�
	gimbal.yaw.gyroMode.pid.dItermOut = gimbal.yaw.gyroMode.pid.kdOut * (gimbal.yaw.gyroMode.pid.errorOut - gimbal.yaw.gyroMode.pid.lastErrorOut);
	//�⻷D
	gimbal.yaw.gyroMode.pid.dItermOut = constrain(gimbal.yaw.gyroMode.pid.dItermOut,-2000,2000);
	//�⻷D�޷�
	gimbal.yaw.gyroMode.pid.itermOut = gimbal.yaw.gyroMode.pid.pItermOut + gimbal.yaw.gyroMode.pid.iItermOut + gimbal.yaw.gyroMode.pid.dItermOut;
	//�⻷���
	gimbal.yaw.gyroMode.pid.itermOut = constrain(gimbal.yaw.gyroMode.pid.itermOut,-GIMBAL_GYRO_YAW_MAX_SPEED,GIMBAL_GYRO_YAW_MAX_SPEED);
	//����޷�
	gimbal.yaw.gyroMode.pid.lastErrorOut = gimbal.yaw.gyroMode.pid.errorOut;
	//��¼��һ�����
	

	gimbal.yaw.gyroMode.targetGyroZ = -gimbal.yaw.gyroMode.pid.itermOut;
//	



//	gimbal.yaw.gyroMode.targetGyroZ = kLpf*gimbal.yaw.gyroMode.targetGyroZ + (1-kLpf)*lastTargetGyroZ;
//	

	/*	-----------------������yaw.gyroMode�⻷�������------------------------	*/
	float feedwardIterm=0;
	//ǰ����
	feedwardIterm = kFeedFoward*(gimbal.yaw.gyroMode.targetGyroZ - lastTargetGyroZ);
	//ǰ������
	feedwardIterm = constrain(feedwardIterm,-3000,3000);
	//�޷�
	lastTargetGyroZ = gimbal.yaw.gyroMode.targetGyroZ;
	//��¼��һ������ֵ
	
	gimbal.yaw.gyroMode.pid.errorIn = gimbal.yaw.gyroMode.targetGyroZ - gimbal.yaw.gyroMode.measureGyroZ;
	//�����ڻ����
	gimbal.yaw.gyroMode.pid.integrateIn += gimbal.yaw.gyroMode.pid.errorIn;
	//�ڻ�����
	gimbal.yaw.gyroMode.pid.integrateIn = constrain(gimbal.yaw.gyroMode.pid.integrateIn,-gimbal.yaw.gyroMode.pid.integrateMaxIn,gimbal.yaw.gyroMode.pid.integrateMaxIn);
	//�����޷�
	if(((gimbal.yaw.gyroMode.pid.errorIn*gimbal.yaw.gyroMode.pid.integrateIn)<0)&&(abs(gimbal.yaw.gyroMode.pid.integrateIn)>(gimbal.yaw.gyroMode.pid.integrateMaxIn/5.f)))
	{
		gimbal.yaw.gyroMode.pid.integrateIn=0;
	}
	gimbal.yaw.gyroMode.pid.pItermIn = gimbal.yaw.gyroMode.pid.kpIn * gimbal.yaw.gyroMode.pid.errorIn;
	//�ڻ�P
	gimbal.yaw.gyroMode.pid.pItermIn = constrain(gimbal.yaw.gyroMode.pid.pItermIn,-5000,5000);
	//�ڻ�P�޷�
	gimbal.yaw.gyroMode.pid.iItermIn = gimbal.yaw.gyroMode.pid.kiIn * gimbal.yaw.gyroMode.pid.integrateIn;
	//�ڻ�I
	gimbal.yaw.gyroMode.pid.dItermIn = gimbal.yaw.gyroMode.pid.kdIn * (gimbal.yaw.gyroMode.pid.errorIn - gimbal.yaw.gyroMode.pid.lastErrorIn);
	//�ڻ�D
	gimbal.yaw.gyroMode.pid.dItermIn = constrain(gimbal.yaw.gyroMode.pid.dItermIn,-2000,2000);
	//�ڻ�D�޷�
	gimbal.yaw.gyroMode.pid.itermIn = gimbal.yaw.gyroMode.pid.pItermIn + gimbal.yaw.gyroMode.pid.iItermIn + gimbal.yaw.gyroMode.pid.dItermIn + feedwardIterm;
	//�ڻ����
	gimbal.yaw.gyroMode.pid.itermIn = constrain(gimbal.yaw.gyroMode.pid.itermIn,-5000,5000);
	//�ڻ�����޷�
	gimbal.yaw.gyroMode.pid.lastErrorIn = gimbal.yaw.gyroMode.pid.errorIn;
	//��¼�ڻ����	
	
		/*	-----------------������yaw.gyroMode�ڻ��������------------------------	*/
}



/**
  * @brief  Yaw���Զ�ģʽ���ƺ���
  * @param  void
  * @retval void
  * @attention �Ӿ��Ƕ�-�����ǽ��ٶȵĴ���PID����
*/
extern float visionYawMathExpect;
//��ѧ����
extern float visionYawStdDevia;
//��׼��
int16_t gyrozTarget=0;
		
//float km=120,ks=20;
float km=0,ks=0;
//ϵ��
extern QueueObj visionQueue;
float compensateGyroYaw=0;
extern float predictAngle;
void YawControlAutoMode(void)
{
	if(gimbal.autoMode.mode == TRACK)
	{
		//����ģʽ
		gimbal.autoMode.yaw.pid.errorOut = visionData.yawAngle-predictAngle;
		//����Ҫ�ж��������
		
		gimbal.autoMode.yaw.pid.errorOut = myDeathZoom(0,0.01f,gimbal.autoMode.yaw.pid.errorOut);
		
		gimbal.autoMode.yaw.pid.integrateOut+=gimbal.autoMode.yaw.pid.errorOut;
		//����
		
		gimbal.autoMode.yaw.pid.integrateOut = constrain(gimbal.autoMode.yaw.pid.integrateOut,-gimbal.autoMode.yaw.pid.integrateMaxOut,gimbal.autoMode.yaw.pid.integrateMaxOut);
		//�����޷�
		
		
		if(((gimbal.autoMode.yaw.pid.errorOut*gimbal.autoMode.yaw.pid.integrateOut)<0)&&(abs(gimbal.autoMode.yaw.pid.integrateOut)>(gimbal.autoMode.yaw.pid.integrateMaxOut/10.f)))
		{
			gimbal.autoMode.yaw.pid.integrateOut=0;
		}	
		
		gimbal.autoMode.yaw.pid.pItermOut = gimbal.autoMode.yaw.pid.kpOut * gimbal.autoMode.yaw.pid.errorOut;
		//p
		
		gimbal.autoMode.yaw.pid.pItermOut = constrain(gimbal.autoMode.yaw.pid.pItermOut,-GIMBAL_GYRO_YAW_MAX_SPEED,GIMBAL_GYRO_YAW_MAX_SPEED);
		//p�޷�
		
		
		gimbal.autoMode.yaw.pid.iItermOut = gimbal.autoMode.yaw.pid.kiOut * gimbal.autoMode.yaw.pid.integrateOut;
		//i
		
		gimbal.autoMode.yaw.pid.dItermOut = gimbal.autoMode.yaw.pid.kdOut * (gimbal.autoMode.yaw.pid.errorOut - gimbal.autoMode.yaw.pid.lastErrorOut);
		//d
		
		gimbal.autoMode.yaw.pid.dItermOut = constrain(gimbal.autoMode.yaw.pid.dItermOut,-1000,1000);
		//d�޷�
		
		gimbal.autoMode.yaw.pid.itermOut = gimbal.autoMode.yaw.pid.pItermOut + gimbal.autoMode.yaw.pid.iItermOut + gimbal.autoMode.yaw.pid.dItermOut;
		//out
		
		gimbal.autoMode.yaw.pid.itermOut = constrain(gimbal.autoMode.yaw.pid.itermOut,-GIMBAL_GYRO_YAW_MAX_SPEED,GIMBAL_GYRO_YAW_MAX_SPEED);
		//�޷�
		
		/*	-----------------������yaw.Aim�⻷�������------------------------	*/
		
		

		

		compensateGyroYaw = km*(visionYawMathExpect-predictAngle) + ks*(visionYawStdDevia+predictAngle);
		

		
		
		
		
		
		
		gyrozTarget = gimbal.autoMode.yaw.pid.itermOut+compensateGyroYaw;
		
		gimbal.autoMode.yaw.pid.errorIn = gimbal.autoMode.yaw.pid.itermOut + compensateGyroYaw - gimbal.yaw.gyroMode.measureGyroZ;
		//���
		
		gimbal.autoMode.yaw.pid.integrateIn += gimbal.autoMode.yaw.pid.errorIn;
		//�ڻ�����
		
		gimbal.autoMode.yaw.pid.integrateIn = constrain(gimbal.autoMode.yaw.pid.integrateIn,-gimbal.autoMode.yaw.pid.integrateMaxIn,gimbal.autoMode.yaw.pid.integrateMaxIn);
		//�����޷�
		
		if(((gimbal.autoMode.yaw.pid.errorIn*gimbal.autoMode.yaw.pid.integrateIn)<0)&&(abs(gimbal.autoMode.yaw.pid.integrateIn)>(gimbal.autoMode.yaw.pid.integrateMaxIn/5.f)))
		{
			gimbal.autoMode.yaw.pid.integrateIn=0;
		}		
		
		gimbal.autoMode.yaw.pid.pItermIn = gimbal.autoMode.yaw.pid.kpIn * gimbal.autoMode.yaw.pid.errorIn;
		//�ڻ�P
		
		gimbal.autoMode.yaw.pid.pItermIn = constrain(gimbal.autoMode.yaw.pid.pItermIn,-5000,5000);
		//�ڻ�P�޷�
		
		gimbal.autoMode.yaw.pid.iItermIn = gimbal.autoMode.yaw.pid.kiIn * gimbal.autoMode.yaw.pid.integrateIn;
		//�ڻ�I
		
		gimbal.autoMode.yaw.pid.dItermIn = gimbal.autoMode.yaw.pid.kdIn * (gimbal.autoMode.yaw.pid.errorIn - gimbal.autoMode.yaw.pid.lastErrorIn);
		//�ڻ�D
		
		gimbal.autoMode.yaw.pid.dItermIn = constrain(gimbal.autoMode.yaw.pid.dItermIn,-2000,2000);
		//�ڻ�D�޷�
		
		gimbal.autoMode.yaw.pid.itermIn = gimbal.autoMode.yaw.pid.pItermIn + gimbal.autoMode.yaw.pid.iItermIn + gimbal.autoMode.yaw.pid.dItermIn;
		//�ڻ����
		
		gimbal.autoMode.yaw.pid.itermIn = constrain(gimbal.autoMode.yaw.pid.itermIn,-5000,5000);
		//�ڻ�����޷�
		
		gimbal.autoMode.yaw.pid.lastErrorIn = gimbal.autoMode.yaw.pid.errorIn;
		//��¼�ڻ����	
		
		
			/*	-----------------������yaw.Aim�ڻ��������------------------------	*/		
	}
	else if(gimbal.autoMode.mode == AIM)
	{
		//��׼ģʽ

	}
}





/**
  * @brief  pitch���Զ�ģʽ���ƺ���
  * @param  void
  * @retval void
  * @attention �Ӿ��Ƕ�-�����ǽ��ٶȵĴ���PID����
*/
void PitchControlAutoMode(void)
{
	if(gimbal.autoMode.mode == TRACK)
	{
		//����ģʽ
		gimbal.autoMode.pitch.pid.errorOut = visionData.pitchAngle;
		//����Ҫ�ж��������
		
		gimbal.autoMode.pitch.pid.errorOut = myDeathZoom(0,0.01f,gimbal.autoMode.pitch.pid.errorOut);
		//����
		
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
		//����
		
		gimbal.autoMode.pitch.pid.integrateOut = constrain(gimbal.autoMode.pitch.pid.integrateOut,-gimbal.autoMode.pitch.pid.integrateMaxOut,gimbal.autoMode.pitch.pid.integrateMaxOut);
		//�����޷�
		
		gimbal.autoMode.pitch.pid.pItermOut = gimbal.autoMode.pitch.pid.kpOut * gimbal.autoMode.pitch.pid.errorOut;
		//p
		
		gimbal.autoMode.pitch.pid.pItermOut = constrain(gimbal.autoMode.pitch.pid.pItermOut,-GIMBAL_GYRO_YAW_MAX_SPEED,GIMBAL_GYRO_YAW_MAX_SPEED);
		//p�޷�
		
		
		gimbal.autoMode.pitch.pid.iItermOut = gimbal.autoMode.pitch.pid.kiOut * gimbal.autoMode.pitch.pid.integrateOut;
		//i
		
		gimbal.autoMode.pitch.pid.dItermOut = gimbal.autoMode.pitch.pid.kdOut * (gimbal.autoMode.pitch.pid.errorOut - gimbal.autoMode.pitch.pid.lastErrorOut);
		//d
		
		gimbal.autoMode.pitch.pid.dItermOut = constrain(gimbal.autoMode.pitch.pid.dItermOut,-1000,1000);
		//d�޷�
		
		gimbal.autoMode.pitch.pid.itermOut = gimbal.autoMode.pitch.pid.pItermOut + gimbal.autoMode.pitch.pid.iItermOut + gimbal.autoMode.pitch.pid.dItermOut;
		//out
		

		
		gimbal.autoMode.pitch.pid.itermOut = kLPF*gimbal.autoMode.pitch.pid.itermOut+(1-kLPF)*lastOut;
		
		
		gimbal.autoMode.pitch.pid.itermOut = constrain(gimbal.autoMode.pitch.pid.itermOut,-GIMBAL_PITCH_MAX_SPEED,GIMBAL_PITCH_MAX_SPEED);
		//�޷�
		
		lastOut = gimbal.autoMode.pitch.pid.itermOut;
		
		/*	-----------------������pitch.Aim�⻷�������------------------------	*/
		

		
		int16_t thrCompensate = 0;
		
	//	thrCompensate = -3.1105f * gimbal.pitch.measureMechAngle + 13951;
	//	thrCompensate = -3.0459f * gimbal.pitch.measureMechAngle + 12795;

		thrCompensate = GetThrCompensate(gimbal.pitch.measureMechAngle);
	
		
		gimbal.autoMode.pitch.pid.errorIn = gimbal.autoMode.pitch.pid.itermOut - gimbal.pitch.measureGyroY;
		//���
		
		gimbal.autoMode.pitch.pid.integrateIn += gimbal.autoMode.pitch.pid.errorIn;
		//�ڻ�����
		
		gimbal.autoMode.pitch.pid.integrateIn = constrain(gimbal.autoMode.pitch.pid.integrateIn,-gimbal.autoMode.pitch.pid.integrateMaxIn,gimbal.autoMode.pitch.pid.integrateMaxIn);
		//�����޷�
		
		gimbal.autoMode.pitch.pid.pItermIn = gimbal.autoMode.pitch.pid.kpIn * gimbal.autoMode.pitch.pid.errorIn;
		//�ڻ�P
		
		gimbal.autoMode.pitch.pid.pItermIn = constrain(gimbal.autoMode.pitch.pid.pItermIn,-5000,5000);
		//�ڻ�P�޷�
		
		gimbal.autoMode.pitch.pid.iItermIn = gimbal.autoMode.pitch.pid.kiIn * gimbal.autoMode.pitch.pid.integrateIn;
		//�ڻ�I
		
		gimbal.autoMode.pitch.pid.dItermIn = gimbal.autoMode.pitch.pid.kdIn * (gimbal.autoMode.pitch.pid.errorIn - gimbal.autoMode.pitch.pid.lastErrorIn);
		//�ڻ�D
		
		gimbal.autoMode.pitch.pid.dItermIn = constrain(gimbal.autoMode.pitch.pid.dItermIn,-2000,2000);
		//�ڻ�D�޷�
		
		gimbal.autoMode.pitch.pid.itermIn = gimbal.autoMode.pitch.pid.pItermIn + gimbal.autoMode.pitch.pid.iItermIn + gimbal.autoMode.pitch.pid.dItermIn;
		//�ڻ����
		
		gimbal.autoMode.pitch.pid.itermIn+=thrCompensate;
		
		gimbal.autoMode.pitch.pid.itermIn = constrain(gimbal.autoMode.pitch.pid.itermIn,-5000,5000);
		//�ڻ�����޷�
		
		gimbal.autoMode.pitch.pid.lastErrorIn = gimbal.autoMode.pitch.pid.errorIn;
		//��¼�ڻ����			
	
	}
	else if(gimbal.autoMode.mode == AIM)
	{
	
	
	
	}

	
	
}


/**
  * @brief  ��̨������
  * @param  pitchOutput pitch�������
						yawOutput  yaw�������
  * @retval void
	* @attention None
*/
void SendToGimbal(int16_t pitchOutput,int16_t yawOutput)
{
	CAN2_Send(0x1FF,yawOutput,pitchOutput,0,0);
}



/**
  * @brief  ��̨����Ƕ����ݸ���
  * @param yawTmp  yaw��Ƕ����ݸ���
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
			3746 - 3770  -  ���� = -15.192*��е�Ƕ�+58639
			3770 - 3780  -  ���� = -41.667*��е�Ƕ�+158508
			3780 - 4054  -  ���� = -2.2791*��е�Ƕ�+9647.5
			4054 - 4078  -  ���� = -11.14*��е�Ƕ�+45545
			4078 - 4080  -  ���� = 0.975*��е�Ƕ�+100
			4080 - 4086  -  ���� = 0.9747*��е�Ƕ�+97.468
			4086 - 4088  -  ���� = -100*��е�Ƕ�+408500
			4088 - 4259  -  ���� = -2.4952*��е�Ƕ�+9851.2
			4259 - 4752  -  ���� = -2.1127*��е�Ƕ�+8216.3
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
		//��ֵ����ֵС�ڲ���
		currentTarget = goalTarget;
	}
	else
	{
		//��ֵ���ڲ���
		if(goalTarget>currentTarget)
			currentTarget+=step;
		else
			currentTarget-=step;
	}
	return currentTarget;
}


