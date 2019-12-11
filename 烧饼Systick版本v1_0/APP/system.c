#include "system.h"
extern GimbalCtrlObj gimbal;
extern RemoteCtrlObj rc;
extern ChassisCtrlObj chassis;
extern ShootCtrlObj shoot;

static volatile uint32_t usTicks = 0;

uint32_t currentTime = 0;
uint32_t loopTime_1ms=0;
uint32_t previousTime = 0;
uint16_t cycleTime = 0; 


short gyrox,gyroy,gyroz;	//������ԭʼ����
float pitch,roll,yaw,yaw_10;		//ŷ����
	
//�޷�
float constrain(float amt, float low, float high)
{
	if (amt < low)
		return low;
	else if (amt > high)
		return high;
	else
		return amt;
}
int32_t constrain_int32(int32_t amt, int32_t low, int32_t high)
{
	if (amt < low)
		return low;
	else if (amt > high)
		return high;
	else
		return amt;
}

int16_t constrain_int16(int16_t amt, int16_t low, int16_t high)
{
	if (amt < low)
		return low;
	else if (amt > high)
		return high;
	else
		return amt;
}

int constrain_int(int amt,int low,int high)
{
	if (amt < low)
		return low;
	else if (amt > high)
		return high;
	else
		return amt;
}

//��������ʼ��
static void cycleCounterInit(void)
{
	RCC_ClocksTypeDef clocks;
	RCC_GetClocksFreq(&clocks);
	usTicks = clocks.SYSCLK_Frequency / 1000000; 
}

//��΢��Ϊ��λ����ϵͳʱ��
uint64_t micros(void)
{
	register uint32_t ms, cycle_cnt;
	do {
			ms = sysTickUptime;
			cycle_cnt = SysTick->VAL;
	} while (ms != sysTickUptime);
	return (ms * 1000) + (usTicks * 1000 - cycle_cnt) / usTicks;
}

//΢�뼶��ʱ
void delay_us(uint32_t us)
{
	uint32_t now = micros();
	while (micros() - now < us);
}

//���뼶��ʱ
void delay_ms(uint32_t ms)
{
	while (ms--)
			delay_us(1000);
}

//�Ժ���Ϊ��λ����ϵͳʱ��
uint32_t millis(void)
{
	return sysTickUptime;
}

extKalman_t kalman_targetGyrox;
extKalman_t kalman_visionPitch;
extKalman_t kalman_visionYaw;
extKalman_t kalman_gyroX;
extKalman_t kalman_gyroZ;
int16_t yaw_int=0;
int pass_num;
bool pass_flag=1;
void System_Init(void)
{	
	static uint32_t loopTime_mpu6050 = 0;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //�ж����������
	CRC_init();	
	cycleCounterInit();
	SysTick_Config(SystemCoreClock / 1000);	//�δ�ʱ�����ã�1ms    ������Ϊ168000ʱ�����жϣ�ʱ��ÿ���ܼ���1000���жϣ���1000hz��Ƶ��
	RemotrCtrlInit();
	Led_Init();
	MPU_Init();
	TIM3_Init();
	UART4_Init();
	CAN1_Init();
	CAN2_Init();
	RemotrCtrlInit();
	GimbalInit();
	ChassisInit();
	ShootInit();
	KalmanCreate(&kalman_targetGyrox,1,400);
	KalmanCreate(&kalman_gyroX,1,600);
	KalmanCreate(&kalman_gyroZ,1,200);
	KalmanCreate(&kalman_visionYaw,1,600);
	KalmanCreate(&kalman_visionPitch,1,20);
	//ң������ʼ��
	while(mpu_dmp_init())//ע���Լ캯��
	{
		currentTime = micros();//��ȡ��ǰϵͳʱ��	
		if((int32_t)(currentTime - loopTime_mpu6050) >= 100000)  
		{	
			loopTime_mpu6050 = currentTime + 100000;			//100ms
			pass_num++;
			if(pass_num>=3)//����ʱ �������Լ캯��
			{
				pass_flag=0;
				pass_num=10;
			}
		}
	}
}
extern bool visionFlag;
int16_t tmp;
//��ѭ��
void Loop(void)
{	
	static uint32_t currentTime = 0;
	static uint32_t loopTime_1ms = 0;
	static uint32_t loopTime_2ms = 0;
	static uint32_t loopTime_50ms = 0;
	
	short gyrozTemp=0,gyroxTemp=0;
	
	static uint16_t startCnt=0;
	static bool startFlag=false;

	
	currentTime = micros();	//��ȡ��ǰϵͳʱ��
	
	if((int32_t)(currentTime - loopTime_1ms) >= 0)  
	{	
		loopTime_1ms = currentTime + 1000;	//1ms		
	}
	
	if((int32_t)(currentTime - loopTime_2ms) >= 0)  
	{	
		loopTime_2ms = currentTime + 2000;	//2ms		
		//ʱ�����ʱ
		
		MPU_Get_Gyroscope(&gyroxTemp,&gyroy,&gyrozTemp);	//��ȡ���ٶ�
		mpu_dmp_get_data(&roll,&pitch,&yaw);		//��ȡŷ����		
		//���������ݶ�ȡ
		
		yaw_int = yaw;
		//��YAW�Ƕ���������ʽ���м���			
		gyrozTemp = myDeathZoom_int(0,25,gyrozTemp);
		//������ȥ��������
		gyrozTemp = KalmanFilter(&kalman_gyroZ,gyrozTemp);
		gyroz = gyrozTemp;
		//z��Ľ��ٶ�
		
		gyroxTemp = myDeathZoom_int(0,25,gyroxTemp);	
		//������ȥ��������
		gyroxTemp = KalmanFilter(&kalman_gyroX,gyroxTemp);
		gyrox = gyroxTemp;
		//x��Ľ��ٶ�
		
		
		if(startCnt>=500)
		{
			startCnt=0;
			startFlag=true;
			gimbal.yaw.gyroMode.targetYawAngle = yaw;
			gimbal.yaw.mechMode.targetMechAngle = gimbal.yaw.mechMode.measureMechAngle;
		}
			

		if(startFlag==true)
		{
			rc.rcProtocol();
			VisionTask();
			GimbalUpdatePosture(yaw_int,gyroz,gyrox);
			if(rc.status == true)
			{					
				shoot.ctrl();
//				chassis.motor.ctrl();
				CAN2_Send(0x200,shoot.turnplate.pid.iterm,0,-chassis.motor.pid.iterm,0);
				if(gimbal.mode == MECHANICAL_MODE)
				{
					gimbal.pitch.ctrl();
					gimbal.yaw.mechMode.ctrl();
					SendToGimbal(gimbal.pitch.pid.itermIn,-gimbal.yaw.mechMode.pid.itermIn);	
				}
				else if(gimbal.mode == GYRO_MODE)
				{
					gimbal.pitch.ctrl();
					gimbal.yaw.gyroMode.ctrl();
					SendToGimbal(gimbal.pitch.pid.itermIn,-gimbal.yaw.gyroMode.pid.itermIn);
				}
				else
				{
					//����ģʽ					
					//ʶ��
					
					if(visionFlag == true)
					{
						//ʶ��
						gimbal.autoMode.yaw.ctrl();
						gimbal.autoMode.pitch.ctrl();
//						gimbal.pitch.ctrl();
						SendToGimbal(gimbal.autoMode.pitch.pid.itermIn,-gimbal.autoMode.yaw.pid.itermIn);		
//						SendToGimbal(gimbal.pitch.pid.itermIn,-gimbal.autoMode.yaw.pid.itermIn);		
						gimbal.pitch.targetMechAngle = gimbal.pitch.measureMechAngle;
						gimbal.yaw.gyroMode.targetYawAngle = gimbal.yaw.gyroMode.measureYawAngle;
						gimbal.yaw.mechMode.targetMechAngle = gimbal.yaw.mechMode.measureMechAngle;
					}
					else if(visionFlag == false)
					{
						//δʶ��
						gimbal.pitch.ctrl();
						gimbal.yaw.mechMode.ctrl();
						SendToGimbal(gimbal.pitch.pid.itermIn,-gimbal.yaw.mechMode.pid.itermIn);					
					}
				}				
			}
			else
			{
				gimbal.pitch.targetMechAngle = gimbal.pitch.measureMechAngle;
				gimbal.yaw.gyroMode.targetYawAngle = gimbal.yaw.gyroMode.measureYawAngle;
				gimbal.yaw.mechMode.targetMechAngle = gimbal.yaw.mechMode.measureMechAngle;
				SendToChassis(0);
				SendToGimbal(0,0);	
				SendToTurnplate(0);
			}
		}
		else
			startCnt++;
	}
	
	if((int32_t)(currentTime - loopTime_50ms) >= 0)//��Ư
	{
		loopTime_50ms = currentTime + 50000;//50ms
	}
}








