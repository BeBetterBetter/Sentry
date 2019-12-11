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


short gyrox,gyroy,gyroz;	//陀螺仪原始数据
float pitch,roll,yaw,yaw_10;		//欧拉角
	
//限幅
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

//计数器初始化
static void cycleCounterInit(void)
{
	RCC_ClocksTypeDef clocks;
	RCC_GetClocksFreq(&clocks);
	usTicks = clocks.SYSCLK_Frequency / 1000000; 
}

//以微秒为单位返回系统时间
uint64_t micros(void)
{
	register uint32_t ms, cycle_cnt;
	do {
			ms = sysTickUptime;
			cycle_cnt = SysTick->VAL;
	} while (ms != sysTickUptime);
	return (ms * 1000) + (usTicks * 1000 - cycle_cnt) / usTicks;
}

//微秒级延时
void delay_us(uint32_t us)
{
	uint32_t now = micros();
	while (micros() - now < us);
}

//毫秒级延时
void delay_ms(uint32_t ms)
{
	while (ms--)
			delay_us(1000);
}

//以毫秒为单位返回系统时间
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
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //中断向量表分组
	CRC_init();	
	cycleCounterInit();
	SysTick_Config(SystemCoreClock / 1000);	//滴答定时器配置，1ms    计数器为168000时产生中断，时钟每秒能计算1000个中断，即1000hz的频率
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
	//遥控器初始化
	while(mpu_dmp_init())//注意自检函数
	{
		currentTime = micros();//获取当前系统时间	
		if((int32_t)(currentTime - loopTime_mpu6050) >= 100000)  
		{	
			loopTime_mpu6050 = currentTime + 100000;			//100ms
			pass_num++;
			if(pass_num>=3)//若超时 则屏蔽自检函数
			{
				pass_flag=0;
				pass_num=10;
			}
		}
	}
}
extern bool visionFlag;
int16_t tmp;
//主循环
void Loop(void)
{	
	static uint32_t currentTime = 0;
	static uint32_t loopTime_1ms = 0;
	static uint32_t loopTime_2ms = 0;
	static uint32_t loopTime_50ms = 0;
	
	short gyrozTemp=0,gyroxTemp=0;
	
	static uint16_t startCnt=0;
	static bool startFlag=false;

	
	currentTime = micros();	//获取当前系统时间
	
	if((int32_t)(currentTime - loopTime_1ms) >= 0)  
	{	
		loopTime_1ms = currentTime + 1000;	//1ms		
	}
	
	if((int32_t)(currentTime - loopTime_2ms) >= 0)  
	{	
		loopTime_2ms = currentTime + 2000;	//2ms		
		//时间戳计时
		
		MPU_Get_Gyroscope(&gyroxTemp,&gyroy,&gyrozTemp);	//读取角速度
		mpu_dmp_get_data(&roll,&pitch,&yaw);		//读取欧拉角		
		//陀螺仪数据读取
		
		yaw_int = yaw;
		//将YAW角度以整形形式进行计算			
		gyrozTemp = myDeathZoom_int(0,25,gyrozTemp);
		//死区滤去抖动噪声
		gyrozTemp = KalmanFilter(&kalman_gyroZ,gyrozTemp);
		gyroz = gyrozTemp;
		//z轴的角速度
		
		gyroxTemp = myDeathZoom_int(0,25,gyroxTemp);	
		//死区滤去抖动噪声
		gyroxTemp = KalmanFilter(&kalman_gyroX,gyroxTemp);
		gyrox = gyroxTemp;
		//x轴的角速度
		
		
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
					//自瞄模式					
					//识别到
					
					if(visionFlag == true)
					{
						//识别到
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
						//未识别到
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
	
	if((int32_t)(currentTime - loopTime_50ms) >= 0)//温漂
	{
		loopTime_50ms = currentTime + 50000;//50ms
	}
}








