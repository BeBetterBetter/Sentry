#include "can2.h"


extern ChassisCtrlObj chassis;
extern GimbalCtrlObj gimbal;
extern ShootCtrlObj shoot;
/*----CAN2_TX-----PB13----*/
/*----CAN2_RX-----PB12----*/

/**
* @brief CAN2初始化函数
* 初始化函数，需要配合can1初始化才能正常使用
* @param void
* @return void
*/
void CAN2_Init(void)
{
    CAN_InitTypeDef        can;
    CAN_FilterInitTypeDef  can_filter;
    GPIO_InitTypeDef       gpio;
    NVIC_InitTypeDef       nvic;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_CAN2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_CAN2); 

    gpio.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 ;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    GPIO_Init(GPIOB, &gpio);

    nvic.NVIC_IRQChannel = CAN2_RX0_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 2;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    CAN_DeInit(CAN2);
    CAN_StructInit(&can);

    can.CAN_TTCM = DISABLE;
    can.CAN_ABOM = DISABLE;    
    can.CAN_AWUM = DISABLE;    
    can.CAN_NART = DISABLE;    
    can.CAN_RFLM = DISABLE;    
    can.CAN_TXFP = ENABLE;     
    can.CAN_Mode = CAN_Mode_Normal; 
    can.CAN_SJW  = CAN_SJW_1tq;
    can.CAN_BS1 = CAN_BS1_9tq;
    can.CAN_BS2 = CAN_BS2_4tq;
    can.CAN_Prescaler = 3;   //CAN BaudRate 42/(1+9+4)/3=1Mbps
    CAN_Init(CAN2, &can);
    
    can_filter.CAN_FilterNumber=14;
    can_filter.CAN_FilterMode=CAN_FilterMode_IdMask;
    can_filter.CAN_FilterScale=CAN_FilterScale_32bit;
    can_filter.CAN_FilterIdHigh=0x0000;
    can_filter.CAN_FilterIdLow=0x0000;
    can_filter.CAN_FilterMaskIdHigh=0x0000;
    can_filter.CAN_FilterMaskIdLow=0x0000;
    can_filter.CAN_FilterFIFOAssignment=0;//the message which pass the filter save in fifo0 CAN_Filter_FIFO0
    can_filter.CAN_FilterActivation=ENABLE;
    CAN_FilterInit(&can_filter);
    
    CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);    ////FIFO0消息挂号中断允许
} 

/**
* @brief CAN2发送函数
* @param void
* @return void
  @send the data to four device
*/
void CAN2_Send(uint32_t Equipment_ID,int16_t Data0,int16_t Data1,int16_t Data2,int16_t Data3)	
{		
	CanTxMsg TxMessage;
	
  TxMessage.StdId = Equipment_ID;					 //使用的标准ID
  TxMessage.IDE = CAN_ID_STD;				 //标准模式
  TxMessage.RTR = CAN_RTR_DATA;			 //发送的是数据
  TxMessage.DLC = 0x08;							 //数据长度为8字节

	TxMessage.Data[0] = Data0>>8; 
	TxMessage.Data[1] = Data0;
	TxMessage.Data[2] = Data1>>8; 
	TxMessage.Data[3] = Data1;
	TxMessage.Data[4] = Data2>>8; 
	TxMessage.Data[5] =	Data2;
	TxMessage.Data[6] = Data3>>8; 
	TxMessage.Data[7] =	Data3;

	CAN_Transmit(CAN2, &TxMessage);	//发送数据
}

/**
* @brief 云台遥控协议控制函数
* 云台遥控协议
* @param void
* @return void
	@pitch - gyrox向下是正，向上是负   机械角度 向下是负 可能超过最大边界值。
	@yaw - gyroz逆时针是负，机械角度逆时针是负，从上向下看
*/
int16_t angle=0;
void CAN2_RX0_IRQHandler(void)
{
	CanRxMsg RxMessage;

	//时间戳，用于计算旋转速度，单位为微秒。
	if (CAN_GetITStatus(CAN2,CAN_IT_FMP0)!= RESET) 
	{				 
			CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
			CAN_Receive(CAN2, CAN_FIFO0, &RxMessage);

		if(RxMessage.StdId == 0x206)//云台电机	pitch	  0x1FF
		{ 
			gimbal.pitch.measureMechAngle = (int16_t)RxMessage.Data[0]<<8 | RxMessage.Data[1];
			angle = gimbal.pitch.measureMechAngle;
		}
		else if(RxMessage.StdId == 0x205)//云台电机 yaw  0x1FF
		{
			gimbal.yaw.mechMode.measureMechAngle = (int16_t)RxMessage.Data[0]<<8 | RxMessage.Data[1];
//			angle = gimbal.yaw.mechMode.measureMechAngle;	
		}
		else if(RxMessage.StdId == 0x201)//拨盘电机  0x200
		{
			shoot.turnplate.measureAngle = (int16_t)RxMessage.Data[0]<<8 | RxMessage.Data[1];
			shoot.turnplate.measureSpeed = (int16_t)RxMessage.Data[2]<<8 | RxMessage.Data[3];
//			shoot.turnplate.measureSpeed = myDeathZoom_int(0,10,shoot.turnplate.measureSpeed);
			
			
		}		
		else if(RxMessage.StdId == 0x203)//底盘电机		0x200
		{
			chassis.motor.measureSpeed = (int16_t)RxMessage.Data[2]<<8 | RxMessage.Data[3];
			chassis.motor.measureSpeed = myDeathZoom_int(0,10,chassis.motor.measureSpeed);
			
		}	
	}
}




