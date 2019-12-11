#include "pwm.h"







//斜坡函数，给个缓冲区间。
void FrictionOutput(uint16_t target,uint16_t *current)
{
	uint8_t step=1;
	//步长
	if(*current>=target+step)
	{
		//如果期望减小则慢慢减小
		*current-=step;
		//递减
	}
	else if(*current<=target-step)
	{
		//如果期望增大则慢慢增大
		*current+=step;
		//递增
	}
	else 
	{
		*current = target;
	}
	Friction_PWM(*current,*current);

}

void MagazineCtrl(uint16_t target,uint16_t *current)
{
	uint8_t step=1;
	//步长
	if(*current>=target+step)
	{
		//如果期望减小则慢慢减小
		*current-=step;
		//递减
	}
	else if(*current<=target-step)
	{
		//如果期望增大则慢慢增大
		*current+=step;
		//递增
	}
	else 
	{
		*current = target;
	}
	TIM1->CCR2 = *current;
}

void Friction_PWM(int16_t pwm1,int16_t pwm2)
{
	PWM1 = pwm1+1000;	
	PWM2 = pwm2+1000;
}


/**
  * @brief  TIM1,舵机(弹仓)初始化
  * @param  void
  * @retval void
  * @attention TIM1->CCR2,PE11
  */
void TIM1_Init(void)	
{
	GPIO_InitTypeDef          gpio;
	TIM_TimeBaseInitTypeDef   tim;
	TIM_OCInitTypeDef         oc;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);		

	gpio.GPIO_Pin = GPIO_Pin_11;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOE,&gpio);
	
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource11, GPIO_AF_TIM1);     
	
	tim.TIM_Prescaler     = 3360-1;
	tim.TIM_CounterMode   = TIM_CounterMode_Up;	 //向上计数
	tim.TIM_Period        = 999;                 //25ms	计数周期   1000 = 25ms   40 = 1ms   40-100 -  1ms - 2.5ms
	tim.TIM_ClockDivision = TIM_CKD_DIV1;		     //时钟分割,不为1则*2
	TIM_TimeBaseInit(TIM1,&tim);
	
	oc.TIM_OCMode = TIM_OCMode_PWM2;		//PWM2模式
	oc.TIM_OutputState = TIM_OutputState_Enable;		//输出比较使能
	oc.TIM_OutputNState = TIM_OutputState_Disable;	//互补输出比较失能
	oc.TIM_Pulse = 0;		//捕获脉冲值
	oc.TIM_OCPolarity = TIM_OCPolarity_Low;		//输出极性低
	oc.TIM_OCNPolarity = TIM_OCPolarity_High;		//互补输出极性高
	oc.TIM_OCIdleState = TIM_OCIdleState_Reset;		
	oc.TIM_OCNIdleState = TIM_OCIdleState_Set;		
	
	TIM_OC2Init(TIM1,&oc);		//通道2
	TIM_OC2PreloadConfig(TIM1,TIM_OCPreload_Enable);
				 
	TIM_ARRPreloadConfig(TIM1,ENABLE);
	
	TIM_CtrlPWMOutputs(TIM1,ENABLE);
	
	TIM_Cmd(TIM1,ENABLE);
	
	TIM1->CCR2 = 0;
}


void TIM3_Init(void)
{
	GPIO_InitTypeDef          gpio;
	TIM_TimeBaseInitTypeDef   tim;
	TIM_OCInitTypeDef         oc;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);	

	gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA,&gpio);
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource6, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7, GPIO_AF_TIM3);    
	
	tim.TIM_Prescaler = 84-1;//1MHz
	tim.TIM_CounterMode = TIM_CounterMode_Up;		
	tim.TIM_Period = 2499;   //25ms一个周期	,每+1代表时间+1微秒
	tim.TIM_ClockDivision = TIM_CKD_DIV1;		
	TIM_TimeBaseInit(TIM3,&tim);
	
	oc.TIM_OCMode = TIM_OCMode_PWM2;		
	oc.TIM_OutputState = TIM_OutputState_Enable;		
	oc.TIM_OutputNState = TIM_OutputState_Disable;	
	oc.TIM_Pulse = 0;		
	oc.TIM_OCPolarity = TIM_OCPolarity_Low;		
	oc.TIM_OCNPolarity = TIM_OCPolarity_High;		//输出极性高
	oc.TIM_OCIdleState = TIM_OCIdleState_Reset;		
	oc.TIM_OCNIdleState = TIM_OCIdleState_Set;		
	TIM_OC1Init(TIM3,&oc);		
	TIM_OC1PreloadConfig(TIM3,TIM_OCPreload_Enable);
	
	TIM_OC2Init(TIM3,&oc);		
	TIM_OC2PreloadConfig(TIM3,TIM_OCPreload_Enable);
				 
	TIM_ARRPreloadConfig(TIM3,ENABLE);
	
	TIM_CtrlPWMOutputs(TIM3,ENABLE);
	
	TIM_Cmd(TIM3,ENABLE);
	PWM1 = 1000;		//解锁摩擦轮,大于640
	PWM2 = 1000;
}



