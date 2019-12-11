#include "usart4.h"

uint8_t uart4Buf[VISION_BUF_SIZE]={0};



void UART4_Init(void)
{
	USART_InitTypeDef  usart;
	GPIO_InitTypeDef   gpio;
	NVIC_InitTypeDef   nvic;
	DMA_InitTypeDef dma;
	
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_DMA1,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);

	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_UART4);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_UART4);

	gpio.GPIO_Pin   = GPIO_Pin_0;
	gpio.GPIO_Mode  = GPIO_Mode_AF;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	gpio.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_Init(GPIOA,&gpio);

	gpio.GPIO_Pin = GPIO_Pin_1;
	GPIO_Init(GPIOA, &gpio); 	
	
	
	usart.USART_BaudRate            = 115200;   
	usart.USART_WordLength          = USART_WordLength_8b;
	usart.USART_StopBits            = USART_StopBits_1;
	usart.USART_Parity              = USART_Parity_No;
	usart.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;
	usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	
	USART_Init(UART4,&usart);
	USART_Cmd(UART4,ENABLE);
	
	USART_ITConfig(UART4,USART_IT_IDLE,ENABLE); // 注意要配置成串口空闲中断 

	USART_DMACmd(UART4,USART_DMAReq_Rx,ENABLE);
	USART_DMACmd(UART4,USART_DMAReq_Tx,ENABLE);
	
	DMA_DeInit(DMA1_Stream2);
	dma.DMA_Channel = DMA_Channel_4;

	dma.DMA_DIR = DMA_DIR_PeripheralToMemory;//方向外设到存储器

	dma.DMA_PeripheralBaseAddr  = (uint32_t)&(UART4->DR);
	dma.DMA_Memory0BaseAddr     = (uint32_t)uart4Buf;
	dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
	dma.DMA_BufferSize = VISION_BUF_SIZE;
	dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
	dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	dma.DMA_Mode = DMA_Mode_Circular;
	dma.DMA_Priority = DMA_Priority_VeryHigh;
	dma.DMA_FIFOMode = DMA_FIFOMode_Enable;
	dma.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

	DMA_Init(DMA1_Stream2,&dma);	
	DMA_Cmd(DMA1_Stream2,ENABLE);  // stream2
	
	nvic.NVIC_IRQChannel                    = UART4_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority  = 0;
	nvic.NVIC_IRQChannelSubPriority         = 1;
	nvic.NVIC_IRQChannelCmd                 = ENABLE;
	NVIC_Init(&nvic);
}



/**
 *	@brief	串口4中断函数
 */
extern VisionRxDataObj visionData;

uint32_t visionFPS=0;
void UART4_IRQHandler( void )
{
	uint8_t res;
	static uint32_t sysCnt=0,lastCnt=0;

	if(USART_GetITStatus(UART4, USART_IT_IDLE) != RESET)
	{		
		//根据ST官方手册,读一下SR和DR寄存器,IDLE才能再次使用,否则会一直进入中断,就会跟串口接收中断没区别
		res = UART4->SR ;
		res = UART4->DR ;
		
		sysCnt = millis();
		//获取当前时间
		/* 空闲中断 */
		visionFPS = sysCnt - lastCnt;
		//视觉帧数
		lastCnt = sysCnt;
		//上一次时间
		
		
		DMA_Cmd(DMA1_Stream2, DISABLE);
		
		res = VISION_BUF_SIZE - DMA_GetCurrDataCounter(DMA1_Stream2);
		
		memcpy(&visionData,uart4Buf,res);
		
		VisionProcess();
		//到时候在这里加上一个数据CRC校验的函数
		
		memset(uart4Buf, 0, VISION_BUF_SIZE);	// 读完之后内容清零
		DMA_Cmd(DMA1_Stream2, ENABLE);
	}
}


/**
  * @brief  串口一次发送一个字节数据
  * @param  自己打包好的要发给裁判的数据
  * @retval void
  * @attention  串口移位发送
  */
void UART4_sendChar(uint8_t cData)
{
	while (USART_GetFlagStatus( UART4, USART_FLAG_TC ) == RESET);
	
	USART_SendData( UART4, cData );   
}


