#include "myus.h"

void My_US_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStrue;//GPIO
	USART_InitTypeDef USART_InitStrue;//USART
	NVIC_InitTypeDef NVIC_InitStrue;//中断
	//定义三个结构体
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);//使能GPIO,用PA9和PA10
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//使能串口，串口，GPIO的时钟使能
	
	//初始化GPIO
	//PA9-TX,PA10-RX
	GPIO_InitStrue.GPIO_Mode=GPIO_Mode_AF_PP;//复用推挽
	GPIO_InitStrue.GPIO_Pin=GPIO_Pin_9;
	GPIO_InitStrue.GPIO_Speed=GPIO_Speed_10MHz;
	GPIO_Init(GPIOA,&GPIO_InitStrue);
	
	GPIO_InitStrue.GPIO_Mode=GPIO_Mode_IN_FLOATING;//浮空
	GPIO_InitStrue.GPIO_Pin=GPIO_Pin_10;
	GPIO_InitStrue.GPIO_Speed=GPIO_Speed_10MHz;
	GPIO_Init(GPIOA,&GPIO_InitStrue);
	
	//初始化串口
	USART_InitStrue.USART_BaudRate=9600;//9600,115200,38400,可以改
	USART_InitStrue.USART_HardwareFlowControl=USART_HardwareFlowControl_None;//不使用硬件流
	USART_InitStrue.USART_Mode=USART_Mode_Rx |USART_Mode_Tx ;//RXTX都要
	USART_InitStrue.USART_Parity=USART_Parity_No;//不用奇偶校验
	USART_InitStrue.USART_StopBits=USART_StopBits_1;//停止位为1
	USART_InitStrue.USART_WordLength=USART_WordLength_8b;//没有奇偶校验，8位；用奇偶校验，9位
	USART_Init(USART1,&USART_InitStrue);
	
	USART_Cmd(USART1,ENABLE);//使能串口
	
	//开启中断 
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);//ITConfig就是在什么情况下开启中断 接收缓存区非空
	
	//初始化中断
	NVIC_InitStrue.NVIC_IRQChannel=USART1_IRQn;//串口1的中断，可以改
	NVIC_InitStrue.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStrue.NVIC_IRQChannelPreemptionPriority=1;//抢占优先级
	NVIC_InitStrue.NVIC_IRQChannelSubPriority=1;
	NVIC_Init(&NVIC_InitStrue);//中断初始化


}

void USART1_IRQHandler(void)
{
	uint16_t res;
	if(USART_GetITStatus(USART1,USART_IT_RXNE)!= RESET)
	{
		res=USART_ReceiveData(USART1);
//		USART_SendData(USART1,res);
		
		if(res<=85&&res>0)
		{
				res=res-0;
				TIM_SetCompare2(TIM2,((1000*res-68000)/153+1500));//500/20000=0.5/20
				USART_SendData(USART1,((1000*res-68000)/153+1500));	
		}
		else if(res>85&&res<=170)
		{
				res=res-85;
				TIM_SetCompare3(TIM3,((1000*res-68000)/153+1500));//500/20000=0.5/20
				USART_SendData(USART1,((1000*res-68000)/153+1500));			
		}
		else if(res>170&&res<=255)
		{
				res=res-170;
				TIM_SetCompare4(TIM4,((1000*res-68000)/153+1500));//500/20000=0.5/20
				USART_SendData(USART1,((1000*res-68000)/153+1500));			
		}
		else
		{				
				TIM_SetCompare2(TIM2,1500);
				TIM_SetCompare3(TIM3,1500);
				TIM_SetCompare4(TIM4,1500);
				USART_SendData(USART1,res);			
		}
//		if(res<=0x32)//50)
//		{
//				res=res-0;
//				TIM_SetCompare2(TIM2,((res*2-20)*1000/90+1500));//500/20000=0.5/20
//				USART_SendData(USART1,((res*2-20)*1000/90+1500));	
//		}
//		else if(res>=0x64&&res<=0x96)
//		{
//				res=res-0x64;
//				TIM_SetCompare3(TIM3,((res*2-20)*1000/90+1500));//500/20000=0.5/20
//				USART_SendData(USART1,((res*2-20)*1000/90+1500));			
//		}
//		else if(res>=0xC8&&res<=0xFA)
//		{
//				res=res-0xC8;
//				TIM_SetCompare4(TIM4,((res*2-20)*1000/90+1500));//500/20000=0.5/20
//				USART_SendData(USART1,((res*2-20)*1000/90+1500));			
//		}
//		else
//		{				
//				TIM_SetCompare2(TIM2,1500);
//				TIM_SetCompare3(TIM3,1500);
//				TIM_SetCompare4(TIM4,1500);
//				USART_SendData(USART1,res);			
//		}

		USART_ClearITPendingBit(USART1,USART_IT_RXNE);

	}
}
//		switch(res)
//		{
//			case 0x01:
//				TIM_SetCompare2(TIM2,500*res);//500/20000=0.5/20
//				USART_SendData(USART1,res);
//				break;
//			case 0x02:
//				TIM_SetCompare2(TIM2,500*res);//500/20000=0.5/20
//				USART_SendData(USART1,res);
//				break;
//			case 0x03:
//				TIM_SetCompare2(TIM2,500*res);//500/20000=0.5/20
//				USART_SendData(USART1,res);
//				break;
//			case 0x04:
//				TIM_SetCompare2(TIM2,500*res);//500/20000=0.5/20
//				USART_SendData(USART1,res);
//				break;
//			default:
//			TIM_SetCompare2(TIM2,1500);//500/20000=0.5/20
//				USART_SendData(USART1,res);
//				break;
//		}	
