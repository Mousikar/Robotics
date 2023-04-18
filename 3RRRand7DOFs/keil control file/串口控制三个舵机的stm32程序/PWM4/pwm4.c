#include "pwm4.h"
//PB3 TIM2CH2
//PB0 TIM3CH3
//PB9 TIM4CH4
void My_TIM4_Init(u16 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStruct;//GPIO
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;//定时器
	TIM_OCInitTypeDef TIM_OCInitStruct;//通道
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);//GPIO时钟开启
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);//定时器3的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);//复用功能，开启AFIO时钟
	
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF_PP;//复用推挽
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_9;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	TIM_TimeBaseInitStruct.TIM_ClockDivision=TIM_CKD_DIV1;//分频因子
	TIM_TimeBaseInitStruct.TIM_CounterMode=TIM_CounterMode_Up;//向上计数
	TIM_TimeBaseInitStruct.TIM_Period=arr;//自动重装载值
	TIM_TimeBaseInitStruct.TIM_Prescaler=psc;//预分频系数
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStruct);
	
	TIM_OCInitStruct.TIM_OCMode=TIM_OCMode_PWM1;
	TIM_OCInitStruct.TIM_OCNPolarity=TIM_OCPolarity_High;
	TIM_OCInitStruct.TIM_OutputState=TIM_OutputState_Enable;
	
	
	TIM_OC4Init(TIM4,&TIM_OCInitStruct);//通道3
	
	TIM_OC4PreloadConfig(TIM4,TIM_OCPreload_Enable);        //使能预装载寄存器
	
  TIM_Cmd(TIM4,ENABLE);        //使能TIM2
	
	//	TIM_SetCompare2(TIM2,1500);//设置比较值
	//	TIM_SetCompare3(TIM2,2000);//设置比较值
	
}
