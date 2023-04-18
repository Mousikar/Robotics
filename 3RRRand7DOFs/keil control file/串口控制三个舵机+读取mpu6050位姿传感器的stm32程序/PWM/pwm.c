#include "pwm.h"
//PA2,PA3 TIM2CH34
void My_TIM2_Init(u16 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStruct;//GPIO
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;//定时器
	TIM_OCInitTypeDef TIM_OCInitStruct;//通道
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);//GPIO时钟开启
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);//定时器2的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);//复用功能，开启AFIO时钟
	
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF_PP;//复用推挽
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_2|GPIO_Pin_3;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	
	TIM_TimeBaseInitStruct.TIM_ClockDivision=TIM_CKD_DIV1;//分频因子
	TIM_TimeBaseInitStruct.TIM_CounterMode=TIM_CounterMode_Up;//向上计数
	TIM_TimeBaseInitStruct.TIM_Period=arr;//自动重装载值
	TIM_TimeBaseInitStruct.TIM_Prescaler=psc;//预分频系数
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);
	
	TIM_OCInitStruct.TIM_OCMode=TIM_OCMode_PWM1;
	TIM_OCInitStruct.TIM_OCNPolarity=TIM_OCPolarity_High;
	TIM_OCInitStruct.TIM_OutputState=TIM_OutputState_Enable;
	
	
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	TIM_OC3Init(TIM2,&TIM_OCInitStruct);//通道3
	TIM_OC4Init(TIM2,&TIM_OCInitStruct);//通道4
	
	TIM_OC3PreloadConfig(TIM2,TIM_OCPreload_Enable);        //使能预装载寄存器
	TIM_OC4PreloadConfig(TIM2,TIM_OCPreload_Enable);
  TIM_Cmd(TIM2,ENABLE);        //使能TIM2
	
	//	TIM_SetCompare2(TIM2,1500);//设置比较值
	//	TIM_SetCompare3(TIM2,2000);//设置比较值
	
}
