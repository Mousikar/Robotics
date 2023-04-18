  #include "led.h"
	
	void My_LED_Init(void)
	{
	GPIO_InitTypeDef  GPIO_InitStructure;							//定义一个结构体
	 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);	  //使能PC端口时钟，开启时钟
	 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;			    //引脚：LED0-->PB.5 端口配置
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 	//模式：推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度：IO口速度为50MHz
	 
  GPIO_Init(GPIOC, &GPIO_InitStructure);			      //初始化GPIOB.5
  GPIO_SetBits(GPIOC,GPIO_Pin_13);										//PB.5 输出高，把IO口拉高
	}

	