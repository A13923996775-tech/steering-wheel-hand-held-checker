#include "stm32f10x.h"                  // Device header

int main(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);    //开启GPIOC端口时钟  初始时钟为关闭状态
	GPIO_InitTypeDef GPIO_InintStructure;    //定义GPIOC初始化结构体变量
	GPIO_InintStructure.GPIO_Mode=GPIO_Mode_Out_PP;   //配置IO模式   推挽输出
	GPIO_InintStructure.GPIO_Pin=GPIO_Pin_13;   //指定引脚：PC13
	GPIO_InintStructure.GPIO_Speed=GPIO_Speed_50MHz;  //配置IO最高翻转速度50
	GPIO_Init(GPIOC,&GPIO_InintStructure);  //执行GPIO初始化
	GPIO_SetBits(GPIOC,GPIO_Pin_13);  //PC13置高电平 关灯
	GPIO_ResetBits(GPIOC,GPIO_Pin_13);  //PC13置低电平 开灯
	while(1)
	{
		
	}
}
