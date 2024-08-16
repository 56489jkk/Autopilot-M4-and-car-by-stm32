#include "Systick.h"

void ms_delay(uint16_t nms)
{
	uint32_t temp;
	SysTick->LOAD = 9000*nms;
	SysTick->VAL=0X00;//清空计数器
	SysTick->CTRL=0X01;//使能，减到零是无动作，采用外部时钟源
 
	do
	{
		temp=SysTick->CTRL;//读取当前倒计数值
	}while((temp&0x01)&&(!(temp&(1<<16))));//等待时间到达
    SysTick->CTRL=0x00; //关闭计数器
    SysTick->VAL =0X00; //清空计数器
}

void Delay1_ms(int i)
{
	int a;
    for(a=1;a<=i;a++)
	{
		ms_delay(1);
	}
}

//nus为要延时的us数.
void DelayUs(uint32_t nus)
{
	uint32_t temp;
	SysTick->LOAD=nus*9; //时间加载
	SysTick->VAL=0x00;        //清空计数器
	SysTick->CTRL=0x01 ;      //开始倒数
	do
	{
		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));//等待时间到达
	SysTick->CTRL=0x00;       //关闭计数器
	SysTick->VAL =0X00;       //清空计数器
}


