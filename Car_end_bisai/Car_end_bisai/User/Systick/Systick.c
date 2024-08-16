#include "Systick.h"

void ms_delay(uint16_t nms)
{
	uint32_t temp;
	SysTick->LOAD = 9000*nms;
	SysTick->VAL=0X00;//��ռ�����
	SysTick->CTRL=0X01;//ʹ�ܣ����������޶����������ⲿʱ��Դ
 
	do
	{
		temp=SysTick->CTRL;//��ȡ��ǰ������ֵ
	}while((temp&0x01)&&(!(temp&(1<<16))));//�ȴ�ʱ�䵽��
    SysTick->CTRL=0x00; //�رռ�����
    SysTick->VAL =0X00; //��ռ�����
}

void Delay1_ms(int i)
{
	int a;
    for(a=1;a<=i;a++)
	{
		ms_delay(1);
	}
}

//nusΪҪ��ʱ��us��.
void DelayUs(uint32_t nus)
{
	uint32_t temp;
	SysTick->LOAD=nus*9; //ʱ�����
	SysTick->VAL=0x00;        //��ռ�����
	SysTick->CTRL=0x01 ;      //��ʼ����
	do
	{
		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));//�ȴ�ʱ�䵽��
	SysTick->CTRL=0x00;       //�رռ�����
	SysTick->VAL =0X00;       //��ռ�����
}


