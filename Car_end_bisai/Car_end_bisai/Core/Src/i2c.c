#include "i2c.h"
#include "Systick.h"
//#include "Delay.h"      //��ʱ����

void I2C_SDA_OUT(void)//SDA�����������
{
  GPIO_InitTypeDef GPIO_InitStructure;	
	//RCC_APB2PeriphClockCmd(CLOCK,ENABLE);
	GPIO_InitStructure.Pin=IIC_IO_SDA;
	GPIO_InitStructure.Speed=GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStructure.Mode=GPIO_MODE_OUTPUT_PP;//SDA�������
	HAL_GPIO_Init(GPIOX,&GPIO_InitStructure); 						
}

void I2C_SDA_IN(void)//SDA���뷽������
{
	GPIO_InitTypeDef GPIO_InitStructure;	
	//RCC_APB2PeriphClockCmd(CLOCK,ENABLE);
	GPIO_InitStructure.Pin=IIC_IO_SDA;
	GPIO_InitStructure.Speed=GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStructure.Mode=GPIO_MODE_INPUT;//SCL��������
	HAL_GPIO_Init(GPIOX,&GPIO_InitStructure);
}

//����Ϊģ��IIC���ߺ���
void IIC_init()
{
	#if mode_car == 1
	GPIO_InitTypeDef  GPIO_InitStructure;
	//RCC_APB2PeriphClockCmd(CLOCK, ENABLE);	 //ʹ��PD�˿�ʱ��
	__HAL_RCC_GPIOB_CLK_ENABLE();
	GPIO_InitStructure.Pin = IIC_IO_SDA |IIC_IO_SCL;	//PD6����Ϊ�������,SCL
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP; 		
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;		 //IO���ٶ�Ϊ50MHz
	HAL_GPIO_Init(GPIOX, &GPIO_InitStructure);					 //�����趨������ʼ��GPIOD
	//GPIO_SetBits(GPIOX,IIC_IO_SDA|IIC_IO_SCL); 
	IIC_SDA=1;	  	  
	IIC_SCL=1;
	#endif
}

void IIC_start()	//��ʼ�ź�
{
	I2C_SDA_OUT();
	IIC_SDA=1;	  	  
	IIC_SCL=1;
	DelayUs(5);
	IIC_SDA=0;
	DelayUs(5);
	IIC_SCL=0;
}

void IIC_stop()		//��ֹ�ź�
{
	I2C_SDA_OUT();
	IIC_SCL=0;
	IIC_SDA=0;
	DelayUs(5);
	IIC_SCL=1; 
	IIC_SDA=1;
	DelayUs(5);
}

//��������һ��Ӧ���ź�
void IIC_ack()
{
	IIC_SCL=0;
	I2C_SDA_OUT();
  IIC_SDA=0;	
  DelayUs(2);
  IIC_SCL=1;
  DelayUs(2);
  IIC_SCL=0;	
}

//����������Ӧ���ź�
void IIC_noack()
{
	IIC_SCL=0;
	I2C_SDA_OUT();
  IIC_SDA=1;
  DelayUs(2);
  IIC_SCL=1;
  DelayUs(2);
  IIC_SCL=0;
}

//�ȴ��ӻ�Ӧ���ź�
//����ֵ��1 ����Ӧ��ʧ��
//		  0 ����Ӧ��ɹ�
unsigned char IIC_wait_ack()
{
	unsigned char tempTime=0;
	I2C_SDA_IN();
	IIC_SDA=1;
	DelayUs(1);
	IIC_SCL=1;
	DelayUs(1);

	while(READ_SDA)
	{
		tempTime++;
		if(tempTime>250)
		{
			IIC_stop();
			return 1;
		}	 
	}

	IIC_SCL=0;
	return 0;
}

void IIC_send_byte(unsigned char txd)
{
	unsigned char i=0;
	I2C_SDA_OUT();
	IIC_SCL=0;;//����ʱ�ӿ�ʼ���ݴ���
	for(i=0;i<8;i++)
	{
		IIC_SDA=(txd&0x80)>>7;//��ȡ�ֽ�
		txd<<=1;
		DelayUs(2);
		IIC_SCL=1;
		DelayUs(2); //��������
		IIC_SCL=0;
		DelayUs(2);
	}
}



//��ȡһ���ֽ�
unsigned char IIC_read_byte(unsigned char  ack)
{
	unsigned char i=0,receive=0;
	I2C_SDA_IN();
  for(i=0;i<8;i++)
  {
   	IIC_SCL=0;
		DelayUs(2);
		IIC_SCL=1;
		receive<<=1;//����
		if(READ_SDA)
		   receive++;//������ȡ��λ
		DelayUs(1);	
  }

  if(!ack)
	  IIC_noack();
	else
		IIC_ack();

	return receive;//���ض�ȡ�����ֽ�
}



//��ȡ����ֽڵ����ݣ�Reg����ַ  Buf���������� Len�����ݳ��ȣ�
uint8_t I2C_Read_Len(uint8_t Reg,uint8_t *Buf,uint8_t Len)
{
	uint8_t i;
	IIC_start();																				//������ʼ�ź�
	IIC_send_byte((CAM_DEFAULT_I2C_ADDRESS << 1) | 0);	//����Ԫ������ַ+дָ��
	if(IIC_wait_ack() == 1)															//�ȴ���Ӧ�����ʧ�ܣ�����ֹͣ�źţ�����1
	{
		IIC_stop();					
		return 1;					
	}
	IIC_send_byte(Reg);																	//���ͼĴ�����ַ
	if(IIC_wait_ack() == 1)															//�ȴ���Ӧ�����ʧ�ܣ�����ֹͣ�źţ�����1
	{
		IIC_stop();
		return 1;
	}
	IIC_start();																				//������ʼ�ź�
	IIC_send_byte((CAM_DEFAULT_I2C_ADDRESS << 1) | 1);	//����Ԫ������ַ+��ָ��
	if(IIC_wait_ack() == 1)															//�ȴ���Ӧ�����ʧ�ܣ�����ֹͣ�źţ�����1
	{
		IIC_stop();
		return 1;
	}
	for(i=0;i<Len;i++)																	//forѭ��Len�ζ�ȡ
	{	
		if(i != Len-1)																		//����������һ��
		{
			Buf[i] = IIC_read_byte(1);											//�����I�ε����ݵ�����ĵ�Iλ�������ʹ�Ӧ�ź�
		}
		else
			Buf[i] = IIC_read_byte(0);											//�����I�ε����ݵ�����ĵ�Iλ�������ͷǴ�Ӧ�ź�
	}
	IIC_stop();																					//����ֹͣ�ź�
	return 0;																						//��ȡ�ɹ�������0
}

//ѭ������һ����������ݣ�addr����ַ  buf����������  leng�����ݳ��ȣ�
int8_t I2C_Write_Len(int8_t Reg,int8_t *Buf,int8_t Len)//I2C��д����
{
	uint8_t i;
	IIC_start();																				//����ʼ�źź���뷢��һ��7λ�ӻ���ַ+1λ����λ���á�0����ʾ�����������ݣ���1����ʾ�����������ݡ�
	IIC_send_byte((CAM_DEFAULT_I2C_ADDRESS << 1) | 0);	//���� ������ַ+д������
	if(IIC_wait_ack() == 1)															//�ȴ���Ӧ�����ʧ�ܣ�����ֹͣ�źţ�����1
	{
		IIC_stop();					
		return 1;					
	}
	IIC_send_byte(Reg);																	//���� �Ĵ�����ַ
	if(IIC_wait_ack() == 1)															//�ȴ���Ӧ�����ʧ�ܣ�����ֹͣ�źţ�����1
	{
		IIC_stop();
		return 1;
	}
	for(i =0;i<Len;i++)																	//ѭ�� len ��д����
	{
		IIC_send_byte(Buf[i]);														//���͵�iλ��8λ����
		if(IIC_wait_ack() == 1)														//�ȴ���Ӧ�����ʧ�ܣ�����ֹͣ�źţ�����1
		{
			IIC_stop();
			return 1;
		}
	}
	IIC_stop();																					//���ͽ���������ֹͣ�ź�
	return 0;																						//���� 0��ȷ�����ͳɹ�
}
