#include "i2c.h"
#include "Systick.h"
//#include "Delay.h"      //延时函数

void I2C_SDA_OUT(void)//SDA输出方向配置
{
  GPIO_InitTypeDef GPIO_InitStructure;	
	//RCC_APB2PeriphClockCmd(CLOCK,ENABLE);
	GPIO_InitStructure.Pin=IIC_IO_SDA;
	GPIO_InitStructure.Speed=GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStructure.Mode=GPIO_MODE_OUTPUT_PP;//SDA推挽输出
	HAL_GPIO_Init(GPIOX,&GPIO_InitStructure); 						
}

void I2C_SDA_IN(void)//SDA输入方向配置
{
	GPIO_InitTypeDef GPIO_InitStructure;	
	//RCC_APB2PeriphClockCmd(CLOCK,ENABLE);
	GPIO_InitStructure.Pin=IIC_IO_SDA;
	GPIO_InitStructure.Speed=GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStructure.Mode=GPIO_MODE_INPUT;//SCL上拉输入
	HAL_GPIO_Init(GPIOX,&GPIO_InitStructure);
}

//以下为模拟IIC总线函数
void IIC_init()
{
	#if mode_car == 1
	GPIO_InitTypeDef  GPIO_InitStructure;
	//RCC_APB2PeriphClockCmd(CLOCK, ENABLE);	 //使能PD端口时钟
	__HAL_RCC_GPIOB_CLK_ENABLE();
	GPIO_InitStructure.Pin = IIC_IO_SDA |IIC_IO_SCL;	//PD6配置为推挽输出,SCL
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP; 		
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;		 //IO口速度为50MHz
	HAL_GPIO_Init(GPIOX, &GPIO_InitStructure);					 //根据设定参数初始化GPIOD
	//GPIO_SetBits(GPIOX,IIC_IO_SDA|IIC_IO_SCL); 
	IIC_SDA=1;	  	  
	IIC_SCL=1;
	#endif
}

void IIC_start()	//起始信号
{
	I2C_SDA_OUT();
	IIC_SDA=1;	  	  
	IIC_SCL=1;
	DelayUs(5);
	IIC_SDA=0;
	DelayUs(5);
	IIC_SCL=0;
}

void IIC_stop()		//终止信号
{
	I2C_SDA_OUT();
	IIC_SCL=0;
	IIC_SDA=0;
	DelayUs(5);
	IIC_SCL=1; 
	IIC_SDA=1;
	DelayUs(5);
}

//主机产生一个应答信号
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

//主机不产生应答信号
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

//等待从机应答信号
//返回值：1 接收应答失败
//		  0 接收应答成功
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
	IIC_SCL=0;;//拉低时钟开始数据传输
	for(i=0;i<8;i++)
	{
		IIC_SDA=(txd&0x80)>>7;//读取字节
		txd<<=1;
		DelayUs(2);
		IIC_SCL=1;
		DelayUs(2); //发送数据
		IIC_SCL=0;
		DelayUs(2);
	}
}



//读取一个字节
unsigned char IIC_read_byte(unsigned char  ack)
{
	unsigned char i=0,receive=0;
	I2C_SDA_IN();
  for(i=0;i<8;i++)
  {
   	IIC_SCL=0;
		DelayUs(2);
		IIC_SCL=1;
		receive<<=1;//左移
		if(READ_SDA)
		   receive++;//连续读取八位
		DelayUs(1);	
  }

  if(!ack)
	  IIC_noack();
	else
		IIC_ack();

	return receive;//返回读取到的字节
}



//读取多个字节的数据（Reg：地址  Buf：数据内容 Len：数据长度）
uint8_t I2C_Read_Len(uint8_t Reg,uint8_t *Buf,uint8_t Len)
{
	uint8_t i;
	IIC_start();																				//发送起始信号
	IIC_send_byte((CAM_DEFAULT_I2C_ADDRESS << 1) | 0);	//发送元器件地址+写指令
	if(IIC_wait_ack() == 1)															//等待响应，如果失败，发送停止信号，返回1
	{
		IIC_stop();					
		return 1;					
	}
	IIC_send_byte(Reg);																	//发送寄存器地址
	if(IIC_wait_ack() == 1)															//等待响应，如果失败，发送停止信号，返回1
	{
		IIC_stop();
		return 1;
	}
	IIC_start();																				//发送起始信号
	IIC_send_byte((CAM_DEFAULT_I2C_ADDRESS << 1) | 1);	//发送元器件地址+读指令
	if(IIC_wait_ack() == 1)															//等待响应，如果失败，发送停止信号，返回1
	{
		IIC_stop();
		return 1;
	}
	for(i=0;i<Len;i++)																	//for循环Len次读取
	{	
		if(i != Len-1)																		//如果不是最后一次
		{
			Buf[i] = IIC_read_byte(1);											//保存第I次的数据到数组的第I位，并发送答应信号
		}
		else
			Buf[i] = IIC_read_byte(0);											//保存第I次的数据到数组的第I位，并发送非答应信号
	}
	IIC_stop();																					//发送停止信号
	return 0;																						//读取成功，返回0
}

//循环发送一个数组的数据（addr：地址  buf：数据内容  leng：数据长度）
int8_t I2C_Write_Len(int8_t Reg,int8_t *Buf,int8_t Len)//I2C的写数据
{
	uint8_t i;
	IIC_start();																				//在起始信号后必须发送一个7位从机地址+1位方向位，用“0”表示主机发送数据，“1”表示主机接收数据。
	IIC_send_byte((CAM_DEFAULT_I2C_ADDRESS << 1) | 0);	//发送 器件地址+写的命令
	if(IIC_wait_ack() == 1)															//等待响应，如果失败，发送停止信号，返回1
	{
		IIC_stop();					
		return 1;					
	}
	IIC_send_byte(Reg);																	//发送 寄存器地址
	if(IIC_wait_ack() == 1)															//等待响应，如果失败，发送停止信号，返回1
	{
		IIC_stop();
		return 1;
	}
	for(i =0;i<Len;i++)																	//循环 len 次写数据
	{
		IIC_send_byte(Buf[i]);														//发送第i位的8位数据
		if(IIC_wait_ack() == 1)														//等待响应，如果失败，发送停止信号，返回1
		{
			IIC_stop();
			return 1;
		}
	}
	IIC_stop();																					//发送结束，发送停止信号
	return 0;																						//返回 0，确定发送成功
}
