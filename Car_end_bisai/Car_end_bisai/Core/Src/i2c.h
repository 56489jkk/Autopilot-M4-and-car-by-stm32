
#ifndef __I2C_H_
#define __I2C_H_
//#include "stm32f10x.h"

//#include "stm32f1xx_hal_def.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"
//#include "sys.h"

#define mode_car 1//0:老车框架  1：新车框架


#define CAM_DEFAULT_I2C_ADDRESS       (0x34)			//I2C地址
#define MOTOR_TYPE_ADDR               20 				//编码电机类型设置寄存器地址
#define MOTOR_FIXED_SPEED_ADDR       	51				//速度寄存器地址。属于闭环控制
#define MOTOR_ENCODER_POLARITY_ADDR   21				//电机编码方向极性地址
#define MOTOR_FIXED_PWM_ADDR      		31				//固定PWM控制地址，属于开环控制
#define MOTOR_ENCODER_TOTAL_ADDR  		60				//4个编码电机各自的总脉冲值
#define ADC_BAT_ADDR                  0					//电压地址


//电机类型具体地址
#define MOTOR_TYPE_WITHOUT_ENCODER        0 		//无编码器的电机,磁环每转是44个脉冲减速比:90  默认
#define MOTOR_TYPE_TT                     1 		//TT编码电机
#define MOTOR_TYPE_N20                    2 		//N20编码电机
#define MOTOR_TYPE_JGB                    3 		//磁环每转是44个脉冲   减速比:90  默认

//*****软件模拟IIC，
//*****修改宏定义即可
//*****不同芯片时注意时钟和延时函数
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2))
#define GPIOB_ODR_Addr    (GPIOB_BASE+12) //0x40010C0C
#define GPIOB_IDR_Addr    (GPIOB_BASE+8) //0x40010C08
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr))
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum))
#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //输出 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //输入 

#define    IIC_IO_SDA      GPIO_PIN_12  //SDA的IO口
#define    IIC_IO_SCL      GPIO_PIN_1  //SCL的IO口
#define    GPIOX           GPIOB       //GPIOx选择
#define    CLOCK		   RCC_APB2Periph_GPIOB //时钟信号
 
#define    IIC_SCL         PBout(1) //SCL
#define    IIC_SDA         PBout(12) //输出SDA
#define    READ_SDA        PBin(12)  //输入SDA


void I2C_SDA_OUT(void);
void I2C_SDA_IN(void);
void IIC_init(void);
void IIC_start(void);
void IIC_stop(void);
void IIC_ack(void);
void IIC_noack(void);

void Control(void);
unsigned char IIC_wait_ack(void);
void IIC_send_byte(unsigned char txd);
unsigned char  IIC_read_byte(unsigned char ack);

uint8_t I2C_Read_Len(uint8_t Reg,uint8_t *Buf,uint8_t Len);
int8_t I2C_Write_Len(int8_t Reg,int8_t *Buf,int8_t Len);
#endif
