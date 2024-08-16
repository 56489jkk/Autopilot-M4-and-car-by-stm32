
#ifndef __I2C_H_
#define __I2C_H_
//#include "stm32f10x.h"

//#include "stm32f1xx_hal_def.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"
//#include "sys.h"

#define mode_car 1//0:�ϳ����  1���³����


#define CAM_DEFAULT_I2C_ADDRESS       (0x34)			//I2C��ַ
#define MOTOR_TYPE_ADDR               20 				//�������������üĴ�����ַ
#define MOTOR_FIXED_SPEED_ADDR       	51				//�ٶȼĴ�����ַ�����ڱջ�����
#define MOTOR_ENCODER_POLARITY_ADDR   21				//������뷽���Ե�ַ
#define MOTOR_FIXED_PWM_ADDR      		31				//�̶�PWM���Ƶ�ַ�����ڿ�������
#define MOTOR_ENCODER_TOTAL_ADDR  		60				//4�����������Ե�������ֵ
#define ADC_BAT_ADDR                  0					//��ѹ��ַ


//������;����ַ
#define MOTOR_TYPE_WITHOUT_ENCODER        0 		//�ޱ������ĵ��,�Ż�ÿת��44��������ٱ�:90  Ĭ��
#define MOTOR_TYPE_TT                     1 		//TT������
#define MOTOR_TYPE_N20                    2 		//N20������
#define MOTOR_TYPE_JGB                    3 		//�Ż�ÿת��44������   ���ٱ�:90  Ĭ��

//*****���ģ��IIC��
//*****�޸ĺ궨�弴��
//*****��ͬоƬʱע��ʱ�Ӻ���ʱ����
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2))
#define GPIOB_ODR_Addr    (GPIOB_BASE+12) //0x40010C0C
#define GPIOB_IDR_Addr    (GPIOB_BASE+8) //0x40010C08
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr))
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum))
#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //��� 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //���� 

#define    IIC_IO_SDA      GPIO_PIN_12  //SDA��IO��
#define    IIC_IO_SCL      GPIO_PIN_1  //SCL��IO��
#define    GPIOX           GPIOB       //GPIOxѡ��
#define    CLOCK		   RCC_APB2Periph_GPIOB //ʱ���ź�
 
#define    IIC_SCL         PBout(1) //SCL
#define    IIC_SDA         PBout(12) //���SDA
#define    READ_SDA        PBin(12)  //����SDA


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
