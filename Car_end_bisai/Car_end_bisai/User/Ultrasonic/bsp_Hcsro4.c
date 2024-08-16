#include "bsp_Hcsro4.h"
#include "gpio.h"
#include "tim.h"
#include "usart.h"
#include "bsp_User_Debug.h"

//���������ʱ��
void HCSR_04_Init()
{
    uint32_t i;
    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);
    for (i = 0; i < 72 * 40; i++)
        __NOP();
    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
}
//�رճ�������ʱ��
void Stop_HCSR_04()
{
    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
}

//���㳬�������ľ���,����floatֵ��

//uint8_t TIM3CH2_CAPTURE_STA;  // ���벶��״̬
//uint16_t TIM3CH2_CAPTURE_VAL; //���벶��ֵ
uint8_t unit_change;
float GetSR04Distance()
{
    float len = 0;
    uint32_t time = 0;
    if (TIM3CH2_CAPTURE_STA & 0X80) // ���벶�� ����
    {
        time = TIM3CH2_CAPTURE_STA & 0X3f; // ����������
        time *= 65535;                     // һ�����Ϊ65536 �õ������ʱ��
        time += TIM3CH2_CAPTURE_VAL;       // �����ʱ��+���ڶ�ʱ����ֵ �õ��ܵ�ʱ��
        if (unit_change == 0)
        {
            len = (time * 342.62 * 100 / 2000000); // ����õ����� cm
        }
//        else if (unit_change == 1)
//        {
//            len = time * 342.62 * 100 / 200000000; // ����õ����� m
//            usart_printf("m\r\n");
//        }
 
        TIM3CH2_CAPTURE_STA = 0; // ������
    }
    return len;
}

//    ������ģ��������
//    char  dispBuff[100];
//	  float Count =0;
  
//    HAL_Delay(100);                     //��ʱ,������ʱ�Ļ�,�ᳬ������Ƶ��
//    HCSR_04_Init();                     //�������ģ��
//    Count = GetSR04Distance();
                                                       //ʹ��c��׼��ѱ���Countת�����ַ���
//    sprintf(dispBuff,"Distance : %.2fcm ",Count);   //��Count��ֵ��ʽ��Ϊ�ַ���,���洢��dispBuff�����С�
//    LCD_ClearLine(LINE(1));	  
//    ILI9341_DispStringLine_EN(LINE(1),dispBuff);   //��ʾdispBuff����

