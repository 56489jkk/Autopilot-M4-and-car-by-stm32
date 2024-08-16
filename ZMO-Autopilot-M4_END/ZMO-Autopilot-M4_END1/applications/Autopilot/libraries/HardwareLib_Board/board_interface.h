#ifndef APPLICATIONS_AUTOPILOT_LIBRARIES_HARDWARELIB_BOARD_BOARD_INTERFACE_H_
#define APPLICATIONS_AUTOPILOT_LIBRARIES_HARDWARELIB_BOARD_BOARD_INTERFACE_H_
#include "loco_config.h"

/*
case 0://不旋转
case 1://顺时针旋转90度
case 2://顺时针旋转180度
case 3://顺时针旋转270度
case 4://反向0度
case 5://反向后顺时针旋转90度
case 6://反向后顺时针旋转180度
case 7://反向后顺时针旋转270度*/
#define Inertial_Sensor_Rotation  4
#define Compass_Sensor_Rotation   0
#define Flow_Sensor_Rotation  3

int get_loop_run_interval_ms(void);//V1
/*-------------------------- PWM CONFIG  --------------------------*/
#define BSP_USING_PWM1
#define BSP_USING_PWM1_CH1
#define BSP_USING_PWM1_CH2
#define BSP_USING_PWM1_CH3
#define BSP_USING_PWM1_CH4
#define PWM_DEV_NAME           "pwm1"  /* PWM设备名称 */

/*驱动电机所用的通道*/
#define MOTOR1_CH    1
#define MOTOR2_CH    2
#define MOTOR3_CH    3
#define MOTOR4_CH    4


/*-------------------------- PWM2 CONFIG  --------------------------*/
#define BSP_USING_PWM2
#define BSP_USING_PWM2_CH1
#define BSP_USING_PWM2_CH2
#define BSP_USING_PWM2_CH3
#define BSP_USING_PWM2_CH4
#define PWM2_DEV_NAME           "pwm2"  /* PWM设备名称 */

/*-------------------------- PWM4 CONFIG  --------------------------*/
#define BSP_USING_PWM4
#define BSP_USING_PWM4_CH1
#define BSP_USING_PWM4_CH2
#define BSP_USING_PWM4_CH3
#define BSP_USING_PWM4_CH4
#define PWM4_DEV_NAME           "pwm4"  /* PWM设备名称 */

/*-------------------------- ADC CONFIG  --------------------------*/
#define POWER_ADC_V     ADC_CHANNEL_10
#define POWER_ADC_I     ADC_CHANNEL_11

#define VDD_5V_SENS_ADC1    ADC_CHANNEL_12
#define SPARE1_ADC1         ADC_CHANNEL_14
#define SPARE2_ADC1         ADC_CHANNEL_15


/*-------------------------- SPI CONFIG  --------------------------*/
#define BSP_USING_SPI2     //陀螺仪 气压计
#define BSP_USING_SPI1     //FLASH/SD
/*-------------------------- USB CONFIG  --------------------------*/
#define BSP_USING_USBDEVICE

/*-------------------------- IIC CONFIG  --------------------------*/
#define BSP_USING_I2C1
#ifdef  BSP_USING_I2C1
#define BSP_I2C1_SCL_PIN    GET_PIN(B, 8)
#define BSP_I2C1_SDA_PIN    GET_PIN(B, 9)
#endif

#define BSP_USING_I2C2      //IST8310
#ifdef  BSP_USING_I2C2
#define BSP_I2C2_SCL_PIN    GET_PIN(B, 10)
#define BSP_I2C2_SDA_PIN    GET_PIN(B, 11)
#endif

/*-------------------------- UART CONFIG  --------------------------*/
// assume max 6 ports
#define SERIALMANAGER_NUM_PORTS 6   //串口的个数
#define BSP_USING_UART1
#define BSP_UART1_TX_PIN       "PA9"
#define BSP_UART1_RX_PIN       "PA10"

#define BSP_USING_UART2
#define BSP_UART2_TX_PIN       "PD5"
#define BSP_UART2_RX_PIN       "PD6"
#define BSP_UART2_TX_USING_DMA

#define BSP_USING_UART3
#define BSP_UART3_TX_PIN       "PD8"
#define BSP_UART3_RX_PIN       "PD9"

#define BSP_USING_UART4
#define BSP_UART4_TX_PIN       "PC10"
#define BSP_UART4_RX_PIN       "PC11"

#define BSP_USING_UART5
#define BSP_UART5_TX_PIN       "PC12"
#define BSP_UART5_RX_PIN       "PD2"

//#define BSP_USING_UART6        //RC_SBUS_RX
//#define BSP_UART6_TX_PIN       "PC6"
//#define BSP_UART6_RX_PIN       "PC7"

#define BSP_USING_CAN1
#define BSP_CAN1_TX_PIN       "PD1"
#define BSP_CAN1_RX_PIN       "PD0"

//allow to have have a dedicated safety switch pin
#define BSP_BUZZER_PIN     GET_PIN(E, 5)
#define BSP_SWITCHW_PIN    GET_PIN(D, 11)
#define BSP_LED_SAFETY_PIN GET_PIN(C, 6)

/*-------------------------- RGB OUT CONFIG  --------------------------*/
#define LED_R_PIN       GET_PIN(E, 2)
#define LED_G_PIN       GET_PIN(E, 3)
#define LED_B_PIN       GET_PIN(E, 4)
#define GPIO_LED_ON     0x00         //Common positive RGB,Low-level lighting

/** enable pins **/
#define VDD_5V_OUT1_EN_PIN           GET_PIN(E, 6)
#define VDD_5V_OUT2_EN_PIN           GET_PIN(E, 6)
#define VDD_3V3_SENSORS_EN_PIN       GET_PIN(E, 0)
#define VDD_3V3_SD_CARD_EN_PIN       GET_PIN(E, 1)

float get_adc_power_voltage(); //单位：V
float get_adc_power_current(); //单位：A
float get_adc_5v_sens(); //单位：V
float adc_measurement_update(float *scale); //单位：V

int PPM_IN_Init(void);
int ppm_get_data(uint16_t *times);
int get_ppm_Queue_num(void);
void WS2812_Display_2( uint8_t red, uint8_t green, uint8_t blue,uint16_t num);
#endif /* APPLICATIONS_AUTOPILOT_LIBRARIES_HARDWARELIB_BOARD_BOARD_INTERFACE_H_ */

