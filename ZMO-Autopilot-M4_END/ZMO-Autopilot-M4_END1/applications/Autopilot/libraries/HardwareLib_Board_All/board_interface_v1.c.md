#include "board_interface.h"
#include "loco_config.h"
#include <rtthread.h>
#include <board.h>
#include <drv_common.h>
#include "drv_spi.h"
#include <stm32f4xx.h>
/************************************STM32F405/7VGT6**************************************/
/*--------------------------------------- V1 --------------------------------------------*/
//2024/3月新版，全采用标准GH1.25标准线序接口。
int get_loop_run_interval_ms(void)
{
    static int last_time= 0;
    int now_time = rt_tick_get();
    uint8_t dt_ms = (now_time-last_time);
    last_time = now_time;
    return dt_ms;
}

/*-------------------------- USB CONFIG  --------------------------*/
void HAL_PCD_MspInit(PCD_HandleTypeDef* hpcd)//USB硬件引脚、时钟初始化
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if(hpcd->Instance==USB_OTG_FS)
    {
        __HAL_RCC_GPIOA_CLK_ENABLE();
        //PA11     ------> USB_OTG_FS_DM
        //PA12     ------> USB_OTG_FS_DP
        GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
        /* Peripheral clock enable */
        __HAL_RCC_USB_OTG_FS_CLK_ENABLE();
        HAL_NVIC_SetPriority(OTG_FS_IRQn, 7, 0);
        HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
        /* USER CODE BEGIN USB_OTG_FS_MspInit 1 */
    }
}

/*-------------------------- PWM CONFIG  --------------------------*/
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim_pwm)
{
    if(htim_pwm->Instance==TIM1) //main1-main4 output
    {
        __HAL_RCC_TIM1_CLK_ENABLE();
    }
    if(htim_pwm->Instance==TIM5) //main5-main8 output
    {
        __HAL_RCC_TIM2_CLK_ENABLE();
    }
    if(htim_pwm->Instance==TIM4) //aux1-aux4 output
    {
        __HAL_RCC_TIM4_CLK_ENABLE();
    }
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim) //pwm输出引脚初始化
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(htim->Instance==TIM1)
  {
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_11|GPIO_PIN_13|GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
  }
  if(htim->Instance==TIM2)
  {
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
  if(htim->Instance==TIM4)
  {
    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
  }
}

/*-------------------------- SPI CONFIG  --------------------------*/
void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)/*SPI引脚初始化*/
{
    if(hspi->Instance == SPI1)  //EEPROM/TF-CARD
    {
        /* Peripheral clock enable */
        __HAL_RCC_SPI1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        GPIO_InitTypeDef GPIO_InitStruct = {0};
        GPIO_InitStruct.Pin = GPIO_PIN_5| GPIO_PIN_6 | GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
    if(hspi->Instance == SPI2) //imu1
    {
        /* Peripheral clock enable */
        __HAL_RCC_SPI2_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        GPIO_InitTypeDef GPIO_InitStruct = {0};
        GPIO_InitStruct.Pin = GPIO_PIN_13| GPIO_PIN_14 | GPIO_PIN_15;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
}

int spi_device_attach(void)
{
    /*******传感器片选引脚*********/
    rt_pin_mode(GET_PIN(B,12), PIN_MODE_OUTPUT); //陀螺仪片选 PB12
    rt_pin_mode(GET_PIN(D,10), PIN_MODE_OUTPUT); //气压计片选PD10
    rt_pin_mode(GET_PIN(B,0), PIN_MODE_OUTPUT);  //FLASH片选 B0
    rt_pin_mode(GET_PIN(A,4), PIN_MODE_OUTPUT);  //SD_CARD片选 PA4

    rt_pin_write(GET_PIN(B,12), PIN_HIGH);
    rt_pin_write(GET_PIN(D,10), PIN_HIGH);
    rt_pin_write(GET_PIN(B,0), PIN_HIGH);
    rt_pin_write(GET_PIN(A,4), PIN_HIGH);

    rt_hw_spi_device_attach("spi2",ICM20689_SPI_DEVICE_NAME, GPIOB, GPIO_PIN_12); //片选引脚PB12
    rt_hw_spi_device_attach("spi2",SPL06_SPI_DEVICE_NAME,    GPIOD, GPIO_PIN_10); //片选引脚PD10

    rt_hw_spi_device_attach("spi1",FM25V_SPI_DEVICE_NAME,    GPIOB, GPIO_PIN_0);  //片选引脚B0
    rt_hw_spi_device_attach("spi1",SD_CARD_SPI_DEVICE_NAME,  GPIOA, GPIO_PIN_4);  //片选引脚PA4
    return RT_EOK;
}
INIT_DEVICE_EXPORT(spi_device_attach);

/*-------------------------- ADC CONFIG  --------------------------*/
#define REFER_VOLTAGE       330         /* 参考电压 3.3V,数据精度乘以100保留2位小数*/
#define CONVERT_BITS        (1 << 12)   /* 转换位数为12位 */
void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)   //底层驱动,引脚配置,时钟使能
{
    GPIO_InitTypeDef GPIO_Initure;
    __HAL_RCC_ADC1_CLK_ENABLE();                 //使能ADC1时钟
    __HAL_RCC_GPIOC_CLK_ENABLE();                //开启GPIOC时钟

    GPIO_Initure.Pin=GPIO_PIN_0;                 //PC0
    GPIO_Initure.Mode=GPIO_MODE_ANALOG;          //模拟
    GPIO_Initure.Pull=GPIO_NOPULL;               //不带上下拉
    HAL_GPIO_Init(GPIOC,&GPIO_Initure);
}

ADC_HandleTypeDef ADC1_Handler;//ADC句柄
//通道值 0~16取值范围为：ADC_CHANNEL_0~ADC_CHANNEL_16
int MY_ADC_Init(void)
{
    ADC1_Handler.Instance=ADC1;
    ADC1_Handler.Init.ClockPrescaler=ADC_CLOCK_SYNC_PCLK_DIV4;   //4分频，ADCCLK=PCLK2/4=90/4=22.5MHZ
    ADC1_Handler.Init.Resolution=ADC_RESOLUTION_12B;             //12位模式
    ADC1_Handler.Init.DataAlign=ADC_DATAALIGN_RIGHT;             //右对齐
    ADC1_Handler.Init.ScanConvMode=DISABLE;                      //非扫描模式
    ADC1_Handler.Init.EOCSelection=DISABLE;                      //关闭EOC中断
    ADC1_Handler.Init.ContinuousConvMode=DISABLE;                //关闭连续转换
    ADC1_Handler.Init.NbrOfConversion=1;                         //1个转换在规则序列中 也就是只转换规则序列1
    ADC1_Handler.Init.DiscontinuousConvMode=DISABLE;             //禁止不连续采样模式
    ADC1_Handler.Init.NbrOfDiscConversion=0;                     //不连续采样通道数为0
    ADC1_Handler.Init.ExternalTrigConv=ADC_SOFTWARE_START;       //软件触发
    ADC1_Handler.Init.ExternalTrigConvEdge=ADC_EXTERNALTRIGCONVEDGE_NONE;//使用软件触发
    ADC1_Handler.Init.DMAContinuousRequests=DISABLE;             //关闭DMA请求
    HAL_ADC_Init(&ADC1_Handler);                                 //初始化
    return RT_EOK;
}

/* 导出到自动初始化 */
INIT_COMPONENT_EXPORT(MY_ADC_Init);

//获得ADC值
//ch: 通道值 0~16，取值范围为：ADC_CHANNEL_0~ADC_CHANNEL_16
//返回值:转换结果
uint16_t Get_Adc(uint32_t ch)
{
      ADC_ChannelConfTypeDef ADC1_ChanConf;

      ADC1_ChanConf.Channel=ch;                                   //通道
      ADC1_ChanConf.Rank=1;                                       //第1个序列，序列1
      ADC1_ChanConf.SamplingTime=ADC_SAMPLETIME_480CYCLES;        //采样时间
      ADC1_ChanConf.Offset=0;
      HAL_ADC_ConfigChannel(&ADC1_Handler,&ADC1_ChanConf);        //通道配置

      HAL_ADC_Start(&ADC1_Handler);                               //开启ADC

      HAL_ADC_PollForConversion(&ADC1_Handler,10);                //轮询转换

      return (uint16_t)HAL_ADC_GetValue(&ADC1_Handler);            //返回最近一次ADC1规则组的转换结果
}

//返回值:通道ch的转换结果
float get_adc_average(uint32_t ch)
{
    static float temp_val=0;
    static float temp_val_lpf=0;
    temp_val = (Get_Adc(ch) * REFER_VOLTAGE / CONVERT_BITS)/100.0f;

    temp_val_lpf += 0.5 *(temp_val*11.00f - temp_val_lpf);

    return temp_val_lpf;
}

/************************************************************************************
 *    遥控器ppm信号输入捕获gpio初始化
***************************************************************************************/
/* 消息队列控制块 */
static struct rt_messagequeue ppm_Queue;
static char msg_pool3[1024];
TIM_HandleTypeDef TIM8_Handler;         //定时器5句柄
int PPM_IN_Init(void)
{
    TIM_IC_InitTypeDef TIM8_CH3Config;

    TIM8_Handler.Instance=TIM8;                          //通用定时器5
    TIM8_Handler.Init.Prescaler=168-1;                   //分频      TIM5的时钟频率为200M
    TIM8_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;    //向上计数器
    TIM8_Handler.Init.Period=0XFFFFFFFF;                 //自动装载值
    TIM8_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_IC_Init(&TIM8_Handler);

    TIM8_CH3Config.ICPolarity=TIM_ICPOLARITY_RISING;     //上升沿捕获
    TIM8_CH3Config.ICSelection=TIM_ICSELECTION_DIRECTTI; //映射到TI1上
    TIM8_CH3Config.ICPrescaler=TIM_ICPSC_DIV1;           //配置输入分频,不分频
    TIM8_CH3Config.ICFilter=0;                           //配置输入滤波器,不滤波
    HAL_TIM_IC_ConfigChannel(&TIM8_Handler,&TIM8_CH3Config,TIM_CHANNEL_3); //配置TIM5通道1
    HAL_TIM_IC_Start_IT(&TIM8_Handler,TIM_CHANNEL_3);    //开始捕获TIM5的通道1
    __HAL_TIM_ENABLE_IT(&TIM8_Handler,TIM_IT_UPDATE);    //使能更新中断

    /* 初始化消息队列 */
    rt_mq_init(&ppm_Queue, "ppm_Queue",
               &msg_pool3[0],        /*内存池指向msg_pool */
               2,                    /*每个消息的大小是 128 - void*/
               sizeof(msg_pool3),    /*内存池的大小是msg_pool的大小*/
               RT_IPC_FLAG_FIFO);    /*如果有多个线程等待,按照FIFO的方法分配消息 */

    return RT_EOK;
}
/* 导出到自动初始化 */
INIT_COMPONENT_EXPORT(PPM_IN_Init);
//定时器5底层驱动，时钟使能，引脚配置
//此函数会被HAL_TIM_IC_Init()调用
//htim:定时器5句柄
void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef GPIO_Initure;
    __HAL_RCC_TIM8_CLK_ENABLE();            //使能TIM8时钟
    __HAL_RCC_GPIOC_CLK_ENABLE();           //开启GPIOC时钟

    GPIO_Initure.Pin=GPIO_PIN_8;           //PC8
    GPIO_Initure.Mode=GPIO_MODE_AF_PP;      //复用推挽输出
    GPIO_Initure.Pull=GPIO_PULLDOWN;        //下拉
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;     //高速
    GPIO_Initure.Alternate=GPIO_AF3_TIM8;   //PC8复用为TIM8通道3
    HAL_GPIO_Init(GPIOC,&GPIO_Initure);

    HAL_NVIC_SetPriority(TIM8_CC_IRQn,2,0);    //设置中断优先级，抢占优先级5，子优先级0
    HAL_NVIC_EnableIRQ(TIM8_CC_IRQn);          //开启ITM5中断
}

//定时器2中断服务函数
void TIM8_CC_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    HAL_TIM_IRQHandler(&TIM8_Handler); //定时器共用处理函数

    /* leave interrupt */
    rt_interrupt_leave();
}

//定时器输入捕获中断处理回调函数,该函数在HAL_TIM_IRQHandler中会被调用
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) //捕获中断发生时执行
{
    static uint16_t temp_cnt1,temp_cnt2;
    if ( GPIOC->IDR & GPIO_PIN_8)
    {
        temp_cnt1 = HAL_TIM_ReadCapturedValue(&TIM8_Handler,TIM_CHANNEL_3);   //获取当前的捕获值.

        TIM_RESET_CAPTUREPOLARITY(&TIM8_Handler,TIM_CHANNEL_3);                      //一定要先清除原来的设置！！
        TIM_SET_CAPTUREPOLARITY(&TIM8_Handler,TIM_CHANNEL_3,TIM_ICPOLARITY_FALLING); //配置TIM5通道1下升沿捕获
    }
    else
    {
        temp_cnt2 = HAL_TIM_ReadCapturedValue(&TIM8_Handler,TIM_CHANNEL_3);     //获取当前的捕获值.
        uint16_t _tmp;
        if ( temp_cnt2 >= temp_cnt1 )
        _tmp = temp_cnt2 - temp_cnt1;
        else
        _tmp = 0xffff - temp_cnt1 + temp_cnt2 + 1;
        rt_mq_send(&ppm_Queue,&_tmp, sizeof(rt_uint16_t));

        TIM_RESET_CAPTUREPOLARITY(&TIM8_Handler,TIM_CHANNEL_3);   //一定要先清除原来的设置！！
        TIM_SET_CAPTUREPOLARITY(&TIM8_Handler,TIM_CHANNEL_3,TIM_ICPOLARITY_RISING); //定时器5通道1设置为上降沿捕获
    }
}

int ppm_get_data(uint16_t *times)
{
    /* 从消息队列中接收消息 */
    return (rt_mq_recv(&ppm_Queue, times, sizeof(uint16_t), 0)== RT_EOK);
}

int get_ppm_Queue_num()
{
    /* 队列中已有的消息数 */
    return ppm_Queue.entry;
}

//static int vtor_config(void)
//{
//    /* Vector Table Relocation in Internal QSPI_FLASH */
//    SCB->VTOR = QSPI_BASE;
//    return 0;
//}
//INIT_BOARD_EXPORT(vtor_config);
