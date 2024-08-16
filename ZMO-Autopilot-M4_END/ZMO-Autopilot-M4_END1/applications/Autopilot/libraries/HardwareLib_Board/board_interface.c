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

    GPIO_Initure.Pin=GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5;      //PC0  PC1  PC2  PC4  PC5
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

static float power_v_lpf=0,power_i_lpf=0,_5v_sens_lpf=0;

//返回值:通道ch的转换结果
float get_adc_power_voltage() //单位：V
{
    return power_v_lpf;
}
//返回值:通道ch的转换结果
float get_adc_power_current() //单位：A
{
    return power_i_lpf;
}
//返回值:通道ch的转换结果
float get_adc_5v_sens() //单位：V
{
    return _5v_sens_lpf;
}

float adc_measurement_update(float *scale) //单位：V
{
    float scale1[5];
    scale1[0]= *(scale+0);
    scale1[1]= *(scale+1);
    scale1[2]= *(scale+2);
    static float temp_val=0;

#ifdef POWER_ADC_V
    temp_val = (Get_Adc(POWER_ADC_V) * REFER_VOLTAGE / CONVERT_BITS)/100.0f;
    power_v_lpf += 0.5 *(temp_val*scale1[0] - power_v_lpf);
#endif

#ifdef POWER_ADC_I
    temp_val = (Get_Adc(POWER_ADC_I) * REFER_VOLTAGE / CONVERT_BITS)/100.0f;
    power_i_lpf += 0.5 *(temp_val*scale1[1] - power_i_lpf);
#endif

#ifdef VDD_5V_SENS_ADC1
    temp_val = (Get_Adc(VDD_5V_SENS_ADC1) * REFER_VOLTAGE / CONVERT_BITS)/100.0f;
    _5v_sens_lpf += 0.5 *(temp_val*scale1[2] - _5v_sens_lpf);
#endif

    return 0;
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

#define Hight_Data            ( 80  )                           //1 码相对计数值
#define Low_Data              ( 30  )                           //0 码相对计数值
#define Reste_Data            ( 80  )                           //80 复位电平相对计数值
#define Led_Num               ( 1   )                           //WS2812灯个数
#define Led_Data_Len          ( 24  )                           //WS2812数据长度，单个需要24个字节
#define WS2812_Data_Len       (Led_Num * Led_Data_Len)          //ws2812级联后需要的数组长度
uint16_t RGB_buffur[Reste_Data + WS2812_Data_Len] = { 0 }; //数据缓存数组

void WS2812_Display_1(uint32_t Color, uint16_t num)
{
    //指针偏移:需要跳过复位信号的N个0
    uint16_t* p = (RGB_buffur + Reste_Data) + (num * Led_Data_Len);

    for (uint8_t i = 0; i < 8; ++i)
    p[i+8]= (((Color << i) & 0X800000) ? Hight_Data :Low_Data);
    for (uint8_t i = 8; i < 16; ++i)
    p[i-8]= (((Color << i) & 0X800000) ? Hight_Data :Low_Data);
    for (uint8_t i = 16; i < 24; ++i)
    p[i]= (((Color << i) & 0X800000) ? Hight_Data :Low_Data);
}


TIM_HandleTypeDef g_timx_pwm_chy_handle;     /* 定时器x句柄 */
DMA_HandleTypeDef hdma_tim1_ch1;
void gtim_timx_pwm_chy_init(uint16_t arr, uint16_t psc)
{
    GPIO_InitTypeDef gpio_init_struct;
    __HAL_RCC_GPIOC_CLK_ENABLE();               /* 开启通道y的CPIO时钟 */
    __HAL_RCC_TIM3_CLK_ENABLE();

    gpio_init_struct.Pin = GPIO_PIN_6; /* 通道y的CPIO口 */
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;           /* 复用推完输出 */
    gpio_init_struct.Pull = GPIO_PULLUP;               /* 上拉 */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;     /* 高速 */
    gpio_init_struct.Alternate=GPIO_AF2_TIM3;         //PC8复用为TIM8通道3
    HAL_GPIO_Init(GPIOC, &gpio_init_struct);


    TIM_OC_InitTypeDef timx_oc_pwm_chy  = {0};                          /* 定时器PWM输出配置 */

    g_timx_pwm_chy_handle.Instance = TIM3;                              /* 定时器 */
    g_timx_pwm_chy_handle.Init.Prescaler = psc;                         /* 定时器分频 */
    g_timx_pwm_chy_handle.Init.CounterMode = TIM_COUNTERMODE_UP;        /* 递增计数模式 */
    g_timx_pwm_chy_handle.Init.Period = arr;                            /* 自动重装载值 */
    HAL_TIM_PWM_Init(&g_timx_pwm_chy_handle);                           /* 初始化PWM */

    timx_oc_pwm_chy.OCMode = TIM_OCMODE_PWM1;                           /* 模式选择PWM1 */
    timx_oc_pwm_chy.Pulse = 0;                                          /* 设置比较值,此值用来确定占空比 */
                                                                        /* 这里默认设置比较值为自动重装载值的一半,即占空比为50% */
    timx_oc_pwm_chy.OCPolarity = TIM_OCPOLARITY_HIGH;                   /* 输出比较极性为低 */
    HAL_TIM_PWM_ConfigChannel(&g_timx_pwm_chy_handle, &timx_oc_pwm_chy, TIM_CHANNEL_1); /* 配置TIMx通道y */
    HAL_TIM_PWM_Start(&g_timx_pwm_chy_handle, TIM_CHANNEL_1);          /* 开启对应PWM通道 */


    __HAL_RCC_DMA1_CLK_ENABLE();                      /* DMA1时钟使能 */
    __HAL_TIM_ENABLE_DMA(&g_timx_pwm_chy_handle,TIM_DMA_CC1);

    /* TIM3_CH1 Init */
    hdma_tim1_ch1.Instance = DMA1_Stream4;
    hdma_tim1_ch1.Init.Channel = DMA_CHANNEL_5;
    hdma_tim1_ch1.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tim1_ch1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim1_ch1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim1_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_tim1_ch1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_tim1_ch1.Init.Mode = DMA_CIRCULAR;
    hdma_tim1_ch1.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_tim1_ch1) != HAL_OK)
    {
       Error_Handler();
    }

    __HAL_LINKDMA(&g_timx_pwm_chy_handle,hdma[TIM_DMA_ID_CC1],hdma_tim1_ch1);
}

void WS2812_Display_2( uint8_t red, uint8_t green, uint8_t blue,uint16_t num)
{
    uint8_t i;
    uint32_t Color=(green << 16 | red << 8 | blue);//将2个8位数据合并转化为32位数据类型

    //指针偏移:需要跳过复位信号的N个0
    uint16_t* p = (RGB_buffur + Reste_Data) + (num * Led_Data_Len);

    for (i = 0; i < 24; ++i)    //对数组进行编辑
    p[i]= (((Color << i) & 0X800000) ? Hight_Data : Low_Data);

    HAL_DMA_Start(&hdma_tim1_ch1,(uint32_t)&RGB_buffur,(uint32_t)&g_timx_pwm_chy_handle.Instance->CCR1, 104);
}

//DMA 传输完成回调函数
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
   HAL_TIM_PWM_Stop_DMA(&g_timx_pwm_chy_handle,TIM_CHANNEL_1);
}

int PWM_IN_Init(void)
{
    gtim_timx_pwm_chy_init(105 - 1, 1 - 1);/* 84Mhz的计数频率,800Khz的PWM. */
    return RT_EOK;
}
/* 导出到自动初始化 */
INIT_COMPONENT_EXPORT(PWM_IN_Init);



