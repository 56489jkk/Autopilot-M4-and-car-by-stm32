/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-04-12     CGY       the first version
 */
#include "CL_Notify.h"
#include <rtdevice.h>
#include "board_interface.h"
#include "loco_config.h"
#include "mode_all.h"
#include "mavlink.h"

int led_init(void)
{
    /* 设定 LED 引脚为推挽输出模式 */
    rt_pin_mode(LED_R_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(LED_G_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(LED_B_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(LED_R_PIN, !GPIO_LED_ON);
    rt_pin_write(LED_G_PIN, !GPIO_LED_ON);
    rt_pin_write(LED_B_PIN, !GPIO_LED_ON);

#ifdef BSP_BUZZER_PIN
    rt_pin_mode(BSP_BUZZER_PIN, PIN_MODE_OUTPUT);  //蜂鸣器
    rt_pin_write(BSP_BUZZER_PIN, 0);
#endif

#ifdef BSP_SWITCHW_PIN
    rt_pin_mode(BSP_SWITCHW_PIN, PIN_MODE_INPUT);  //按键
#endif

    return  0;
}
/* 导出到自动初始化 */
INIT_DEVICE_EXPORT(led_init);


uint16_t led_accuracy =20;//该时间相当于控制led的周期  ms
float   LED_Brightness[3] = {0,20,0}; //TO 20 //RGB  刚启动，g色灯常亮
uint8_t RGB_Brightness[3] = {0,255,0}; //刚启动，g色灯常亮, 0灭、 255最亮
void LED_1ms_DRV() //0~20
{
    static uint16_t led_cnt[3];
    for(uint8_t i=0;i<3;i++)
    {
        if(led_cnt[i] < LED_Brightness[i])
        {
            switch(i)
            {
                case 0:
                    LED1_ON;
                    RGB_Brightness[0] = 255;
                    WS2812_Display_2(RGB_Brightness[0], RGB_Brightness[1], RGB_Brightness[2],0);
                break;
                case 1:
                    LED2_ON;
                    RGB_Brightness[1] = 255;
                    WS2812_Display_2(RGB_Brightness[0], RGB_Brightness[1], RGB_Brightness[2],0);
                break;
                case 2:
                    LED3_ON;
                    RGB_Brightness[2] = 255;
                    WS2812_Display_2(RGB_Brightness[0], RGB_Brightness[1], RGB_Brightness[2],0);
                break;
            }
        }
        else
        {
            switch(i)
            {
                case 0:
                    LED1_OFF;
                    RGB_Brightness[0] = 0;
                    WS2812_Display_2(RGB_Brightness[0], RGB_Brightness[1], RGB_Brightness[2],0);
                break;
                case 1:
                    LED2_OFF;
                    RGB_Brightness[1] = 0;
                    WS2812_Display_2(RGB_Brightness[0], RGB_Brightness[1], RGB_Brightness[2],0);
                break;
                case 2:
                    LED3_OFF;
                    RGB_Brightness[2] = 0;
                    WS2812_Display_2(RGB_Brightness[0], RGB_Brightness[1], RGB_Brightness[2],0);
                break;
            }
        }

        if(++led_cnt[i]>=led_accuracy)
        {
            led_cnt[i] = 0;
        }
    }
}

//                    调用周期            暗到亮的时间间隔
void ledBreath(uint8_t dT_ms,uint8_t led,uint16_t T)       //LED呼吸函数
{
    static uint8_t dir[3];
    uint8_t i;
    for(i=0; i<3; i++)
    {
        if(led & (1<<i))
        {
            switch(dir[i])
            {
                case 0:
                    LED_Brightness[i] += safe_div(led_accuracy,((float)T/(dT_ms)),0);
                    if(LED_Brightness[i]>led_accuracy)
                    {
                        dir[i] = 1;
                    }

                break;
                case 1:
                    LED_Brightness[i] -= safe_div(led_accuracy,((float)T/(dT_ms)),0);
                    if(LED_Brightness[i]<0)
                    {
                        dir[i] = 0;
                    }

                break;

                default:
                    dir[i] = 0;
                break;
            }
        }
        else
            LED_Brightness[i] = 0;
    }
}

void ledOnOff(uint8_t led) //
{
    uint8_t i;
    for(i=0; i<3; i++)
    {
        if(led & (1<<i))
            LED_Brightness[i] = 20;
        else
            LED_Brightness[i] = 0;
    }
}
                 //  调用周期                     亮时间         灭时间
void ledFlash(uint8_t dT_ms,uint8_t led, uint16_t on_ms,uint16_t off_ms) //LED闪烁函数
{
    static uint16_t tim_tmp;
    if(tim_tmp < on_ms)
        ledOnOff(led);
    else
        ledOnOff(0);
    tim_tmp += dT_ms;
    if(tim_tmp >= (on_ms + off_ms))
        tim_tmp = 0;
}

/*****************状态指示*********************/
void led_state_update(uint8_t dT_ms)
{
    //陀螺仪、加速度计、气压计未校准成功时  白色灯快闪烁
    if(!gyroIsCalibrationComplete(get_gyro_instance())||!acceIsCalibrationComplete(get_gyro_instance())||!baroIsCalibrationComplete())
    {
        ledFlash(dT_ms,BIT_WLED,40,40);
    }

    //正在校准电调 三个灯全常亮
    else if(get_esc_calibrating_state())
    {
        ledFlash(dT_ms,BIT_WLED,40,0);
    }

    else if(!get_opt_health()&&is_use_external_sensor(OpticalFlow_id))
    {
      ledBreath(dT_ms,BIT_CLED,300);
    }

    //失去遥控器信号时  第一个led呼吸
    else if(!get_rc_health())
    {
        ledBreath(dT_ms,BIT_RLED,350);
    }

    else if(!get_vision_health())
    {
        ledBreath(dT_ms,BIT_GLED,310);
    }
    else//无其他提示，正常显示模式档位及外置光流、Gps等状态
    {
        int8_t position=0;
        read_6pos_switch(&position);
        {
            if (position == 0) {
                ledFlash(dT_ms,BIT_RLED,800,800);
            }
            if (position == 3) {
                ledFlash(dT_ms,BIT_GLED,800,800);
            }
            if (position == 5) {
                ledFlash(dT_ms,BIT_BLED,800,800);
            }
        }
    }

    /*电池片的参数*/
    static float under_voltage=3.5;     //低电压    即开始报警的电压
    static float critical_voltage=3.1;  //临界电压  即开始降落的电压
    static float cells_min=3.0;         //锂电池放电的最低电压
    static float cells_max=4.2;         //锂电池充电的最大电压

    /* 采集电池连接稳定时的电压 用于以后的电池片数判断 */
    static uint16_t power_on_count=0,power_on_v=0,number_of_cells=0,low_counter=0;
    if (get_adc_power_voltage()>6) {
        power_on_count++;
    }
    else {
        power_on_count=0;
        number_of_cells=0;
        power_on_v = 0;
    }
    if (power_on_count==10) {                          //延时100ms
        power_on_v = get_adc_power_voltage();     //电池连接稳定时的电压
    }

    /*电池片数判断*/
    if (power_on_v>=cells_min*3&&power_on_v<=cells_max*3) {
        number_of_cells=3;
    }
    else if (power_on_v>=cells_min*4&&power_on_v<=cells_max*4) {
        number_of_cells=4;
    }
    else if (power_on_v>=cells_min*5&&power_on_v<=cells_max*5) {
        number_of_cells=5;
    }
    else if (power_on_v>=cells_min*6&&power_on_v<=cells_max*6) {
        number_of_cells=6;
    }

    /*电池低电量报警与低电量降落*/
    if (number_of_cells>0) {
        if (get_adc_power_voltage()<under_voltage*number_of_cells) {   //报警
            low_counter++;
        }
        else {
            low_counter=0;
        }

        if (low_counter>200) {
             rt_pin_write(BSP_BUZZER_PIN, 1);
        }
        else {
             rt_pin_write(BSP_BUZZER_PIN, 0);
        }

        if (get_adc_power_voltage() < critical_voltage*number_of_cells) {   //降落
            //set_flight_mode(LAND_Mode,UNKNOWN);  //低于10.2V自动降落
        }
    }else { //电池供电时不起作用

        if (rt_pin_read(BSP_SWITCHW_PIN)) {
            rt_pin_write(BSP_BUZZER_PIN, 1);
        } else {
            rt_pin_write(BSP_BUZZER_PIN, 0);
        }
    }
    /*调试读取打印电池电压*/
    //rt_kprintf("POWER:%d\r\n",(rt_uint16_t)(number_of_cells));
    //rt_kprintf("POWER:%d\r\n",(rt_uint16_t)(get_adc_average(POWER_ADC_V)*100));
}


/*******************************************************CPU占用率计算***************************************************************/

#include <rtthread.h>
#include <rthw.h>

#define CPU_USAGE_CALC_TICK    1000
#define CPU_USAGE_LOOP        1000


static rt_uint8_t  cpu_usage_major = 0, cpu_usage_minor= 0;
static rt_uint32_t total_count = 0;


static void cpu_usage_idle_hook(void)
{
    rt_tick_t tick;
    rt_uint32_t count;
    volatile rt_uint32_t loop;


    if (total_count == 0)
    {
        /* get total count */
        rt_enter_critical();
        tick = rt_tick_get();
        while(rt_tick_get() - tick < CPU_USAGE_CALC_TICK)
        {
            total_count ++;
            loop = 0;


            while (loop < CPU_USAGE_LOOP) loop ++;
        }
        rt_exit_critical();
    }


    count = 0;
    /* get CPU usage */
    tick = rt_tick_get();
    while (rt_tick_get() - tick < CPU_USAGE_CALC_TICK)
    {
        count ++;
        loop  = 0;
        while (loop < CPU_USAGE_LOOP) loop ++;
    }


    /* calculate major and minor */
    if (count < total_count)
    {
        count = total_count - count;
        cpu_usage_major = (count * 100) / total_count;
        cpu_usage_minor = ((count * 100) % total_count) * 100 / total_count;
    }
    else
    {
        total_count = count;
        /* no CPU usage */
        cpu_usage_major = 0;
        cpu_usage_minor = 0;
    }
}


void cpu_usage_get(rt_uint8_t *major, rt_uint8_t *minor)
{
    RT_ASSERT(major != RT_NULL);
    RT_ASSERT(minor != RT_NULL);


    *major = cpu_usage_major;
    *minor = cpu_usage_minor;
}


int cpu_usage_init(void)
{
    /* set idle thread hook */
    rt_thread_idle_sethook(cpu_usage_idle_hook);
    return RT_EOK;
}

/* 导出到自动初始化 */
//INIT_COMPONENT_EXPORT(cpu_usage_init);

