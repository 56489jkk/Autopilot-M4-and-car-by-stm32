/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-03-31     CGY       the first version
 */
#include "CL_RC_Channel.h"
#include "loco_config.h"
uint16_t sbus_ch[18]={0};

int32_t last_sbus_update_ms;
bool get_rc_sbus_health()
{
    if(rt_tick_get() - last_sbus_update_ms > 100){
        return  false;
    }
    else  return  true;
}

void rc_sbus_data_receive(uint8_t com_data)
{
    static uint8_t data_len = 0;
    static uint8_t state = 0;
    static uint8_t sbus_len = 0;
    static uint8_t sbus_buf[30];
    if(state==0&&com_data==0x0f)   //帧头
    {
        state=1;
        data_len = 24;             //帧长    24
        sbus_len = 0;
    }
    else if(state==1&&data_len>0)  //帧头֡
    {
        data_len--;
        sbus_buf[sbus_len++] = com_data;
        if(data_len==0&&sbus_buf[23]==0)//数据长度减到零
        {
             sbus_ch[0] = ((int16_t)sbus_buf[0] >> 0| ((int16_t)sbus_buf[1] << 8 )) & 0x07FF;
             sbus_ch[1] = ((int16_t)sbus_buf[1] >> 3| ((int16_t)sbus_buf[2] << 5 )) & 0x07FF;
             sbus_ch[2] = ((int16_t)sbus_buf[2] >> 6| ((int16_t)sbus_buf[3] << 2 ) | (int16_t)sbus_buf[4] << 10 ) & 0x07FF;
             sbus_ch[3]=  ((int16_t)sbus_buf[4] >> 1| ((int16_t)sbus_buf[5] << 7 )) & 0x07FF;
             sbus_ch[4] = ((int16_t)sbus_buf[5] >> 4| ((int16_t)sbus_buf[6] << 4 )) & 0x07FF;
             sbus_ch[5] = ((int16_t)sbus_buf[6] >> 7| ((int16_t)sbus_buf[7] << 1 ) | (int16_t)sbus_buf[8] << 9 ) & 0x07FF;
             sbus_ch[6] = ((int16_t)sbus_buf[8] >> 2| ((int16_t)sbus_buf[9] << 6 )) & 0x07FF;
             sbus_ch[7] = ((int16_t)sbus_buf[9] >> 5| ((int16_t)sbus_buf[10] << 3 )) & 0x07FF;
             last_sbus_update_ms = rt_tick_get();
        }
    }
    else
    {
        state  =0;
    }

}


/* 结构体数组,指示每个通道低中高三种状态 */
CH_Pos channelPos[CH_NUM];      //辅助通道位置状态
int16_t rc_original[CH_NUM] = {0};                  //储存遥控器原始数据
float rc_value_unit[4];                             //遥控器原始数据加死区后单位化
uint16_t  rc_ppm_in[CH_NUM], rc_pulse[CH_NUM];
int raw_ppm_data_update(void)
{
    static uint16_t ppm=0,ch_sta = 0;               //暂存，用来解析RC数据
    if(ppm_get_data(&ppm))
    {
        if(ppm > 2100 || ch_sta == CH_NUM )
        {
             ch_sta = 0;
        }
        else
        {
            rc_pulse[ch_sta] = (ppm+500);  //接收到的信号数值大约在1000 到 2000之间


            rc_ppm_in[ch_sta] = rc_pulse[ch_sta];

            ch_sta++;
        }
    }


//    static uint16_t rc_sbus_in[18];
//    for (uint8_t i = 0; i < 8; ++i) {
//
//        rc_sbus_in[i] = 1000 + (int)((sbus_ch[i]-200)*(1000/(float)(1800-200)));
//        rc_ppm_in[i] = rc_sbus_in[i];
//    }

    return RT_EOK;
}

/* 遥控数据单位化和去除死区 */
static float data_to_unit_dead(int16_t rawData, int16_t deadband)
{
    float norm = ((float)data_to_deadzone(rawData,0,deadband) / (float)(500-deadband));
    return norm;
}
/* 遥控数据转换为速率 */
float rcDataToRate(float norm, float max_rate)
{
  return norm * max_rate;
}
/* 遥控数据转换为角度 */
float rcDataToAngle(float norm, float max_angle)
{
   return norm * max_angle;
}

void main_channel_value_update(void)
{
    for(uint8_t i=0; i<CH_NUM; ++i)
    {
        if(rc_ppm_in[i] != 0)                                           //如果该通道有值,处理通道数据
        {
            if (rc_ppm_in[1]<910||rc_ppm_in[2]<910) {
                _rc_health = false;
            }
            else  _rc_health = true;

            rc_original[i] = 1.1f *(rc_ppm_in[i] - 1500);               //处理成+-500摇杆量,1.21是为遥控器做的适配

            rc_original[i] = (int16_t)limit(rc_original[i],-500,500);   //限制到+—500

        }
        /*这里要改一下,这个样子不能设置失控保护*/
        else
        {
            rc_original[i] = 0;                                         //没值全部默认设为中位(1500)//遥控器不开相当于遥控器输入0
            _rc_health = false;
        }


        if(rc_original[i] < (-300))
        {
            channelPos[i] = LO;        //处理通道 档位
        }
        else if(rc_original[i] > (300))
        {
            channelPos[i] = HI;         //高档
        }
        else
        {
            channelPos[i] = CE;         //中档
        }
    }

    /*遥控数据单位化和加死区*/
    rc_value_unit[ROL] =  data_to_unit_dead(rc_original[ROL], 40);
    rc_value_unit[PIT] =  data_to_unit_dead(rc_original[PIT], 40);
    rc_value_unit[YAW] =  data_to_unit_dead(rc_original[YAW], 50);
    rc_value_unit[THR] =  data_to_unit_dead(rc_original[THR], 100);

}
bool get_rc_health(void)
{
   return  _rc_health;
}

/*遥控数据单位化和加死区后的数据*/
float get_channel_roll_control_in()  //线性数据 -1 to 1
{
    return  rc_value_unit[ROL];
}

/*遥控数据单位化和加死区后的数据*/
float get_channel_pitch_control_in()  //线性数据 -1 to 1
{
    return  rc_value_unit[PIT];
}

/*遥控数据单位化和加死区后的数据*/
float get_channel_yaw_control_in()   //线性数据 -1 to 1
{
    return  rc_value_unit[YAW];
}

/*遥控数据单位化和加死区后的数据*/
float get_channel_thr_control_in()   //线性数据 -1 to 1
{
    return  rc_value_unit[THR];
}

/*遥控数据单位化和加死区后的数据*/
float get_one_way_throttle()   //线性数据 0 to 1  一般用于手动控制油门
{
    if(rc_ppm_in[THR] == 0) return 0;

    return  (rc_original[THR]+500)*0.001f;
}

uint16_t get_channel_value(int8_t ch_num) //大概范围为   1000 ~ 2000
{
    const int8_t num = ch_num;           //通道由ch_num定义
    if (num <= 0) {
        return RT_NULL;
    }
    if (num >= 16) {
        return RT_NULL;
    }
    return  rc_ppm_in[num-1];
}

// Support for mode switches
uint16_t get_flight_mode_channel_value() //大概范围为   1000 ~ 2000
{
    const int8_t num = 5;                //用通道5控制模式
    if (num <= 0) {
        return RT_NULL;
    }
    if (num >= 16) {
        return RT_NULL;
    }
    return  rc_ppm_in[num-1];
}

switch_t  switch_state={-1,-1,0};
#define SWITCH_DEBOUNCE_TIME_MS  100  //多段开关拨到下一个状态的延时时间。
bool switch_debounce_completed(int8_t position)  //一个状态切换到另一个状态做一个延时
{
    // switch change not detected
    if (switch_state.current_position == position) {
        // reset debouncing
         switch_state.debounce_position = position;
    } else {
        // switch change detected
        const uint32_t tnow_ms = rt_tick_get();

        // position not established yet
        if (switch_state.debounce_position != position) {
            switch_state.debounce_position = position;
            switch_state.last_edge_time_ms = tnow_ms;
        } else if (tnow_ms - switch_state.last_edge_time_ms >= SWITCH_DEBOUNCE_TIME_MS) {
            // position estabilished; debounce completed
            switch_state.current_position = position;
            return true;
        }
    }
    return false;
}

// read a 6 position switch
bool read_6pos_switch(int8_t *position)  //读取模式控制通道的位置，通道值分成六段
{
    // calculate position of 6 pos switch
    const uint16_t pulse_width = get_flight_mode_channel_value(); //获取飞行模式控制的通道值
    if (pulse_width <= 950 || pulse_width >= 2050) {
        return false;  // This is an error condition
    }

    if      (pulse_width < 1231) *position = 0;
    else if (pulse_width < 1361) *position = 1;
    else if (pulse_width < 1491) *position = 2;
    else if (pulse_width < 1621) *position = 3;
    else if (pulse_width < 1750) *position = 4;
    else *position = 5;

    //判断开关状态是否切换成功
    if (!switch_debounce_completed(*position)||!get_rc_health()) {
        return false;
    }
    return true;
}
