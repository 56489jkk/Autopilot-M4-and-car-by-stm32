/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-03-31     CGY       the first version
 */
#ifndef APPLICATIONS_AUTOPILOT_LIBRARIES_CORELIB_AHRS_CL_RC_CHANNEL_H_
#define APPLICATIONS_AUTOPILOT_LIBRARIES_CORELIB_AHRS_CL_RC_CHANNEL_H_
#include "loco_config.h"
enum
{
    ROL ,
    PIT ,
    THR ,
    YAW ,
    AUX1 ,
    AUX2 ,
    AUX3 ,
    AUX4 ,
    CH_NUM = 12
};

typedef enum
{
    LO = 0,
    CE = 1,
    HI = 2,
}CH_Pos;
extern CH_Pos channelPos[CH_NUM];      //辅助通道位置状态
// Structure used to detect and debounce switch changes
typedef struct
{
    int8_t debounce_position ;
    int8_t current_position ;
    uint32_t last_edge_time_ms;
} switch_t;

extern uint16_t  rc_ppm_in[CH_NUM], Throttle;
int raw_ppm_data_update(void);
void main_channel_value_update(void);
float get_channel_roll_control_in();   //线性数据
/*遥控数据单位化和加死区后的数据*/
float get_channel_pitch_control_in();  //线性数据
/*遥控数据单位化和加死区后的数据*/
float get_channel_yaw_control_in();    //线性数据
/*遥控数据单位化和加死区后的数据*/
float get_channel_thr_control_in();    //线性数据 -1 to 1
float get_one_way_throttle();          //线性数据 0 to 1  一般用于手动控制油门

bool _rc_health;
uint16_t get_flight_mode_channel_value(void);     //大概范围为   1000 ~ 2000 // Support for mode switches
uint16_t get_channel_value(int8_t ch_num);        //大概范围为   1000 ~ 2000
bool get_rc_health(void);
bool read_6pos_switch(int8_t *position);          //read a 6 position switch
bool switch_debounce_completed(int8_t position);  //一个状态切换到另一个状态做一个延时

void rc_sbus_data_receive(uint8_t com_data);
bool get_rc_sbus_health(void);

#endif /* APPLICATIONS_AUTOPILOT_LIBRARIES_CORELIB_AHRS_CL_RC_CHANNEL_H_ */
