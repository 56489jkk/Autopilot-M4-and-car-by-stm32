/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-04-01     CGY       the first version
 */
#include "loco_config.h"
#include "mode_all.h"
#include "math.h"
enum ModeReason _last_reason;
uint16_t _new_flightmode=0;
/***********************************************************
  *@brief  获取最新的飞行模式。
*************************************************************/
uint16_t get_new_mode()
{
    return   _new_flightmode;
}
/***********************************************************
  *@brief  设置飞行模式。
  *@param  mode：飞行模式的编号
  *@param  reason： 切换飞行模式的原因
*************************************************************/
bool set_flight_mode(enum FlightMode mode, enum ModeReason reason)
{
    uint16_t new_flightmode = mode;
    set_mode_cnt++;

#ifdef DISALLOW_GCS_MODE_CHANGE_DURING_RC_FAILSAFE
    if (reason == ModeReason::GCS_COMMAND && copter.failsafe.radio) {
        // don't allow mode changes while in radio failsafe
        return false;
    }
#endif

    /***/
    if (mode==ALT_HOLD_Mode&&!get_opt_health()) {

        return false;
    }
    /***/
    if (mode==FLOWHOLD_Mode&&!get_opt_health()) {

        return false;
    }

    // update last reason
    const enum ModeReason last_reason = _last_reason;
    _last_reason = reason;

    // return immediately if we are already in the desired mode
    if (mode == get_new_mode()) {

        if (1 && (reason != last_reason)) {

        }
        return true;
    }

    //初始化刚设置的飞行模式
    switch(new_flightmode) {
        case ACRO_Mode:

        break;
        case STABILIZE_Mode:
            mode_stabilize_initialization();         //自稳模式
        break;
        case ALT_HOLD_Mode:
            mode_altitude_hold_initialization();     //定高模式
        break;
        case POSHOLD_Mode:
            mode_position_hold_initialization();     //定点模式
        break;
        case FLOWHOLD_Mode:
            mode_flow_hold_initialization();         //定点模式
        break;
        case GUIDED_Mode:
            mode_guided_initialization();
        break;
        case LAND_Mode:
            mode_land_initialization();
        break;
        case CIRCLE_Mode:
            mode_circle_initialization();
        break;


        default:
            return false;
        break;
    }

    _new_flightmode = new_flightmode;

    return true;
}

uint16_t get_mode_cut(void)
{
    return set_mode_cnt;
}


/***********************************************************
  *@brief  飞行模式更新。
  *@param  new_mode：最新飞行模式的编号
*************************************************************/
void update_flight_mode(int16_t new_mode) //200Hz
{
    switch(new_mode) {
        case ACRO_Mode:

        break;

        case STABILIZE_Mode:
            mode_stabilize_run();         //自稳模式(飞行员控制飞行器角度,手动控制油门)
        break;

        case ALT_HOLD_Mode:
            mode_altitude_hold_run();     //定高模式(飞行员控制飞行器角度,油门中位时高度保持)
        break;

        case POSHOLD_Mode:
            mode_position_hold_run();     //定点模式(外部传感器控制飞行器位置,飞行员不控制时保持位置不变)
        break;

        case FLOWHOLD_Mode:
            mode_flow_hold_run();         //定点模式(光流感器控制飞行器位置,飞行员不控制时保持位置不变)
        break;

        case GUIDED_Mode:
            mode_guided_run();            //指导模式(外部感器控制飞行器位置,可按照设置的航点飞行)
        break;

        case LAND_Mode:
            mode_land_run();              //降落模式(飞行员不控制时可自动降落)
        break;

        case CIRCLE_Mode:
            mode_circle_run();            //画圆模式(飞行员不控制时可自动按照设定的圆形轨迹飞行)
        break;

        default:
        break;
    }
}


/***********************************************************
  *@brief  获取飞行员的摇杆期望,单位为角度,并且做幅值限制。
*************************************************************/
void get_pilot_desired_lean_angles(float *roll_out, float *pitch_out, float angle_max)
{
    float roll_angle,pitch_angle;
    // throttle failsafe check
    if (!get_rc_health()) {
        *roll_out  = 0;
        *pitch_out = 0;
        return;
    }
    ////先简单的扩大到所需角度,之后再精确处理。
    roll_angle = get_channel_roll_control_in()*25;
    pitch_angle = get_channel_pitch_control_in()*25;

    // do lateral tilt to euler roll conversion
    roll_angle = atanf(cosf(pitch_angle*(PI/180))*tanf(roll_angle*(PI/180)))*(180/PI);

    *roll_out  = roll_angle;
    *pitch_out = pitch_angle;
}


// input_expo calculates the expo function on the normalised input.
// The input must be in the range of -1 to 1.
// The expo should be less than 1.0 but limited to be less than 0.95.
float input_expo(float input, float expo)
{
    float _input = limit(input, -1.0, 1.0);
    if (expo < 0.95) {
        return (1 - expo) * _input / (1 - expo * fabsf(_input));
    }
    return _input;
}

/***********************************************************
  *@brief  获取飞行员的偏航期望,单位为角速度,并且做指数化。
  *@param  yaw_in： 1到-1的摇杆量
*************************************************************/
float get_pilot_desired_yaw_rate(float yaw_in)
{
    // throttle failsafe check
    //if(copter.failsafe.radio || !copter.ap.rc_receiver_present) {
    //    return 0.0f;
    //}

    // convert pilot input to the desired yaw rate
    return 200.0 * input_expo(yaw_in, 0.2f);
}


/***********************************************************
  *@brief 得到处理后的油门期望,使手动控制油门时更加容易,范围0 to 1。
*************************************************************/
float get_pilot_desired_throttle()
{
    const float thr_mid = 0.48f;  //悬停油门
    float throttle_control = get_one_way_throttle(); //获取遥控器原始油门,大约范围0到1

    // ensure reasonable throttle values
    throttle_control = limit(throttle_control,0,1);

    // calculate normalised throttle input
    float throttle_in = throttle_control;

    const float expo = limit(-(thr_mid-0.5f)/0.375f,-0.5f, 1.0f);

    // calculate the output throttle using the given expo function
    float throttle_out = throttle_in*(1.0f-expo) + expo*throttle_in*throttle_in*throttle_in;

    return throttle_out;
}

/***********************************************************
  *@brief  获取飞行员的起降期望,单位cm/s。
  *@param  throttle_control： 1到-1的油门摇杆量
*************************************************************/
float get_pilot_desired_climb_rate(float throttle_control)
{
    float climb_rate;
    ////先简单的扩大到所需速度,之后再精确处理。
    climb_rate = throttle_control*100;     //单位 cm/s
    return climb_rate;
}











