/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-04-05     CGY       the first version
 */
//遥控器数据分析
#include "usb_transfer.h"
#include <rtthread.h>
#include "loco_config.h"
#include "CL_AHRS.h"
#include "mode_all.h"
#include "CL_RC_Channel.h"
#include "rc_copter.h"
//enum FlightMode {
//    STABILIZE_Mode =     0,  // manual airframe angle with manual throttle
//    ACRO_Mode =          1,  // manual body-frame angular rate with manual throttle
//    ALT_HOLD_Mode =      2,  // manual airframe angle with automatic throttle
//    AUTO_Mode =          3,  // fully automatic waypoint control using mission commands
//    GUIDED_Mode =        4,  // fully automatic fly to coordinate or fly at velocity/direction using GCS immediate commands
//    LOITER_Mode =        5,  // automatic horizontal acceleration with automatic throttle
//    RTL_Mode =           6,  // automatic return to launching point
//    CIRCLE_Mode =        7,  // automatic circular flight with automatic throttle
//    LAND_Mode =          9,  // automatic landing with horizontal position control
//    DRIFT_Mode =        11,  // semi-autonomous position, yaw and throttle control
//    SPORT_Mode =        13,  // manual earth-frame angular rate control with manual throttle
//    FLIP_Mode =         14,  // automatically flip the vehicle on the roll axis
//    AUTOTUNE_Mode =     15,  // automatically tune the vehicle's roll and pitch gains
//    POSHOLD_Mode =      16,  // automatic position hold with manual override, with automatic throttle
//    BRAKE_Mode =        17,  // full-brake using inertial/GPS system, no pilot input
//    THROW_Mode =        18,  // throw to launch mode using inertial/GPS system, no pilot input
//    AVOID_ADSB_Mode =   19,  // automatic avoidance of obstacles in the macro scale - e.g. full-sized aircraft
//    GUIDED_NOGPS_Mode = 20,  // guided mode but only accepts attitude and altitude
//    SMART_RTL_Mode =    21,  // SMART_RTL returns to home by retracing its steps
//    FLOWHOLD_Mode  =    22,  // FLOWHOLD holds position with optical flow without rangefinder
//    FOLLOW_Mode    =    23,  // follow attempts to follow another vehicle or ground station
//    ZIGZAG_Mode    =    24,  // ZIGZAG mode is able to fly in a zigzag manner with predefined point A and point B
//    SYSTEMID_Mode  =    25,  // System ID mode produces automated system identification signals in the controllers
//    AUTOROTATE_Mode =   26,  // Autonomous autorotation
//    AUTO_RTL_Mode =     27,  // Auto RTL, this is not a true mode, AUTO will report as this mode if entered to perform a DO_LAND_START Landing sequence
//    TURTLE_Mode =       28,  // Flip over after crash
//    };
//飞行模式编号如上。使用方法为：例如位置pos0所对应的是飞行模式为1， 三段开关时：低位时的位置为pos0、中位为pos3、高位为pos5
//摇杆位置编号：                                                   pos0  pos1  pos2  pos3  pos4  pos5
uint8_t FlightModeProtocol[6]={2,    4,    4,   22,    5,    0};   //默认的飞行模式

uint16_t get_flight_modes(uint8_t position)//获取摇杆位置的期望模式
{
    static uint16_t mode_name = 0;

    switch(position) {
        case 0:
            mode_name = FlightModeProtocol[0];
        break;
        case 1:
            mode_name = FlightModeProtocol[1];
        break;
        case 2:
            mode_name = FlightModeProtocol[2];
        break;
        case 3:
            mode_name = FlightModeProtocol[3];
        break;
        case 4:
            mode_name = FlightModeProtocol[4];
        break;
        case 5:
            mode_name = FlightModeProtocol[5];
        break;
        default:
        break;
    }
    return mode_name;
}


void mode_switch_changed(int8_t new_pos)   //通过遥控器的拨码开关设定飞行模式
{
    set_flight_mode(get_flight_modes(new_pos),RC_COMMAND); //设定飞行模式
}


void read_mode_switch() //读取遥控器开关位置并设置飞行模式
{
    int8_t position=0;
    if (read_6pos_switch(&position)) {
        //set flight mode and simple mode setting
        mode_switch_changed(position);
    }
}




