/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-04-07     CGY       the first version
 */
#ifndef APPLICATIONS_AUTOPILOT_LIBRARIES_DRIVERLIB_OPTICALFLOWSENSOR_DL_T265_H_
#define APPLICATIONS_AUTOPILOT_LIBRARIES_DRIVERLIB_OPTICALFLOWSENSOR_DL_T265_H_


#include "loco_config.h"

typedef struct
{
    unsigned int  len;
    unsigned char buf[512];
}_Vio_Rx;


typedef union{
    float fvalue;
    unsigned char cv[4];
}float_union;

typedef struct {
 uint64_t usec; /*< [us] Timestamp (UNIX time or time since system boot)*/
 float pos_x; /*< [m] Local X position*/
 float pos_y; /*< [m] Local Y position*/
 float pos_z; /*< [m] Local Z position*/

 float vel_x; /*< [m] Local X position*/
 float vel_y; /*< [m] Local Y position*/
 float vel_z; /*< [m] Local Z position*/

 float roll; /*< [rad] Roll angle*/
 float pitch; /*< [rad] Pitch angle*/
 float yaw; /*< [rad] Yaw angle*/
 float covariance[21]; /*<  Row-major representation of pose 6x6 cross-covariance matrix upper right triangle (states: x, y, z, roll, pitch, yaw; first six entries are the first ROW, next five entries are the second ROW, etc.). If unknown, assign NaN value to first element in the array.*/
 uint8_t reset_counter; /*<  Estimate reset counter. This should be incremented when the estimate resets in any of the dimensions (position, velocity, attitude, angular speed). This is designed to be used when e.g an external SLAM system detects a loop-closure and the estimate jumps.*/
}vision_estimate_t;

extern vision_estimate_t  t265_estimate,t265_target;
extern int rudder_flag;
extern int postion_flag;
extern int into_postion;
extern int32_t last_t265_update_ms;
void T265_data_deal(_Vio_Rx rx);
void send_take_off_flag_to_cv(void);
void send_cmd_start_t265(void);
void send_cmd_restart_t265(void);
void t265_data_Receive(uint8_t data);
bool get_t265_health(void);
bool is_t265_landFlag(void);
#endif /* APPLICATIONS_AUTOPILOT_LIBRARIES_DRIVERLIB_OPTICALFLOWSENSOR_DL_T265_H_ */
